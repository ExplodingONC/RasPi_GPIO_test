// includes
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>

// BCM2711 peripheral register offsets
#define BCM2711_PERI_BASE_OFFSET 0xFE000000
#define GPIO_BASE_OFFSET 0x00200000
#define BCM2711_GPIO_REG BCM2711_PERI_BASE_OFFSET + GPIO_BASE_OFFSET

// BCM2711 GPIO register offsets
#define GPFSEL0 0
#define GPFSEL1 1
#define GPFSEL2 2
#define GPFSEL3 3
#define GPFSEL4 4
#define GPFSEL5 5
#define GPSET0 7
#define GPSET1 8
#define GPCLR0 10
#define GPCLR1 11
#define GPLEV0 13
#define GPLEV1 14
#define GPPUD 37
#define GPPUDCLK0 38
#define GPPUDCLK1 39
// BCM2711 has a different mechanism for pin pull-up/down/enable
#define GPPUPPDN0 57 /* Pin pull-up/down for pins 15:0  */
#define GPPUPPDN1 58 /* Pin pull-up/down for pins 31:16 */
#define GPPUPPDN2 59 /* Pin pull-up/down for pins 47:32 */
#define GPPUPPDN3 60 /* Pin pull-up/down for pins 57:48 */

// custom pin usage
#define FV_GPIO 24
#define FV_PIN 0x1 << FV_GPIO
#define LV_GPIO 23
#define LV_PIN 0x1 << LV_GPIO
#define EF_GPIO 10
#define EF_PIN 0x1 << EF_GPIO
#define REN_GPIO 23
#define REN_PIN 0x1 << REN_GPIO
#define PCLK_GPIO 9
#define PCLK_PIN 0x1 << PCLK_GPIO

// output parameter
#define CYCLE_HOLD 5 // cycles held after clock output

// global vars?
volatile uint32_t *base = 0;

/*
 * Acquire the register base address from LinuxOS
 * Input: none
 * Output: success / fail state
 */
int GPIO_reg_setup(void)
{
    // set up GPIO regs
    int fd;
    uint32_t reg_base = BCM2711_GPIO_REG;
    // fisrt check for /dev/gpiomem, else we need root access for /dev/mem
    if ((fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC)) >= 0)
    {
        base = (uint32_t *)mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    }
    else
    {
        if (geteuid())
        {
            printf("Must be root\n");
            return 1;
        }
        if ((fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0)
        {
            printf("Unable to open /dev/mem: %s\n", strerror(errno));
            return 2;
        }
        base = (uint32_t *)mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, reg_base);
    }
    if (base == (uint32_t *)-1)
    {
        printf("mmap (GPIO) failed: %s\n", strerror(errno));
        return 3;
    }
    return 0;
}

/*
 * Set GPIO pull-none/up/down
 * Input: gpio - GPIO number
 *        pull - 0 = none, 1 = up, 2 = down
 * Output: success
 */
int GPIO_set_pull(unsigned int gpio, int pull)
{
    uint32_t reg_offset = GPPUPPDN0 + (gpio / 16);
    uint32_t bit_offset = (gpio % 16) * 2;
    base[reg_offset] = (base[reg_offset] & ~(3 << bit_offset)) | (pull << bit_offset);
    return 0;
}

/*
 * Set GPIO function select
 * Input: gpio - GPIO number
 *        fsel - 0 = FUNC_IP, 1 = FUNC_OP, 2 = FUNC_A5, 3 = FUNC_A4,
 *               4 = FUNC_A0, 5 = FUNC_A1, 6 = FUNC_A2, 7 = FUNC_A3
 * Output: success
 */
int GPIO_set_fsel(unsigned int gpio, int fsel)
{
    // GPFSEL0-5 with 10 sels per reg, 3 bits per sel (bit[29:0] used)
    uint32_t reg_offset = GPFSEL0 + (gpio / 10);
    uint32_t bit_offset = (gpio % 10) * 3;
    base[reg_offset] = (base[reg_offset] & ~(0x7 << bit_offset)) | (fsel << bit_offset);
    return 0;
}

/*
 * Read once from GPIO bank0
 * Input: none
 * Output: GPIO bank0
 */
int single_read(void)
{
    return base[GPLEV0];
}

/*
 * Read data stream from GPIO bank0 (sync to PCLK)
 * Input: length - data length
 * Output: array of GPIO bank0
 */
uint32_t *burst_read(int length)
{
    // read GPIO level reg
    int clock_tick = 0;
    uint32_t GPIO_level;
    uint32_t *GPIO_result = (uint32_t *)malloc(sizeof(uint32_t) * length);
    for (int i = 0; i < length;)
    {
        GPIO_level = base[GPLEV0];
        // detect FV and LV signal
        if (1 || (GPIO_level & FV_PIN) && (GPIO_level & LV_PIN))
        {
            // detect PCLK signal
            if (!clock_tick && (1 || (GPIO_level & PCLK_PIN)))
            {
                clock_tick = 1;
                GPIO_result[i] = GPIO_level;
                i++;
            }
            else if (1 || !(GPIO_level & PCLK_PIN))
            {
                clock_tick = 0;
            }
        }
    }
    return GPIO_result;
}

/*
 * Read data stream from GPIO bank0 (sync to PCLK)
 * Input: width - frame width
 *        height - frame length
 * Output: 1D array of GPIO bank0
 */
uint32_t *read_area_frame(int width, int height) // include header bytes in "width"
{
    // read GPIO level reg
    int clock_tick = 0;
    int frame_valid = 0;
    int total_size = 8 * width * height; // 1 frame contains 4 sub-frames, 1 pixel contains 2 sub-pixels
    uint32_t GPIO_level;
    uint32_t *GPIO_result = (uint32_t *)malloc(sizeof(uint32_t) * total_size);
    // if there is a frame already in progress (FV=1), wait till it ends
    do
    {
        GPIO_level = base[GPLEV0];
    } while (0 && (GPIO_level & FV_PIN));
    // wait untill frame starts (FV=1)
    while (!frame_valid)
    {
        GPIO_level = base[GPLEV0];
        // detect FV signal
        if (1 || (GPIO_level & FV_PIN))
        {
            frame_valid = 1;
        }
    }
    // capture on-going frame
    for (int i = 0; i < total_size;)
    {
        GPIO_level = base[GPLEV0];
        // detect FV signal
        if (0 && !(GPIO_level & FV_PIN))
        {
            frame_valid = 0;
            break;
        }
        // detect LV signal
        if (1 || (GPIO_level & LV_PIN))
        {
            // detect PCLK signal
            if (!clock_tick && (1 || (GPIO_level & PCLK_PIN)))
            {
                clock_tick = 1;
                GPIO_result[i] = GPIO_level;
                ++i;
            }
            else if (1 || !(GPIO_level & PCLK_PIN))
            {
                clock_tick = 0;
            }
        }
    }
    return GPIO_result;
}

/*
 * Read data stream from GPIO bank0 (fifo controlled by PCLK output)
 * Input: width - frame width
 *        height - frame length
 * Output: 1D array of GPIO bank0
 */
uint32_t *read_area_frame_fifo(int width, int height) // include header bytes in "width"
{
    GPIO_set_pull(PCLK_GPIO, 0);
    // read GPIO level reg
    int last_word = 1;
    int total_size = 8 * width * height; // 1 frame contains 4 sub-frames, 1 pixel contains 2 sub-pixels
    uint32_t GPIO_level;
    uint32_t *GPIO_result = (uint32_t *)malloc(sizeof(uint32_t) * total_size);
    // capture fifo output
    for (int i = 0; i < total_size;)
    {
        if (last_word)
        {
            // RCLK pulse
            base[GPSET0] = PCLK_PIN;
            for (int tick = 0; tick < CYCLE_HOLD; tick++)
                asm("nop");
            base[GPCLR0] = PCLK_PIN;
            // read #EF flag
            GPIO_level = base[GPLEV0];
            if (1 || (GPIO_level & EF_PIN))
            {
                last_word = 0;
                // set #REN signal
                base[GPCLR0] = REN_PIN;
            }
        }
        else
        {
            // RCLK pulse
            base[GPSET0] = PCLK_PIN;
            for (int tick = 0; tick < CYCLE_HOLD; tick++)
                asm("nop");
            base[GPCLR0] = PCLK_PIN;
            // read data
            GPIO_level = base[GPLEV0];
            // data save
            GPIO_result[i] = GPIO_level;
            i++;
            // detect #EF signal
            if (0 && !(GPIO_level & EF_PIN))
            {
                last_word = 1;
                // clear #REN signal
                base[GPSET0] = REN_PIN;
            }
        }
    }
    return GPIO_result;
}
