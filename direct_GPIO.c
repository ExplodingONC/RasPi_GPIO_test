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
#define FV_PIN 0x00000001
#define LV_PIN 0x00000002
#define PCLK_PIN 0x00000004

// global vars?
volatile uint32_t *base = 0;

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

int single_read(void)
{
    return base[GPLEV0];
}

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
        if ((GPIO_level & FV_PIN) && (GPIO_level & LV_PIN))
        {
            // detect PCLK signal
            if (!clock_tick && (GPIO_level & PCLK_PIN))
            {
                clock_tick = 1;
                GPIO_result[i] = GPIO_level;
                ++i;
            }
            else if (!(0))
            {
                clock_tick = 0;
            }
        }
    }
    return GPIO_result;
}

uint32_t *read_area_frame(int width, int height) // include header bytes in "width"
{
    // read GPIO level reg
    int clock_tick = 0;
    int frame_valid = 0;
    int total_size = 4 * width * height; // 1 frame contains 4 sub-frames
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
        if ((GPIO_level & FV_PIN))
        {
            frame_valid = 1;
        }
    }
    // capture on-going frame
    for (int i = 0; i < total_size;)
    {
        GPIO_level = base[GPLEV0];
        // detect FV signal
        if (!(GPIO_level & FV_PIN))
        {
            frame_valid = 0;
            break;
        }
        // detect LV signal
        if ((GPIO_level & LV_PIN))
        {
            // detect PCLK signal
            if (!clock_tick && (GPIO_level & PCLK_PIN))
            {
                clock_tick = 1;
                GPIO_result[i] = GPIO_level;
                ++i;
            }
            else if (!(0))
            {
                clock_tick = 0;
            }
        }
    }
    return GPIO_result;
}
