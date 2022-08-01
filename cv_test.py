# import system modules
import os
import sys
import time
import screeninfo
import threading
# import project modules
import scipy.constants as const
import scipy.io as sciio
import h5py
import numpy
import cv2
from skimage import io as imgio

# thread tasks


def front_end_task(stop, img):
    # display HTTP cats
    while True:
        http_code = input("Select an HTTP status code: ")
        if "\x1b" in http_code:
            print("Found escape code, exiting...")
            break
        try:
            image = imgio.imread(f"https://http.cat/{http_code}.jpg")  # RGB order
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        except:
            print(f"{http_code} is probably not an HTTP status code.")
        else:
            img[0] = image  # BGR order
        if stop():
            break


def back_end_task(stop, img):
    # display calibration image
    while True:
        for GL in range(0, 256):
            img[1] = cal_lut[:, :, GL]
            time.sleep(0.1)
            if stop():
                break
        if stop():
            break


# get calibration LUT
try:
    # MATLAB v7.3+
    cal_file = h5py.File('cal_profile_v73.mat', 'r')
    cal_lut = cal_file.get('cal_img')  # dimension order is reversed
    cal_lut = numpy.array(cal_lut)
    cal_lut = numpy.moveaxis(cal_lut, [0, 1, 2], [2, 1, 0])
    print("MATLAB 7.3 MAT-file")
except:
    # MATLAB v7-
    mat = sciio.loadmat('cal_profile.mat')
    cal_lut = mat['cal_img']
    print(mat['__header__'].decode("UTF-8").split(",")[0])
finally:
    print("Calibration LUT shape:", cal_lut.shape)
print()

# target the physical display
target_DISPLAY_env = ":0"
current_DISPLAY_env = os.getenv("DISPLAY")
if current_DISPLAY_env != target_DISPLAY_env:
    print(f"Wrong $DISPLAY environment variable: \"{current_DISPLAY_env}\"! Try fixing it...")
    os.environ["DISPLAY"] = target_DISPLAY_env
    # check the env var again
    current_DISPLAY_env = os.getenv("DISPLAY")
    if current_DISPLAY_env != target_DISPLAY_env:
        print(f"Fail to set $DISPLAY environment variable! Current $DISPLAY: \"{current_DISPLAY_env}\".")
        sys.exit()
    else:
        print(f"Successfully set $DISPLAY environment variable: \"{current_DISPLAY_env}\"!")
else:
    print(f"$DISPLAY environment variable: \"{current_DISPLAY_env}\". Continue.")
print()

# check if display is available
try:
    monitors = screeninfo.get_monitors()
    print("Total screen count:", numpy.size(monitors))
    for monitor in monitors:
        print(monitor)
except:
    print("No display is attached!")
    sys.exit()
print()

# allocate monitor functions, returns and threads
frontend_monitor = 0
backend_monitor = 1
img = [None, None]  # list for return imgs
stop_threads = False
try:
    # front end
    if frontend_monitor <= numpy.size(monitors) - 1:
        cv2.namedWindow("Front-end", cv2.WINDOW_NORMAL)
        # move to target screen before full screen
        cv2.moveWindow("Front-end", monitors[frontend_monitor].x, monitors[frontend_monitor].y)
        cv2.setWindowProperty("Front-end", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        # cv2.resizeWindow("Front-end", monitors[frontend_monitor].width, monitors[frontend_monitor].height)
        front_end_thread = threading.Thread(target=front_end_task, args=(lambda: stop_threads, img), daemon=True)
        print("Front end image window ready.")
    else:
        print("Front-end display index is not available!")
        sys.exit()
    # back end
    if backend_monitor <= numpy.size(monitors) - 1:
        cv2.namedWindow("Back-end", cv2.WINDOW_NORMAL)
        # move to target screen before full screen
        cv2.moveWindow("Back-end", monitors[backend_monitor].x, monitors[backend_monitor].y)
        cv2.setWindowProperty("Back-end", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        # cv2.resizeWindow("Back-end", monitors[backend_monitor].width, monitors[backend_monitor].height)
        back_end_thread = threading.Thread(target=back_end_task, args=(lambda: stop_threads, img), daemon=True)
        print("Back end image window ready.")
    else:
        print("Back-end display index is not available!")
        # sys.exit()
except Exception as err:
    print("Error initializing display targets!\r\n", err)
else:
    print()

# start threads
try:
    front_end_thread.start()
except:
    print("Front end not available!")
try:
    back_end_thread.start()
except:
    print("Back end not available!")

# wait for updates
while True:
    if img[0] is not None:
        try:
            cv2.imshow("Front-end", img[0])
            cv2.waitKey(1)
        finally:
            img[0] = None
    if img[1] is not None:
        try:
            cv2.imshow("Back-end", img[1])
            cv2.waitKey(1)
        finally:
            img[1] = None
    if not front_end_thread.is_alive():
        stop_threads = True
        break

front_end_thread.join()
back_end_thread.join()
cv2.destroyAllWindows()
