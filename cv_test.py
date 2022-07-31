# import system modules
import os
import sys
import screeninfo
# import project modules
import scipy.constants
import numpy
import cv2
from skimage import io

# target display No.
target_DISPLAY_env = ":0"
# use physical display
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
target_monitor = 0
try:
    monitors = screeninfo.get_monitors()
    print("Total screen count:", numpy.size(monitors))
    for monitor in monitors:
        print(monitor)
    if target_monitor > numpy.size(monitors) - 1:
        print("Target display index is not available!")
        sys.exit()
except:
    print("No display is attached!")
    sys.exit()
print()

# create image window
cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.moveWindow("Image", monitors[target_monitor].x, monitors[target_monitor].y)  # move to target screen before full screen
cv2.setWindowProperty("Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
#cv2.resizeWindow("Image", monitors[target_monitor].width, monitors[target_monitor].height)
print("Image window ready.")

# display HTTP cats
while True:
    http_code = input("Select an HTTP status code: ")
    if "\x1b" in http_code:
        print("Found escape code, exiting...")
        break
    try:
        image = io.imread(f"https://http.cat/{http_code}.jpg")  # RGB order
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    except:
        print(f"{http_code} is probably not an HTTP status code.")
    else:
        cv2.imshow("Image", image)  # BGR order
        cv2.waitKey(1)

cv2.destroyAllWindows()
