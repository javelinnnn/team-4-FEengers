#import the necessary libraries
import sensor
import time
import math
from machine import UART

uart = UART(3, 9600)
#These are the colour tracking thresholds. The LAB Color Space is used and these have to be tuned
# at different venues
thresholds = [
    (25, 70, 35, 60, 15, 40),  # threshold for red traffic sign
    (18, 30, -30, 0, 0, 10),  # threshold for green traffic sign
]
#The above values represent minimum L, maximum L, minimum A, maximum A, minimum B, maximum B

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold"(set to 200) and more area than
# "area_threshold"(also set to 200) are returned by "find_blobs" below

while True:
    clock.tick()
    img = sensor.snapshot()
    data = [] # list where information is stored
    for blob in img.find_blobs(thresholds, pixels_threshold=200, area_threshold=200):
        # These values depend on the blob not being circular - otherwise they will be shaky.

if blob.code() == 1: # if red traffic sign is detected
            img.draw_rectangle(blob.rect(), (255,0,0)) # a red rectangle will be drawn around it
        elif blob.code() == 2: # if the green traffic sign is detected
            img.draw_rectangle(blob.rect(), (0,255,0)) # a green rectangle will be drawn
        img.draw_cross(blob.cx(), blob.cy()) #draw a cross at the centroid x and centroid y of blob
        data.append([blob.w(), blob.code()]) # append the width and colour of blob to list "data"
        # Note - the blob rotation is unique to 0-180 only.
#        img.draw_keypoints(
#            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
#        )
    print(data)
    code = 0
    max_w = 0
    for b in data:
        if b[0] > max_w:
            max_w = b[0]
            code = b[1]
    uart.write(str(code).encode() + b'\n') #converts "code" to bytes to be sent to ESP32
