"""This Module Tracks (kind of)"""
import os
import numpy as np
import cv2
import csv
import time
from pynput.keyboard import Key, Controller

# set working directory
os.chdir('C:\\Users\\Timothy Lee\\Pictures\\183DA Lab3')

# define constants
GBOUNDARY = [([90, 130, 90], [200, 200, 200])]
PBOUNDARY = [([60, 20, 60], [140, 40, 180])]
IMG_WIDTH = 640
IMG_LENGTH = 480
BOX_X_LOWER = 20
BOX_X_UPPER = 600
BOX_Y_LOWER = 50
BOX_Y_UPPER = 470
BOX_WIDTH_CM = 63
BOX_LENGTH_CM = 45
FONT = cv2.FONT_HERSHEY_PLAIN
TIME_INTERVAL_S = 0.1

#csv command format is ['command', time], must be terminated with a stop


def main():
    """main baby"""
    # clear log
    f = open('log.csv', "w+")

    # add table headers to log
    with open('log.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["time(ms)", "x(cm)", "y(cm)", "theta(rad)"])

    # load csv data into numpy matrix
    with open('cmd.csv', 'r') as file:
        reader = csv.reader(file, delimiter=',')
        cmd = np.array(list(reader))

    # begin video capture
    cap = cv2.VideoCapture(0)

    # start time, wait for 's' for data capture, wait for 'c' for cmds
    prev_time = time.monotonic()

    # init local variables
    c_time = 0
    time_increment = 0
    start_data = False
    start_cmd = False
    cmd_line = 0;
    last_cmd = 'S'
    keyboard = Controller()

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # draw box bound lines, check for calibration
        draw_box(frame)

        # extract Bounds
        g_lower = np.array(GBOUNDARY[0][0], dtype="uint8")
        g_upper = np.array(GBOUNDARY[0][1], dtype="uint8")

        p_lower = np.array(PBOUNDARY[0][0], dtype="uint8")
        p_upper = np.array(PBOUNDARY[0][1], dtype="uint8")

        # create and overlay masks
        g_mask = cv2.inRange(frame, g_lower, g_upper)
        p_mask = cv2.inRange(frame, p_lower, p_upper)
        mask = cv2.bitwise_or(p_mask, g_mask)
        output = cv2.bitwise_and(frame, frame, mask=mask)

        # create visualization from data
        gx, gy = get_xy(g_mask)
        px, py = get_xy(p_mask)
        theta = get_angle(gx, gy, px, py)
        vis = get_vis(gx, gy, px, py, theta)
        draw_box(vis)

        # concatenate images
        temp = cv2.imread('dirthouse.PNG')  # placeholder image
        viewer = np.vstack([np.hstack([temp, output]), np.hstack([frame, vis])])

        # at every time interval, log data
        if start_data:
            viewer = cv2.putText(viewer, "logging data...", (25, 130), FONT, 10, (255, 255, 255))
        if time.monotonic() - prev_time >= TIME_INTERVAL_S:
            time_increment += TIME_INTERVAL_S*1000
        if time.monotonic() - prev_time >= TIME_INTERVAL_S and start_data:
            # gather data
            print("logging")
            x, y = pix2cm(gx, gy)
            x_str = np.array2string(x)
            y_str = np.array2string(y)
            theta_str = np.array2string(theta)
            time_str = str(time_increment)

            # input into log
            with open('log.csv', 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([time_str, x_str, y_str, theta_str])
            # increment current time
            prev_time = time.monotonic()
        
        # commands are sent through 192.168.4.1 keyboard interface
        if start_cmd and cmd_line < np.shape(cmd)[0]:
            # handle commands from cmd.csv
            if time.monotonic() - c_time >= cmd[cmd_line][1].astype(np.float):
                c_time = time.monotonic()
                if cmd[cmd_line][0] == 'F':
                    keyboard.release(last_cmd)
                    keyboard.press(Key.up)
                    last_cmd = Key.up
                elif cmd[cmd_line][0] == 'B':
                    keyboard.release(last_cmd)
                    keyboard.press(Key.down)
                    last_cmd = Key.down
                elif cmd[cmd_line][0] == 'R':
                    keyboard.release(last_cmd)
                    keyboard.press(Key.right)
                    last_cmd = Key.right
                elif cmd[cmd_line][0] == 'L':
                    keyboard.release(last_cmd)
                    keyboard.press(Key.left)
                    last_cmd = Key.left
                elif cmd[cmd_line][0] == 'S':
                    keyboard.release(last_cmd)
                    last_cmd = 'S'
                cmd_line += 1
            viewer = cv2.putText(viewer, "sending cmds!", (25, 260), FONT, 10, (255, 255, 255))
        elif cmd_line >= np.shape(cmd)[0]:
            keyboard.release(last_cmd)
            start_cmd = False
            cmd_line = 0

        # Display the resulting frame and visual
        cv2.imshow('masked frame', viewer)

        # Wait for command
        inp = cv2.waitKey(1) & 0xFF
        if inp == ord('s'):
            start_data = not start_data
        elif inp == ord('c'):
            start_cmd = True
            time.sleep(1)
        elif inp == ord('q'):
            break

    # When everything done, release the captureq
    cap.release()
    cv2.destroyAllWindows()

def get_xy(color):
    """calculate and return position of dot of given color"""

    #find nonzero indicies
    ind = np.nonzero(color)
    y = IMG_LENGTH - np.average(ind[0])
    x = np.average(ind[1])

    return x, y

def get_angle(gx, gy, px, py):
    """calculate angle between green and pink points"""

    delta_x = px - gx
    delta_y = py - gy
    radians = np.arctan2(delta_y, delta_x)
    return radians



def get_vis(gx, gy, px, py, theta):
    """create graphic displaying the state data"""

    img = np.zeros((IMG_LENGTH, IMG_WIDTH, 3), dtype="uint8")
    if np.isnan(gx) or np.isnan(gy) or np.isnan(px) or np.isnan(py):
        return img

    # display x, y, theta
    img = cv2.circle(img, (int(gx), IMG_LENGTH - int(gy)), 30, (0, 255, 0), 1)
    img = cv2.circle(img, (int(px), IMG_LENGTH - int(py)), 1, (255, 0, 255), 3)

    line_theta_x = int(25*np.cos(theta) + gx)
    line_theta_y = int(-25*np.sin(theta) + IMG_LENGTH - gy)
    img = cv2.line(img, (int(gx), IMG_LENGTH - int(gy)), (line_theta_x, line_theta_y), (255, 255, 0), 1)

    x, y = pix2cm(gx, gy)
    temp = "x(cm): " + np.array2string(x)
    img = cv2.putText(img, temp, (25, 25), FONT, 0.8, (255, 255, 255))
    temp = "y(cm): " + np.array2string(y)
    img = cv2.putText(img, temp, (25, 45), FONT, 0.8, (255, 255, 255))
    temp = "theta(deg): " + np.array2string(theta*(180/np.pi))
    img = cv2.putText(img, temp, (25, 65), FONT, 0.8, (255, 255, 255))
    return img

def draw_box(frame):
    """Display box bounds being used for mapping"""
    # top
    frame = cv2.line(frame, (BOX_X_LOWER, IMG_LENGTH - BOX_Y_UPPER), \
                            (BOX_X_UPPER, IMG_LENGTH - BOX_Y_UPPER), (255, 255, 255), 1)
    # bottom
    frame = cv2.line(frame, (BOX_X_LOWER, IMG_LENGTH - BOX_Y_LOWER), \
                            (BOX_X_UPPER, IMG_LENGTH - BOX_Y_LOWER), (255, 255, 255), 1)
    # left
    frame = cv2.line(frame, (BOX_X_LOWER, IMG_LENGTH - BOX_Y_UPPER), \
                            (BOX_X_LOWER, IMG_LENGTH - BOX_Y_LOWER), (255, 255, 255), 1)
    # right
    frame = cv2.line(frame, (BOX_X_UPPER, IMG_LENGTH - BOX_Y_UPPER), \
                            (BOX_X_UPPER, IMG_LENGTH - BOX_Y_LOWER), (255, 255, 255), 1)

def pix2cm(x, y):
    """Use box bounds to convert pixel positions to centimeters from wall"""

    x_conv = BOX_WIDTH_CM/(BOX_X_UPPER - BOX_X_LOWER)
    y_conv = BOX_LENGTH_CM/(BOX_Y_UPPER - BOX_Y_LOWER)

    x_cm = x*x_conv
    y_cm = y*y_conv

    return x_cm, y_cm

main()
