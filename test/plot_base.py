import cv2
import numpy as np
import math
import parse
import sys

SCRREN_SIZE = 1500
METER2PIX = 1
BASE_ORIGIN = 1
BASE_1SECOND = 2
window_name = 'Bobik'
image = np.zeros((SCRREN_SIZE,SCRREN_SIZE,3), np.uint8)


def ros2cv(point):
    x = int(point[1]) + SCRREN_SIZE/2
    y = -1 * int(point[0]) + SCRREN_SIZE/2
    return (int(x), int(y))

def draw_base(A, B, C, type):
    global image

    if type == 1:
        color = (90, 120, 90)
        thickness = 3
    if type == 2:
        color = (80, 200, 80)
        thickness = 4
    image = cv2.line(image, ros2cv(A), ros2cv(B), color, thickness)
    image = cv2.line(image, ros2cv(B), ros2cv(C), color, thickness)
    image = cv2.line(image, ros2cv(C), ros2cv(A), color, thickness)
    cv2.circle(image, ros2cv(A), 7, (100,100,200), -1)
    cv2.circle(image, ros2cv(B), 7, (100,200,100), -1)
    cv2.circle(image, ros2cv(C), 7, (200,100,100), -1)


def draw_caster(start, gamma, speed):
    global image
    color = (200, 120, 120)
    thickness = 2

    g = int(gamma) / -1000 # from milliradians
    x = math.sin(g) * int(speed) * METER2PIX
    y = -math.cos(g) * int(speed) * METER2PIX

    (ax, ay) = ros2cv(start)
    ax = int(ax + x)
    ay = int(ay + y)

    image = cv2.arrowedLine(image, ros2cv(start), (ax, ay), color, thickness)




draw_base((131,-228),(131,228),(-263,0), BASE_ORIGIN)

cv2.imshow(window_name, image)
for line in sys.stdin:
    # line = 'test/test_casters.cpp:271:test_hl_fwd:INFO: Base config;0; 400;0;0; 131;-228;100;400; 131;228;-100;400; -263;0;1570;400'
    format_string = '{}Base config;{}; {};{};{}; {};{};{};{}; {};{};{};{}; {};{};{};{}'
    # data_str = line.split('config;')[1]
    # data = data_str.split(';')
    data = parse.parse(format_string, line)
    print(data)
    if data:
        _, frame_id, cmd_x, cmd_y, cmd_gamma,  ax,ay,ag,aspd,  bx,by,bg,bspd,  cx,cy,cg,cspd = data.fixed
        draw_base((ax,ay),(bx,by),(cx,cy), BASE_1SECOND)
        draw_caster((ax,ay), ag, aspd)
        draw_caster((bx,by), bg, bspd)
        draw_caster((cx,cy), cg, cspd)

        cv2.imshow(window_name, image) 
        cv2.waitKey(1)

cv2.waitKey(50000)
