import numpy as np
from math import pi, cos, sin

while True:
    alpha, beta, gamma = int(input()) / 180 * pi, int(input()) / 180 * pi, int(input()) / 180 * pi
    x_to_origin = np.array([[1, 0, 0, 0],
                   [0, cos(-alpha), -sin(alpha), 0],
                   [0, sin(alpha), cos(alpha), 0],
                   [0, 0, 0, 1]])
    z_to_origin = np.array([
        [cos(-gamma), -sin(-gamma), 0, 0],
        [sin(-gamma), cos(-gamma) ,0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])
    ori = x_to_origin @ z_to_origin
    print(ori)
    print(ori @ np.array([[alpha], [beta], [gamma], [1]]))