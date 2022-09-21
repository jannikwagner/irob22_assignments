#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Jannik Wagner}
# {19971213-1433}
# {wagne@kth.se}

from dubins import *
import numpy as np
import math


def solution(car):
    ''' <<< write your code below >>> '''

    # initial state
    x, y = car.x0, car.y0
    theta = 0
    # arbitrary control
    phi = 0.2

    # compute next state after 0.01 seconds
    xn, yn, thetan = step(car, x, y, theta, phi)

    # assemble path
    controls, times = [phi], [0, 0.01]

    # old
    controls = [0]
    times = [0, 1]

    ''' <<< write your code below >>> '''

    return controls, times


def is_illegal(car, x, y):
    return is_outside(car, x, y) or has_collision(car, x, y)


def is_outside(car: Car, x, y):
    return not (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub)


def has_collision(car: Car, x, y):
    return any(is_within_obstacle(obs, x, y) for obs in car.obs)


def is_within_obstacle(obs, x, y):
    x_obs, y_obs, r_obs = obs
    return math.sqrt((x_obs - x)**2 + (y_obs - y)**2) < r_obs
