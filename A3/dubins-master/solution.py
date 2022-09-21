#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Jannik Wagner}
# {19971213-1433}
# {wagne@kth.se}

import random
from dubins import *
import numpy as np
import math

PHI_OPTIONS = [-math.pi/4, 0, math.pi/4]

EPSILON = 10**-2


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


def target_distance(car: Car, x, y):
    return math.sqrt((car.xt - x)**2 + (car.yt - y)**2)


def is_at_target(car: Car, x, y):
    THRESHOLD = 1.5
    return target_distance(car, x, y) < THRESHOLD


def get_sample(car: Car):
    # TODO: add bias towards goal
    x = random.random() * (car.xub - car.xlb) + car.xlb
    y = random.random() * (car.yub - car.ylb) + car.ylb
    return x, y


def RRT(car: Car):
    # initial state
    x, y = car.x0, car.y0
    theta = 0

    edges = []
    nodes = [(x, y, theta)]
    while True:
        x_s, y_s = get_sample(car)
        i_n = find_nearest_point(edges, nodes, x_s, y_s)
        c_n = get_controls(*nodes[i_n], x_s, y_s)
        # TODO: check if controls are possible
        # TODO: add node and edge


def find_nearest_point(edges, points, x_s, y_s):
    raise NotImplementedError()


def get_controls(x_n, y_n, theta_n, x_s, y_s):
    raise NotImplementedError()
