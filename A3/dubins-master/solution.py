#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Jannik Wagner}
# {19971213-1433}
# {wagne@kth.se}

import random
from dubins import *
import numpy as np
import math

PHI_MIN = -math.pi/4
PHI_MAX = math.pi/4
PHI_OPTIONS = [PHI_MIN, 0, PHI_MAX]

EPSILON = 10**-2

V = 0.01
V_ANG = 0.01
T_ROT = math.pi*2*100


def solution(car):
    ''' <<< write your code below >>> '''

    nodes, edges = RRT(car)
    tcontrols = graph_to_solution(edges)
    controls, times = get_controls_and_times(tcontrols)

    ''' <<< write your code below >>> '''

    return controls, times


def is_illegal(car: Car, x, y):
    return is_outside(car, x, y) or has_collision(car, x, y)


def is_outside(car: Car, x, y):
    return not (car.xlb <= x <= car.xub and car.ylb <= y <= car.yub)


def has_collision(car: Car, x, y):
    return any(is_within_obstacle(obs, x, y) for obs in car.obs)


def is_within_obstacle(obs, x, y):
    x_obs, y_obs, r_obs = obs
    return math.sqrt((x_obs - x)**2 + (y_obs - y)**2) < r_obs


def target_distance(car: Car, x, y):
    return euclidean(car.xt, car.yt, x, y)


def euclidean(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def is_at_target(car: Car, x, y):
    THRESHOLD = 1.5
    return target_distance(car, x, y) < THRESHOLD-EPSILON


def get_sample(car: Car):
    if random.random() < 0.1:  # add bias towards goal
        x, y = car.xt, car.yt
    else:
        x = random.random() * (car.xub - car.xlb) + car.xlb
        y = random.random() * (car.yub - car.ylb) + car.ylb
    theta = random.random() * math.pi*2
    return x, y, theta


def RRT(car: Car):
    # initial state
    x, y = car.x0, car.y0
    theta = 0

    edges = [(None, None)]  # parent index, tcontrols to get here from parent
    nodes = [(x, y, theta)]

    while True:
        x_s, y_s, theta_s = get_sample(car)
        i_c = find_closest_point(nodes, x_s, y_s, theta_s)
        tcontrols = get_controls(*nodes[i_c], x_s, y_s, theta_s)

        x, y, theta = nodes[i_c]
        new_tcontrols, x, y, theta = apply_controls_1(
            car, x, y, theta, tcontrols)

        # print(new_tcontrols, x, y, theta)
        nodes.append((x, y, theta))
        edges.append((i_c, new_tcontrols))

        if is_at_target(car, x, y):
            return nodes, edges


def graph_to_solution(edges):
    parent = len(edges)-1
    tcontrols = []
    while parent is not None:
        edge = edges[parent]
        parent, edge_controls = edge
        tcontrols = edge_controls + tcontrols
    return tcontrols


def get_controls_and_times(tcontrols):
    time = 0
    times = [0]
    controls = []
    for phi, dt in tcontrols:
        controls.append(phi)
        time += dt
        times.append(time)
    return controls, times


def apply_controls_1(car: Car, x, y, theta, tcontrols):
    new_tcontrols = []
    stop_flag = False
    for phi, t in tcontrols:
        for i in range(round(100*t)):
            x, y, theta = step(car, x, y, theta, phi)
            if is_illegal(car, x, y):  # TODO: add safety margin
                stop_flag = True
                new_tcontrols.append((phi, (i+1)/100))
                break
        if stop_flag:
            break
        new_tcontrols.append((phi, t))
    return new_tcontrols, x, y, theta


def apply_controls_2(car: Car, x, y, theta, tcontrols):
    new_flat_controls = []
    flat_controls = flatten_tcontrols(tcontrols)
    for phi in flat_controls:
        x, y, theta = step(car, x, y, theta, phi)
        if is_illegal(car, x, y):  # TODO: add safety margin
            new_flat_controls.append(phi)
            break
    new_tcontrols = unflatten_tcontrols(new_flat_controls)
    return new_tcontrols, x, y, theta


def flatten_tcontrols(tcontrols):
    flat_controls = []
    for phi, t in tcontrols:
        new_controls += [phi] * round(t*100)
    return flat_controls


def unflatten_tcontrols(flat_controls):
    tcontrols = []
    previos_phi = flat_controls[0]
    counter = 1
    for phi in flat_controls[1:]:
        if phi == previos_phi:
            counter += 1
        else:
            tcontrols.append((previos_phi, counter/100))
            counter = 1
            previos_phi = phi
    tcontrols.append((previos_phi, counter/100))
    return tcontrols


def find_closest_point(nodes, x_s, y_s, theta_s):
    # TODO: make more efficient, e.g. kdtrees
    distances = {i: euclidean(nodes[i][0], nodes[i][1], x_s, y_s)
                 for i in range(len(nodes))}
    return min(distances, key=lambda x: distances[x])


def get_controls(x_c, y_c, theta_c, x_s, y_s, theta_s):
    raise NotImplementedError()
    phi1, t1 = PHI_MAX, 1
    phi2, t2 = 0, 1
    phi3, t3 = PHI_MIN, 1

    return (phi1, t1), (phi2, t2), (phi3, t3)


def get_controls_2(car: Car, x_c, y_c, theta_c, x_s, y_s, theta_s):
    raise NotImplementedError()
    phi1, t1 = PHI_MAX, 1
    phi2, t2 = 0, 1
    phi3, t3 = PHI_MIN, 1

    return (phi1, t1), (phi2, t2), (phi3, t3)


def fin_out_v():
    car = Car()

    # initial state
    x, y = car.x0, car.y0
    theta = 0
    print(x, y, theta/math.pi)
    # arbitrary control
    phi = PHI_MAX
    print("###########")
    print("###########")
    print("###########")

    for i in range(629):
        # compute next state after 0.01 seconds
        x, y, theta = step(car, x, y, theta, phi)
        print(x, y, theta/math.pi)

    V = 0.01
    V_ANG = 0.01
    T_ROT = math.pi*2*100


if __name__ == "__main__":
    pass
