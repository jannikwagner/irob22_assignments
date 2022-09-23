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
STEPSIZE = 0.01

V = 0.01
V_ANG = 0.01
T_ROT = math.pi*2


def solution(car):
    ''' <<< write your code below >>> '''

    nodes, edges = RRT(car)
    tcontrols = graph_to_solution(edges)
    controls, times = tcontrols_to_controls(tcontrols)

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

        x, y, theta = nodes[i_c]
        success, local_plan, x_n, y_n, theta_n = local_planner(
            car, x, y, theta, x_s, y_s, theta_s, PHI_MIN)  # turn right
        if not success:
            success, local_plan, x_n, y_n, theta_n = local_planner(
                car, x, y, theta, x_s, y_s, theta_s, PHI_MAX)  # turn left
        if not success:
            continue
        tcontrols, x, y, theta = local_plan, x_n, y_n, theta_n

        print("SAMPLE: ", x_s, y_s, theta_s)
        print("CLOSEST: ", i_c, x, y, theta)
        print("LOCAL PLAN: ", tcontrols, x, y, theta)

        # new_tcontrols, x, y, theta = apply_controls_1(
        #     car, x, y, theta, tcontrols)
        # new_tcontrols = tcontrols

        nodes.append((x, y, theta))
        edges.append((i_c, tcontrols))

        if is_at_target(car, x, y):
            return nodes, edges


def graph_to_solution(edges):
    parent = len(edges)-1
    tcontrols = []
    while parent != 0:
        edge = edges[parent]
        parent, edge_controls = edge
        tcontrols = edge_controls + tcontrols
    return tcontrols


def tcontrols_to_controls(tcontrols):
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
        for i in range(round(t/STEPSIZE)):
            x, y, theta = step(car, x, y, theta, phi, STEPSIZE)
            if is_illegal(car, x, y):  # TODO: add safety margin
                stop_flag = True
                new_tcontrols.append((phi, (i+1)*STEPSIZE))
                break
        if stop_flag:
            break
        new_tcontrols.append((phi, t))
    return new_tcontrols, x, y, theta


def apply_controls_2(car: Car, x, y, theta, tcontrols):
    new_flat_controls = []
    flat_controls = flatten_tcontrols(tcontrols)
    for phi in flat_controls:
        x, y, theta = step(car, x, y, theta, phi, STEPSIZE)
        if is_illegal(car, x, y):  # TODO: add safety margin
            new_flat_controls.append(phi)
            break
    new_tcontrols = unflatten_tcontrols(new_flat_controls)
    return new_tcontrols, x, y, theta


def flatten_tcontrols(tcontrols):
    flat_controls = []
    for phi, t in tcontrols:
        new_controls += [phi] * round(t/STEPSIZE)
    return flat_controls


def unflatten_tcontrols(flat_controls):
    tcontrols = []
    previos_phi = flat_controls[0]
    counter = 1
    for phi in flat_controls[1:]:
        if phi == previos_phi:
            counter += 1
        else:
            tcontrols.append((previos_phi, counter*STEPSIZE))
            counter = 1
            previos_phi = phi
    tcontrols.append((previos_phi, counter*STEPSIZE))
    return tcontrols


def find_closest_point(nodes, x_s, y_s, theta_s):
    # TODO: make more efficient, e.g. kdtrees
    distances = {i: euclidean(nodes[i][0], nodes[i][1], x_s, y_s)
                 for i in range(len(nodes))}
    return min(distances, key=lambda x: distances[x])


def local_planner(car: Car, x_c, y_c, theta_c, x_s, y_s, theta_s, phi):
    x, y, theta = x_c, y_c, theta_c
    alpha_old = -math.inf
    alpha_has_decreased_before = False

    dt = 0

    while True:
        x, y, theta = step(car, x, y, theta, phi, STEPSIZE)
        current_direction = (math.cos(theta), math.sin(theta))
        diff_vec = (x_s - x, y_s - y)
        alpha = angle(current_direction, diff_vec)

        if is_illegal(car, x, y):
            return False, [(phi, dt)], x, y, theta

        if alpha_has_decreased_before and alpha > alpha_old:
            break
        if alpha < alpha_old:
            alpha_has_decreased_before = True

        dt += STEPSIZE
        alpha_old = alpha
    tcontrols = [(phi, dt)]

    dist_old = math.inf
    dt = 0
    while True:
        x, y, theta = step(car, x, y, theta, 0, STEPSIZE)
        dist = distance((x, y), (x_s, y_s))

        if is_illegal(car, x, y):
            return False, tcontrols + [(0, dt)], x, y, theta

        if dist > dist_old:
            break

        dt += STEPSIZE
        dist_old = dist
    tcontrols.append((0, dt))

    return True, tcontrols, x, y, theta


def dot(v1, v2):
    return v1[0]*v2[0] + v1[1]*v2[1]


def length(v):
    return math.sqrt(dot(v, v))


def angle(v1, v2):
    return math.acos(dot(v1, v2) / length(v1) / length(v2))


def difference_vector(p1, p2):
    return (p2[0]-p1[0], p2[1]-p2[1])


def distance(p1, p2):
    return length(difference_vector(p1, p2))


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
    T_ROT = math.pi*2/STEPSIZE


if __name__ == "__main__":
    pass
