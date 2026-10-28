"""Map a desired Cube transform to the frozen G1 nominal suction TCPs.

Pure CPU kinematics; this does not establish contact, qualify dynamics, solve
an online HQP, or authorize actuation.
"""
import numpy as np
from offline.task28_g1_1.task28_validator.closed_chain import desired_tcps, evaluate_closure
from offline.task28_g1_1.task28_validator.se3 import checked_transform


def desired_suction_frames(cube_world):
    cube = checked_transform(cube_world)
    left, right = desired_tcps(cube)
    return {'left': left, 'right': right}


def frozen_closure_gate(cube_world, left_tcp_world, right_tcp_world):
    return evaluate_closure(cube_world, left_tcp_world, right_tcp_world)
