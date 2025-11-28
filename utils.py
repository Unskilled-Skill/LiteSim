# utils.py
import math
import numpy as np
import queue

class QueueRedirector:
    def __init__(self, q):
        self.q = q
    def write(self, string):
        self.q.put(string)
    def flush(self): pass

def normalize_angles(angles_deg):
    normalized = []
    for a in angles_deg:
        a = (a + 180) % 360 - 180
        normalized.append(a)
    return normalized

def rpy_to_matrix(roll, pitch, yaw):
    alpha, beta, gamma = math.radians(roll), math.radians(pitch), math.radians(yaw)
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    cg, sg = math.cos(gamma), math.sin(gamma)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    Rz = np.array([[cg, -sg, 0], [sg, cg, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx