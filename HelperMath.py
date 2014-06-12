import numpy as np

def mag(v):
    return np.linalg.norm(v)
def norm(v):
    return v/mag(v)
def pts_to_line(pt1, pt2):
    try: 
        slope = (float(pt2[1]) - float(pt1[1])) / (float(pt2[0]) - float(pt1[0]))
        a = -slope
        b = 1
        c = slope*pt1[0] - pt1[1]
    except ZeroDivisionError:
        b = 0
        a = 1
        c = -pt2[0]
    return a,b,c
def distance_to_line(pt, line):
    """Line is a tuple of a,b,c where ax + by + c = 0"""
    a,b,c = line
    return abs(a*pt[0] + b*pt[1] + c) / np.sqrt(a**2 + b**2)