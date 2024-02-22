import math
def deg2rad(deg):
    return [math.radians(degree) for degree in deg]


def rad2deg(radians):
    # Convert radians to degrees for each joint value
    return [math.degrees(rad) for rad in radians]


def swap_order(i, j, k):
    i[j], i[k] = i[k], i[j]
    return i

def reverse_sign(i, j):
    i[j] = -i[j]
    return i

