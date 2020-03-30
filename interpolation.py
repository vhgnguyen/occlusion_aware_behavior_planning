#%%
import numpy as np


def M1(t):
    return np.array([[1, t, t**2], [0, 1, 2*t], [0, 0, 2]])


def M2(t):
    return np.array([[t**4, t**5, t**6],
                     [4*t**3, 5*t**4, 6*t**5],
                     [12*t**2, 20*t**3, 30*t**4]])


def C0123(x0, M1):
    return np.dot(np.linalg.inv(M1), x0)


def C4567(x1, c0123, M1, M2):
    M2_inv = np.linalg.inv(M2)
    a = x1 - np.dot(M1, c0123)
    return np.dot(M2_inv, a)


def septic(t, c0123, c4567):
    m1 = M1(t)
    m2 = M2(t)
    return np.dot(m1, c0123) + np.dot(m2, c4567)


t = 1
x0 = np.array([[1, 2, 1]]).T
s1 = float(x0[0] + x0[1]*t + 0.5*x0[2]*t**2)
v1 = float(x0[1] + x0[2]*t)
x1 = np.array([[s1, v1, 0]]).T

m1 = M1(0)
m2 = M2(t)

c0123 = C0123(x0, m1)
c4567 = C4567(x1, c0123, m1, m2)

print(c4567)


# %%
