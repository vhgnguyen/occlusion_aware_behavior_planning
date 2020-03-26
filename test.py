import numpy as np
import matplotlib.pyplot as plt

def exprate(t, tau):
    return 1 - np.exp(-tau*t)

def sigmoid(t, a, b, c):
    s = a
    s = a / (1 + 2 * np.exp( - b * (t - c)))
    return s

def gompertz(t, a, b, c, d):
    s = a*np.exp(-b*np.exp(-c*(t-d)))
    return s

tau = [0.0001, 0.1, 1]
t = np.linspace(0, 20, 1000)

res = np.empty((0, 3))
for tt in t:
    # r = rate(t, i)
    # r = sigmoid(tt, 1, 1, 8)
    r1 = gompertz(tt, 1, 3, 0.3, 6)
    r2 = gompertz(tt, 1, 3, 0.7, 6)
    r3 = gompertz(tt, 1, 3, 0.3, 6)
    res = np.append(res, np.array([[r1, r2, r3]]), axis=0)

fig = plt.figure()
plt.plot(t, res[:, 0])
plt.plot(t, res[:, 1])
plt.plot(t, res[:, 2])

plt.show()
