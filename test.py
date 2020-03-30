import numpy as np
import matplotlib.pyplot as plt

def exprate(t, tau):
    return 1 - np.exp(-tau*t)

def sigmoid(t, a, b, c):
    s = a
    s = a / (1 + np.exp(- b * (t - c)))
    return s

def gompertz(t, a, b, c, d):
    s = a*np.exp(-b*np.exp(-c*(t-d)))
    return s

t = np.linspace(0, 20, 1000)
app = [0.15, 0.3, 0.5]
res = np.empty((0, 3))
for tt in t:
    # r = rate(t, i)
    # r = sigmoid(tt, 1, 1, 8)
    r1 = gompertz(tt, 5, 4, app[0], 6)
    r2 = gompertz(tt, 5, 4, app[1], 6)
    r3 = gompertz(tt, 5, 4, app[2], 6)
    res = np.append(res, np.array([[r1, r2, r3]]), axis=0)

fig = plt.figure()
plt.plot(t, res[:, 0], label=str(app[0]))
plt.plot(t, res[:, 1], label=str(app[1]))
plt.plot(t, res[:, 2], label=str(app[2]))
plt.legend()
plt.show()

t = np.linspace(0, 1, 1000)
b = [1, 5, 10]
sig = np.empty((0, 3))
for tt in t:
    # r = rate(t, i)
    # r = sigmoid(tt, 1, 1, 8)
    r1 = sigmoid(tt, 3, b[0], 0.5)
    r2 = sigmoid(tt, 3, b[1], 0.5)
    r3 = sigmoid(tt, 3, b[2], 0.5)
    sig = np.append(sig, np.array([[r1, r2, r3]]), axis=0)

fig = plt.figure()
plt.plot(t, sig[:, 0], label=str(b[0]))
plt.plot(t, sig[:, 1], label=str(b[1])) 
plt.plot(t, sig[:, 2], label=str(b[2]))
plt.legend()
plt.show()
