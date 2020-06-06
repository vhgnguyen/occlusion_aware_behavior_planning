#%%
import numpy as np
import matplotlib.pyplot as plt

def exprate(t, a, b):
    return 1- 1/(1+ a * np.exp(-a* (t-b)))

def sigmoid(t, a, b, c):
    s = a / (1 + np.exp(- b * (t - c)))
    return s

def gompertz(t, a, b, c, d):
    s = a*np.exp(-b*np.exp(-c*(t-d)))
    return s

def interact(a, b, k):
    return 1 - 1 / (1 + np.exp(k * (a-b)))

#%%
t = np.linspace(5, 30, 1000)
app = [0.5, 1, 2]
res = np.empty((0, 3))
for tt in t:
    # r = rate(t, i)
    # r = sigmoid(tt, 1, 1, 8)
    r1 = exprate(tt, app[0], 10)
    r2 = exprate(tt, app[1], 10)
    r3 = exprate(tt, app[2], 10)
    res = np.append(res, np.array([[r1, r2, r3]]), axis=0)

fig = plt.figure()
plt.plot(t, res[:, 0], label="alpha="+str(app[0]))
plt.plot(t, res[:, 1], label="alpha="+str(app[1]))
plt.plot(t, res[:, 2], label="alpha="+str(app[2]))
plt.xlabel("Relative velocity value")
plt.ylabel("Severity")
plt.legend()
plt.show()    


#%%

t = np.linspace(0, 20, 1000)
app = [2, 8, 10]
res = np.empty((0, 3))
for tt in t:
    # r = rate(t, i)
    # r = sigmoid(tt, 1, 1, 8)
    r1 = interact(tt, app[0], 10)
    r2 = interact(tt, app[1], 10)
    r3 = interact(tt, app[2], 10)
    res = np.append(res, np.array([[r1, r2, r3]]), axis=0)

fig = plt.figure()
plt.plot(t, res[:, 0], label="alpha="+str(app[0]))
plt.plot(t, res[:, 1], label="alpha="+str(app[1]))
plt.plot(t, res[:, 2], label="alpha="+str(app[2]))
plt.xlabel("Relative velocity value")
plt.ylabel("Severity")
plt.legend()
plt.show()    


t = np.linspace(0, 20, 1000)
app = [0.1, 0.3, 1]
res = np.empty((0, 3))
for tt in t:
    # r = rate(t, i)
    # r = sigmoid(tt, 1, 1, 8)
    r1 = gompertz(tt, 1, 2, app[2], 5)
    r2 = gompertz(tt, 1, 3, app[2], 5)
    r3 = gompertz(tt, 1, 4, app[2], 5)
    res = np.append(res, np.array([[r1, r2, r3]]), axis=0)

fig = plt.figure()
plt.plot(t, res[:, 0], label="alpha="+str(app[0]))
plt.plot(t, res[:, 1], label="alpha="+str(app[1]))
plt.plot(t, res[:, 2], label="alpha="+str(app[2]))
plt.xlabel("Relative velocity value")
plt.ylabel("Severity")
plt.legend()
plt.show()

t = np.linspace(0, 20, 1000)
b = [1, 3, 7]
sig = np.empty((0, 3))
for tt in t:
    # r = rate(t, i)
    # r = sigmoid(tt, 1, 1, 8)
    r1 = sigmoid(tt, 1, b[0], 8)
    r2 = sigmoid(tt, 1, b[1], 8)
    r3 = sigmoid(tt, 1, b[2], 8)
    sig = np.append(sig, np.array([[r1, r2, r3]]), axis=0)

fig = plt.figure()
plt.plot(t, sig[:, 0], label=str(b[0]))
plt.plot(t, sig[:, 1], label=str(b[1])) 
plt.plot(t, sig[:, 2], label=str(b[2]))
plt.xlabel("Relative velocity")
plt.ylabel("Severity")
plt.legend()
plt.show()


# %%
