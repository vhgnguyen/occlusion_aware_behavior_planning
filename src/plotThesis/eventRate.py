# %%
import numpy as np
import matplotlib.pyplot as plt
import tikzplotlib

# from matplotlib import rc
# rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
# ## for Palatino and other serif fonts use:
# #rc('font',**{'family':'serif','serif':['Palatino']})
# rc('text', usetex=True)

# ------- Various sigmoidal function ---------
def exprate(t, tau):
    return 1 - np.exp(-tau*t)


def expRate(t, a, b, c):
    return a * np.exp(-b*(t - c))


def sigmoid(t, a, b):
    s = a / (1 + (t / (1-t))**(-b))
    return s


def gompertz(t, a, b, c, d):
    s = a*np.exp(-b*np.exp(-c*(t-d)))
    return s

def expRate1(t, a, b):
    return a * (1.0 - np.exp(-b*t)) \
            / (1.0 - np.exp(-b))


# %%
# ------- Plot expRate ---------
t = np.linspace(-5, 0, 100)
v_expRate = np.vectorize(expRate)
a = 1
c = -5
b = [1, 3, 7]

e1 = v_expRate(t, a, b[0], c)
e2 = v_expRate(t, a, b[1], c)
e3 = v_expRate(t, a, b[2], c)

fig = plt.figure(figsize=(15, 6))
plt.plot(t, e1, label="$\\beta_a=$"+str(b[0]), c='r')
plt.plot(t, e2, label="$\\beta_a=$"+str(b[1]), c='k')
plt.plot(t, e3, label="$\\beta_a=$"+str(b[2]), c='b')
plt.ylabel("Event rate $\\tau^{-1}$ [$s^{-1}$]")
plt.xlabel("Deacceleration $a$ [$m/s^2$]")
plt.legend()
tikzplotlib.save("eventAcc.tex",  axis_height='5cm', axis_width='12cm')

# %%
# ------- Plot sigmoidRate ---------
t = np.linspace(0.001, 0.999, 100)
v_sigmoid = np.vectorize(sigmoid)
a = 1
b = [1, 3, 7]

e1 = v_sigmoid(t, a, b[0])
e2 = v_sigmoid(t, a, b[1])
e3 = v_sigmoid(t, a, b[2])

fig = plt.figure(figsize=(15, 6))
plt.plot(t, e1, label="$\\beta_c=$"+str(b[0]), c='r')
plt.plot(t, e2, label="$\\beta_c=$"+str(b[1]), c='k')
plt.plot(t, e3, label="$\\beta_c=$"+str(b[2]), c='b')
plt.ylabel("Event rate $\\tau^{-1}$ [$s^{-1}$]")
plt.xlabel("Collision probability I [%]")
plt.legend()
tikzplotlib.save("eventColSig.tex",  axis_height='5cm', axis_width='12cm')


# %%
# ------- Plot expRate1 ---------
t = np.linspace(0, 1, 100)
v_expRate1 = np.vectorize(expRate1)
a = 1
b = [1, 3, 7]

e1 = v_expRate1(t, a, b[0])
e2 = v_expRate1(t, a, b[1])
e3 = v_expRate1(t, a, b[2])

fig = plt.figure(figsize=(15, 6))
plt.plot(t, e1, label="$\\beta_b=$"+str(b[0]), c='r')
plt.plot(t, e2, label="$\\beta_b=$"+str(b[1]), c='k')
plt.plot(t, e3, label="$\\beta_b=$"+str(b[2]), c='b')
plt.ylabel("Event rate $\\tau^{-1}$ [$s^{-1}$]")
plt.xlabel("Collision probability I [%]")
plt.legend(loc='lower right')
tikzplotlib.save("eventColExp.tex",  axis_height='5cm', axis_width='12cm')

# %%


