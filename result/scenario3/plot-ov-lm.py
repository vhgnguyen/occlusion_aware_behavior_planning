#%%
import numpy as np
import matplotlib.pyplot as plt

dt_name = ["s3-vx8-dt-ov-lm.txt"] 
dd_name = ["s3-vx8-dd-ov-lm.txt"]

vx = [8]

fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(14, 6))
ax[0].set_xlabel("Time [s]")
ax[0].set_ylabel("Velocity [m/s]")
ax[0].set_ylim(0, 15)
ax[1].set_xlabel("Time [s]")
ax[1].set_ylabel("Acceleration [m/s2]")
ax[1].set_ylim(-6, 3)

for (n, v) in zip(dt_name, vx):
    f = np.genfromtxt(n)
    ax[0].plot(f[:, 0], f[:, 1], label="v="+str(v)+"m/s")
    ax[1].plot(f[:, 0], f[:, 2], label="v="+str(v)+"m/s")
ax[0].axhline(y=8, linestyle='-.', color='k')
ax[0].legend()
ax[1].legend()
plt.show()

#%%
fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(14, 6))
ax[0].set_xlabel("Travel distance [m]")
ax[0].set_ylabel("Velocity [m/s]")
ax[0].set_ylim(0, 15)
ax[1].set_xlabel("Travel distance [m]")
ax[1].set_ylabel("Acceleration [m/s2]")
ax[1].set_ylim(-6, 3)

for (n, v) in zip(dd_name, vx):
    f = np.genfromtxt(n)
    ax[0].plot(f[:, 0], f[:, 1], label="v="+str(v)+"m/s")
    ax[1].plot(f[:, 0], f[:, 2], label="v="+str(v)+"m/s")
ax[0].axhline(y=8, linestyle='-.', color='k')
ax[0].legend()
ax[1].legend()
plt.show()



# %%
