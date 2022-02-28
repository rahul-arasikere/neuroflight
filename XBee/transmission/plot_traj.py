import pickle
import matplotlib.pyplot as plt
import numpy as np

def unroll_rpy(rpy):
    return np.array([rpy.roll, rpy.pitch, rpy.yaw])

def unroll_act(act):
    return np.array([
        act.top_left,
        act.top_right,
        act.bottom_left,
        act.bottom_right,
    ])

def unroll_obs(obs):
    print(unroll_rpy(obs.error))
    return np.concatenate([
        unroll_rpy(obs.error),
        unroll_rpy(obs.ang_vel),
        unroll_rpy(obs.ang_acc),
        unroll_act(obs.prev_action)
    ])

traj = pickle.load( open( "traj.p", "rb" ) )

fig = plt.figure(constrained_layout=True)
ax = fig.subplot_mosaic([["er", "a1"], ["ep", "a2"], ["ey", "a3"], ["ey", "a4"]])
unrolled = np.array(list(map(unroll_obs, traj))).T
print(unrolled.shape)
for obs in traj:
    print(obs)
ax["er"].plot(unrolled[0])
ax["ep"].plot(unrolled[1])
ax["ey"].plot(unrolled[2])

ax["a1"].plot(unrolled[-4])
ax["a2"].plot(unrolled[-3])
ax["a3"].plot(unrolled[-2])
ax["a4"].plot(unrolled[-1])

plt.show()