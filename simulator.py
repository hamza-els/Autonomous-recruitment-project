#%%
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
# %%

class Simulator:
    def __init__(self, controller_callback):
        """initializes a Simulator

        Args:
            controller_callback (function): function that takes in a numpy array of [x, y, theta, v, phi] and outputs a control [a, phidot]
        """
        self.cb = controller_callback
        self.dynamics = ca.external('F', ca.Importer('assets/system_dynamics.c', 'shell'))
        self.left_cones = np.load('assets/left.npy')
        self.right_cones = np.load('assets/right.npy')
        self.cones = np.concatenate([self.left_cones, self.right_cones], axis=0)
        self.centerline = np.load('assets/centerline.npy')
        self.car_outline = np.load('assets/pts_mat.npy')
        self.A = np.load('assets/a_mat.npy')
        self.b = np.load('assets/b_mat.npy')
        self.steering_limits = (-0.5, 0.5)
        self.accel_limits = (-10, 4)
        self.vel_limits = (-1, 20)
        self.steering_vel_limits = (-1, 1)
    def R(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
    def check_collision(self, state):
        return np.any(np.all(self.A@self.R(-state[2])@((self.cones - state[0:2]).T) < self.b[:, np.newaxis], axis=0), axis=0)
    def check_accel(self, state, control):
        l = 0.79 # wheelbase=1.58, this is half
        return (
            control[0]**2
         + ((state[3]**2 / l) * np.sin(np.arctan(0.5) * np.tan(state[2])))**2
        ) > 12**2 
    def run(self, tf=90):
        self.log = []
        state = np.zeros(5)
        for t in np.arange(0, tf, 0.01):
            u = self.cb(state)
            assert isinstance(u, np.ndarray), f"expected numpy array from controller but got type {type(u)}"
            assert u.shape==(2,), f"expected shape (2,) from controller but received {u.shape}"
            u = np.array([
                np.clip(u[0], *self.accel_limits),
                np.clip(u[1], *self.steering_vel_limits)
            ])

            if ((state[4] > self.steering_limits[1] and u[1] > 0)
             or (state[4] < self.steering_limits[0] and u[1] < 0)):
                u[1] = 0
            if ((state[2] > self.vel_limits[1] and u[0] > 0)
             or (state[2] < self.vel_limits[0] and u[0] < 0)):
                u[0] = 0
            safe = self.check_collision(state)
            feasible = self.check_accel(state, u)
            self.log.append((t, state, u, safe, feasible))
            state = self.dynamics(state, u).toarray().flatten()
    def get_results(self):
        try: log = self.log
        except: raise ValueError("cannot animate; no results exist. Did you .run() the simulator?")
        ts = np.array([i[0] for i in self.log])
        xs = np.concatenate([i[1][:, np.newaxis] for i in self.log], axis=1)
        us = np.concatenate([i[2][:, np.newaxis] for i in self.log], axis=1)
        crash  = np.array([i[3] for i in self.log])
        slip  = np.array([i[4] for i in self.log])
        return (ts, xs, us, crash, slip)
    def plot(self):
        ts, xs, us, crash, slip = self.get_results()
        fig, axs = plt.subplots(7, sharex=True)

        axs[0].plot(ts, xs[0]); axs[0].set_ylabel('x pos (m)')
        axs[1].plot(ts, xs[1]); axs[1].set_ylabel('y pos (m)')
        axs[2].plot(ts, xs[2]); axs[2].set_ylabel('heading (rad)')
        axs[3].plot(ts, xs[3]); axs[3].set_ylabel('velocity (m/s)')
        axs[4].plot(ts, xs[4]); axs[4].set_ylabel('steering angle (rad)')

        axs[5].plot(ts, us[0]); axs[5].set_ylabel('fwd accel (m/s^2)')
        axs[6].plot(ts, us[1]); axs[6].set_ylabel('steering velocity (rad/s)')
        for i in range(5):
            axs[i].scatter(ts[crash], xs[i, crash], color='tab:red')
            axs[i].scatter(ts[slip], xs[i, slip], color='tab:orange')
        for i in range(2):
            axs[5+i].scatter(ts[crash], us[i, crash], color='tab:red')
            axs[5+i].scatter(ts[slip], us[i, slip], color='tab:orange')
        
        plt.show()
    def animate(self):
        plt.scatter(*self.left_cones.T, color='tab:blue')
        plt.scatter(*self.right_cones.T, color='tab:orange')
        ts, xs, us, crash, slip = self.get_results()
        plt.scatter(*xs[0:2, np.logical_and(np.logical_not(crash), np.logical_not(slip))], color='tab:green')
        plt.scatter(*xs[0:2, crash], color='tab:red')
        plt.scatter(*xs[0:2, slip], color='yellow')
        plt.show()

