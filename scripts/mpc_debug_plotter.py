#!/usr/bin/env python3

import collections
import math
import threading

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
from std_msgs.msg import Float64MultiArray


class MpcDebugPlotter:
    def __init__(self):
        self.topic = rospy.get_param("~debug_topic", "/mpc_debug")
        self.max_points = int(rospy.get_param("~max_points", 600))
        self.lock = threading.Lock()

        self.times = collections.deque(maxlen=self.max_points)
        self.along_err = collections.deque(maxlen=self.max_points)
        self.lateral_err = collections.deque(maxlen=self.max_points)
        self.eyaw = collections.deque(maxlen=self.max_points)
        self.vx = collections.deque(maxlen=self.max_points)
        self.vy = collections.deque(maxlen=self.max_points)
        self.w = collections.deque(maxlen=self.max_points)
        self.ok = collections.deque(maxlen=self.max_points)

        self.sub = rospy.Subscriber(self.topic, Float64MultiArray,
                                    self.on_debug, queue_size=10)

        self.fig, (self.ax_err, self.ax_vel, self.ax_ok) = plt.subplots(
            3, 1, sharex=True, figsize=(10, 8)
        )
        self.fig.canvas.manager.set_window_title("MPC Debug Monitor")

        self.line_along, = self.ax_err.plot([], [], label="along_err")
        self.line_lateral, = self.ax_err.plot([], [], label="lateral_err")
        self.line_eyaw, = self.ax_err.plot([], [], label="yaw_err")
        self.ax_err.set_ylabel("Error")
        self.ax_err.grid(True, linestyle="--", alpha=0.4)
        self.ax_err.legend(loc="upper right")

        self.line_vx, = self.ax_vel.plot([], [], label="vx")
        self.line_vy, = self.ax_vel.plot([], [], label="vy")
        self.line_w, = self.ax_vel.plot([], [], label="w")
        self.ax_vel.set_ylabel("Cmd")
        self.ax_vel.grid(True, linestyle="--", alpha=0.4)
        self.ax_vel.legend(loc="upper right")

        self.line_ok, = self.ax_ok.plot([], [], label="mpc_ok")
        self.ax_ok.set_ylabel("OK")
        self.ax_ok.set_xlabel("Time (s)")
        self.ax_ok.set_ylim(-0.1, 1.1)
        self.ax_ok.grid(True, linestyle="--", alpha=0.4)

    def on_debug(self, msg):
        # Order:
        # 0 ok, 1 t, 2 nearest, 3 x, 4 y, 5 yaw,
        # 6 ref_x, 7 ref_y, 8 ref_yaw,
        # 9 ex, 10 ey, 11 eyaw,
        # 12 along_err, 13 lateral_err,
        # 14 vx, 15 vy, 16 w
        if len(msg.data) < 17:
            return
        with self.lock:
            self.ok.append(msg.data[0])
            self.times.append(msg.data[1])
            self.along_err.append(msg.data[12])
            self.lateral_err.append(msg.data[13])
            self.eyaw.append(msg.data[11])
            self.vx.append(msg.data[14])
            self.vy.append(msg.data[15])
            self.w.append(msg.data[16])

    def update(self, _):
        with self.lock:
            t = list(self.times)
            along = list(self.along_err)
            lateral = list(self.lateral_err)
            eyaw = list(self.eyaw)
            vx = list(self.vx)
            vy = list(self.vy)
            w = list(self.w)
            ok = list(self.ok)

        if not t:
            return self.line_along, self.line_lateral, self.line_eyaw, \
                self.line_vx, self.line_vy, self.line_w, self.line_ok

        t0 = t[0]
        t_rel = [ti - t0 for ti in t]

        self.line_along.set_data(t_rel, along)
        self.line_lateral.set_data(t_rel, lateral)
        self.line_eyaw.set_data(t_rel, eyaw)
        self.line_vx.set_data(t_rel, vx)
        self.line_vy.set_data(t_rel, vy)
        self.line_w.set_data(t_rel, w)
        self.line_ok.set_data(t_rel, ok)

        for ax in (self.ax_err, self.ax_vel, self.ax_ok):
            ax.relim()
            ax.autoscale_view()

        self.ax_ok.set_ylim(-0.1, 1.1)

        return self.line_along, self.line_lateral, self.line_eyaw, \
            self.line_vx, self.line_vy, self.line_w, self.line_ok

    def run(self):
        # Keep a reference to prevent garbage collection stopping updates.
        self.anim = animation.FuncAnimation(
            self.fig, self.update, interval=100, blit=False
        )
        plt.show()


def main():
    rospy.init_node("mpc_debug_plotter", anonymous=True)
    plotter = MpcDebugPlotter()
    plotter.run()


if __name__ == "__main__":
    main()
