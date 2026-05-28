#!/usr/bin/env python3
"""
accel_tracking_error_node.py
=============================
Simulates acceleration tracking error for a quadrotor controller.

Takes the commanded acceleration (geometry_msgs/Twist) and outputs
a "achieved" acceleration that reflects realistic tracking imperfections:
lag, bandwidth limits, smooth disturbances, and steady-state error.

This is NOT an IMU noise model. The signal should be smooth and
physically plausible — a quadrotor's attitude + rotor loop cannot
execute step changes instantly, and disturbances are correlated in time.

Tracking Error Models
---------------------
1. first_order_lag  (DEFAULT)
   Achieved acceleration is low-pass filtered with time constant tau.
   Models rotor/attitude loop bandwidth. Clean and interpretable.

       a_achieved[k] = a_achieved[k-1] + (dt/tau) * (a_cmd[k] - a_achieved[k-1])

2. second_order
   Models the rotor+attitude loop as a second-order system with
   natural frequency wn and damping ratio zeta. Supports overshoot
   (underdamped, zeta < 1) and sluggish response (overdamped, zeta > 1).

3. first_order_lag_ou
   First-order lag (model 1) plus an Ornstein-Uhlenbeck disturbance.
   OU noise is smooth, mean-reverting, and coloured — suitable for
   wind gusts, rotor asymmetry, and CoM offset effects.

       d[k] = d[k-1] - (dt/tau_d) * d[k-1] + sigma_d * sqrt(2*dt/tau_d) * N(0,1)
       a_achieved[k] = lag_output[k] + d[k]

Parameters (ROS2 params or edit DEFAULTS below)
----------
  input_topic   : commanded acceleration topic  (default: /cmd_accel)
  output_topic  : achieved acceleration topic   (default: /achieved_accel)
  model         : tracking error model          (default: first_order_lag)
  dt            : expected sample period [s]    (default: 0.01)

  -- first_order_lag & first_order_lag_ou --
  tau           : lag time constant [s]         (default: 0.05)
                  Typical quadrotor attitude loop: 0.03 – 0.15 s

  -- second_order --
  wn            : natural frequency [rad/s]     (default: 20.0)
  zeta          : damping ratio [-]             (default: 0.7)
                  <1 = underdamped (overshoot), 1 = critically damped,
                  >1 = overdamped (sluggish)

  -- first_order_lag_ou (disturbance params) --
  ou_tau        : OU mean-reversion time [s]    (default: 1.0)
                  How long a gust / disturbance persists
  ou_sigma      : OU noise intensity [m/s²]     (default: 0.3)
                  Amplitude of smooth disturbance

Usage
-----
  python3 accel_tracking_error_node.py

  ros2 run <pkg> accel_tracking_error_node \
    --ros-args -p model:=first_order_lag_ou -p tau:=0.08 \
               -p ou_sigma:=0.4 -p ou_tau:=1.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import numpy as np


# ---------------------------------------------------------------------------
# Tracking error models — one instance per axis, all stateful
# ---------------------------------------------------------------------------

class FirstOrderLag:
    """
    Discrete first-order low-pass filter modelling attitude/rotor lag.

    The achieved output converges toward the command with time constant tau.
    Equivalent to a continuous system  H(s) = 1 / (tau*s + 1).
    """

    def __init__(self, tau: float, dt: float):
        """
        Parameters
        ----------
        tau : lag time constant [s]. Larger = slower / worse tracking.
        dt  : sample period [s].
        """
        if tau <= 0:
            raise ValueError("tau must be positive")
        self.alpha = dt / tau          # blend factor; clipped to (0, 1)
        self.alpha = np.clip(self.alpha, 1e-6, 1.0)
        self.state = 0.0               # achieved acceleration state

    def step(self, cmd: float) -> float:
        self.state += self.alpha * (cmd - self.state)
        return self.state

    def __str__(self):
        return f"FirstOrderLag(alpha={self.alpha:.4f})"


class SecondOrderSystem:
    """
    Discrete second-order system modelling the rotor + attitude loop.

    Continuous transfer function:
        H(s) = wn² / (s² + 2*zeta*wn*s + wn²)

    Discretised with Euler integration of the state-space form:
        x1_dot = x2
        x2_dot = wn²*(cmd - x1) - 2*zeta*wn*x2

    Parameters
    ----------
    wn   : natural frequency [rad/s]. Higher = faster response.
    zeta : damping ratio. 0.7 – 1.0 is typical for a well-tuned loop.
    dt   : sample period [s].
    """

    def __init__(self, wn: float, zeta: float, dt: float):
        self.wn = wn
        self.zeta = zeta
        self.dt = dt
        self.x1 = 0.0    # position state (achieved accel)
        self.x2 = 0.0    # velocity state (rate of change)

    def step(self, cmd: float) -> float:
        # State-space Euler integration
        x1_dot = self.x2
        x2_dot = (self.wn ** 2) * (cmd - self.x1) - 2 * self.zeta * self.wn * self.x2

        self.x1 += self.dt * x1_dot
        self.x2 += self.dt * x2_dot
        return self.x1

    def __str__(self):
        return f"SecondOrder(wn={self.wn}, zeta={self.zeta})"


class OrnsteinUhlenbeckDisturbance:
    """
    Ornstein-Uhlenbeck (OU) process for smooth stochastic disturbances.

    The OU process is the continuous-time analogue of AR(1):
        d[k] = d[k-1] * exp(-dt/tau) + sigma * sqrt(1 - exp(-2*dt/tau)) * N(0,1)

    This gives coloured (low-pass) noise that is:
      - Mean-reverting to zero
      - Smooth and temporally correlated
      - Suitable for wind gusts, rotor asymmetry, CoM offsets

    Parameters
    ----------
    tau   : correlation / reversion time [s]. Larger = slower drift.
    sigma : steady-state standard deviation [m/s²]. Sets amplitude.
    dt    : sample period [s].
    """

    def __init__(self, tau: float, sigma: float, dt: float):
        self.decay = np.exp(-dt / tau)
        self.noise_scale = sigma * np.sqrt(1.0 - self.decay ** 2)
        self.state = 0.0

    def sample(self) -> float:
        self.state = self.decay * self.state + self.noise_scale * np.random.randn()
        return self.state


class FirstOrderLagWithOU:
    """
    First-order lag tracking error with an additive OU disturbance.

    Represents a sluggish attitude loop that is also subject to
    smooth external forces (wind, asymmetric rotor wear, CoM shift).
    """

    def __init__(self, tau: float, ou_tau: float, ou_sigma: float, dt: float):
        self.lag = FirstOrderLag(tau, dt)
        self.ou = OrnsteinUhlenbeckDisturbance(ou_tau, ou_sigma, dt)

    def step(self, cmd: float) -> float:
        return self.lag.step(cmd) + self.ou.sample()

    def __str__(self):
        return (f"FirstOrderLagOU(alpha={self.lag.alpha:.4f}, "
                f"ou_decay={self.ou.decay:.4f}, "
                f"ou_scale={self.ou.noise_scale:.4f})")


# ---------------------------------------------------------------------------
# Per-axis set — each DOF has independent state
# ---------------------------------------------------------------------------

class AxisModelSet:
    """Six independent model instances, one per Twist DOF."""

    AXES = ["lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"]

    def __init__(self, factory_fn):
        self._models = {ax: factory_fn() for ax in self.AXES}

    def step(self, axis: str, cmd: float) -> float:
        return self._models[axis].step(cmd)


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class AccelTrackingErrorNode(Node):

    DEFAULTS = {
        "input_topic":  "/cmd_accel",
        "output_topic": "/achieved_accel",
        "model":        "first_order_lag",
        "dt":           0.02,    # [s]  match your controller rate
        # first_order_lag / first_order_lag_ou
        "tau":          0.03,    # [s]  attitude loop lag
        # second_order
        "wn":           20.0,    # [rad/s]
        "zeta":         0.7,     # [-]
        # OU disturbance (first_order_lag_ou only)
        "ou_tau":       1.0,     # [s]   gust persistence
        "ou_sigma":     0.3,     # [m/s²] gust amplitude
    }

    def __init__(self):
        super().__init__("accel_tracking_error_node")

        for name, default in self.DEFAULTS.items():
            self.declare_parameter(name, default)

        p = {name: self.get_parameter(name).value for name in self.DEFAULTS}

        self._models = self._build_models(p)
        self.get_logger().info(
            f"Tracking error model: {self._models._models['lin_x']}"
        )

        self._pub = self.create_publisher(Twist, p["output_topic"], 10)
        self._sub = self.create_subscription(
            Twist, p["input_topic"], self._callback, 10
        )

        self.get_logger().info(
            f"Subscribing : {p['input_topic']}\n"
            f"Publishing  : {p['output_topic']}"
        )

    def _build_models(self, p: dict) -> AxisModelSet:
        model = p["model"].lower()

        if model == "first_order_lag":
            def factory():
                return FirstOrderLag(p["tau"], p["dt"])

        elif model == "second_order":
            def factory():
                return SecondOrderSystem(p["wn"], p["zeta"], p["dt"])

        elif model == "first_order_lag_ou":
            def factory():
                return FirstOrderLagWithOU(
                    p["tau"], p["ou_tau"], p["ou_sigma"], p["dt"]
                )

        else:
            self.get_logger().warn(
                f"Unknown model '{model}', falling back to 'first_order_lag'."
            )
            def factory():
                return FirstOrderLag(p["tau"], p["dt"])

        return AxisModelSet(factory)

    def _callback(self, msg: Twist) -> None:
        out = Twist()

        out.linear.x  = self._models.step("lin_x", msg.linear.x)
        out.linear.y  = self._models.step("lin_y", msg.linear.y)
        out.linear.z  = self._models.step("lin_z", msg.linear.z)
        # out.angular.x = self._models.step("ang_x", msg.angular.x)
        # out.angular.y = self._models.step("ang_y", msg.angular.y)
        # out.angular.z = self._models.step("ang_z", msg.angular.z)
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        self._pub.publish(out)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = AccelTrackingErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()