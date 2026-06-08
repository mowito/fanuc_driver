#!/usr/bin/env python3
"""
Tapered Peg Insertion — DIY Compliance via Servo + Wrench
==========================================================
No admittance controller. Full control over:
  - wrench filtering (EMA, tunable alpha)
  - compliance gain (torque → angular correction)
  - push speed
  - force thresholds

Chain:
    this node
        |  computes full twist:
        |    linear.z  = -PUSH_SPEED           (constant Z push)
        |    angular.x = -GAIN * filtered_Ty   (Rx self-alignment)
        |    angular.y =  GAIN * filtered_Tx   (Ry self-alignment)
        v
    /servo_node/delta_twist_cmds
        v
    servo_node  (Cartesian → joints, Jacobian handled internally)
        v
    forward_position_controller
        v
    hardware

Tuning workflow:
    Step 1 — set COMPLIANCE_GAIN = 0.0, verify Z push and F/T readings
    Step 2 — increase COMPLIANCE_GAIN slowly (0.001 → 0.005 → 0.01)
             and observe angular correction under lateral torque
    Step 3 — tune FILTER_ALPHA if corrections are noisy or sluggish

Filter (EMA):
    filtered = FILTER_ALPHA * raw + (1 - FILTER_ALPHA) * filtered
    FILTER_ALPHA = 0.1  → heavy smoothing, slow response
    FILTER_ALPHA = 0.5  → moderate
    FILTER_ALPHA = 1.0  → no filtering (raw signal)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, WrenchStamped
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController, ListControllers

import math
import time
import numpy as np


# =============================================================================
# TUNING PARAMETERS
# =============================================================================

# --- Push ---
PUSH_SPEED        = 0.1   # m/s along -fanuc_flange Z (3 mm/s)

# --- Force thresholds ---
FORCE_THRESHOLD_N = 50.0    # Fz (N) → insertion complete
JAM_FORCE_N       = 100.0    # lateral force (N) → abort

# --- Depth safety ---
INSERTION_DEPTH   = 0.05    # m (50 mm hard limit)

# --- Retract ---
RETRACT_SPEED     = 0.01    # m/s
RETRACT_DEPTH     = 0.03    # m

# --- Compliance (DIY admittance on Rx/Ry) ---
# Torque (Nm) in fanuc_flange frame → angular velocity (rad/s) correction
# Start at 0.0, increase slowly until you see self-alignment behaviour.
# Too high → oscillation. Too low → no correction.
COMPLIANCE_GAIN   = 0.1     # rad/s per Nm  — START HERE, tune up gradually

# --- Wrench filter ---
# EMA alpha: lower = smoother but slower, higher = faster but noisier
FILTER_ALPHA      = 0.2     # 0.1–0.3 recommended for insertion

# --- Ramp ---
# Linearly scale twist from 0 → full over this duration to avoid startup jerk.
# Set to 0.0 to disable.
RAMP_TIME         = 0.5     # seconds

# --- Loop rate ---
LOOP_HZ           = 100     # must match servo_node update rate

# Twist command frame
COMMAND_FRAME     = "fanuc_flange"


# =============================================================================
# EMA Filter
# =============================================================================

class EMAFilter:
    """Exponential Moving Average filter for a scalar."""

    def __init__(self, alpha: float):
        self.alpha = alpha
        self._value = None

    def update(self, raw: float) -> float:
        if self._value is None:
            self._value = raw
        else:
            self._value = self.alpha * raw + (1.0 - self.alpha) * self._value
        return self._value

    def reset(self):
        self._value = None

    @property
    def value(self) -> float:
        return self._value if self._value is not None else 0.0


# =============================================================================
# Node
# =============================================================================

class PegInsertionNode(Node):

    def __init__(self):
        super().__init__("peg_insertion")

        self._wrench: WrenchStamped = None
        self._joint_states: JointState = None

        self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            lambda msg: setattr(self, "_wrench", msg),
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            lambda msg: setattr(self, "_joint_states", msg),
            10,
        )

        self._twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )
        self._switch_ctrl = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._list_ctrl = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )

        # One filter per wrench component we care about
        self._f = {
            "fx": EMAFilter(FILTER_ALPHA),
            "fy": EMAFilter(FILTER_ALPHA),
            "fz": EMAFilter(FILTER_ALPHA),
            "tx": EMAFilter(FILTER_ALPHA),
            "ty": EMAFilter(FILTER_ALPHA),
        }
        
        self._x_dot = 0.0
        self._y_dot = 0.0

    # -------------------------------------------------------------------------
    # Wrench helpers
    # -------------------------------------------------------------------------

    def _update_filters(self):
        if self._wrench is None:
            return
        w = self._wrench.wrench
        self._f["fx"].update(w.force.x)
        self._f["fy"].update(w.force.y)
        self._f["fz"].update(w.force.z)
        self._f["tx"].update(w.torque.x)
        self._f["ty"].update(w.torque.y)

    def _fz(self) -> float:
        """
        Filtered Fz along insertion axis.
        Uses abs() to handle sensor mounting sign.
        If pushing reads negative, change to: return -self._f["fz"].value
        """
        return abs(self._f["fz"].value)

    def _f_lateral(self) -> float:
        return math.sqrt(self._f["fx"].value**2 + self._f["fy"].value**2)

    def _log_ft(self, filtered: bool = True):
        if self._wrench is None:
            self.get_logger().warn("  No F/T data")
            return
        if filtered:
            self.get_logger().info(
                f"  F/T (filtered)"
                f"  fx={self._f['fx'].value:+6.2f}"
                f"  fy={self._f['fy'].value:+6.2f}"
                f"  fz={self._f['fz'].value:+6.2f} N"
                f"   tx={self._f['tx'].value:+5.2f}"
                f"  ty={self._f['ty'].value:+5.2f} Nm"
            )
        else:
            w = self._wrench.wrench
            self.get_logger().info(
                f"  F/T (raw)"
                f"  fx={w.force.x:+6.2f}"
                f"  fy={w.force.y:+6.2f}"
                f"  fz={w.force.z:+6.2f} N"
                f"   tx={w.torque.x:+5.2f}"
                f"  ty={w.torque.y:+5.2f} Nm"
            )

    # -------------------------------------------------------------------------
    # Compliance twist computation
    # -------------------------------------------------------------------------

    @staticmethod
    def _deadband(value: float, threshold: float) -> float:
        """Return 0.0 if abs(value) < threshold, else return value unchanged."""
        return 0.0 if abs(value) < threshold else value

    def _compliance_twist(self, vz: float, ramp: float = 1.0) -> TwistStamped:
        """
        Build the full twist command:
          linear.z  = vz * ramp                       (push, scaled by ramp)
          angular.x = -GAIN * filtered_Ty * ramp      (Ry torque → Rx correction)
          angular.y =  GAIN * filtered_Tx * ramp      (Rx torque → Ry correction)

        ramp: 0.0 → 1.0 scalar applied to the whole twist to avoid startup jerk.

        Sign convention (fanuc_flange frame, peg pointing in +Z):
          Positive Tx (torque about X) → peg tip pushed in +Y → correct by rotating +Ry
          Positive Ty (torque about Y) → peg tip pushed in -X → correct by rotating -Rx

        If self-alignment goes the wrong direction, flip the sign on the
        relevant angular component below.
        """
        # Clamp ramp to [0, 1]
        ramp = max(0.0, min(1.0, ramp))

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = COMMAND_FRAME

        msg.twist.linear.z  = vz * ramp

        if COMPLIANCE_GAIN > 0.0:
            tx = self._deadband(self._f["tx"].value, 0.1)
            ty = self._deadband(self._f["ty"].value, 0.1)
            
            K = 1
            M = 0.3
            B = 5

            # admittance controller:
            # x_dot_dot = (tx - B * x_dot) / M
            acc_x = (tx - B * self._x_dot) / M
            acc_y = (ty - B * self._y_dot) / M

            self._x_dot += acc_x * (1 / LOOP_HZ)
            self._y_dot += acc_y * (1 / LOOP_HZ)

            self._x_dot = np.clip(self._x_dot, -0.2, 0.2)
            self._y_dot = np.clip(self._y_dot, -0.2, 0.2)

            msg.twist.angular.x =  self._x_dot #(COMPLIANCE_GAIN * tx * ramp)
            msg.twist.angular.y =  self._y_dot #(COMPLIANCE_GAIN * ty * ramp)
        return msg

    def _stop_servo(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = COMMAND_FRAME
        for _ in range(5):
            self._twist_pub.publish(msg)
            time.sleep(0.01)

    # -------------------------------------------------------------------------
    # Controller management
    # -------------------------------------------------------------------------

    def _get_controller_states(self) -> dict:
        if not self._list_ctrl.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("list_controllers not available")
        fut = self._list_ctrl.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return {c.name: c.state for c in fut.result().controller}

    def _ensure_fpc_active(self):
        """
        Activate forward_position_controller.
        Deactivate admittance chain if running.
        Servo must output to forward_position_controller:
            command_out_topic: /forward_position_controller/commands
            command_out_type:  std_msgs/Float64MultiArray
        """
        self.get_logger().info("Checking controllers...")
        states = self._get_controller_states()

        fpc = states.get("forward_position_controller", "missing")
        adm = states.get("admittance_controller",       "missing")
        jtc = states.get("joint_trajectory_controller", "missing")

        self.get_logger().info(
            f"  forward_position_controller: {fpc}\n"
            f"  admittance_controller:       {adm}\n"
            f"  joint_trajectory_controller: {jtc}"
        )

        if fpc == "active":
            self.get_logger().info("  forward_position_controller already active.")
            return

        to_deactivate = [c for c, s in
                         [("admittance_controller", adm),
                          ("joint_trajectory_controller", jtc)]
                         if s == "active"]

        if not self._switch_ctrl.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("switch_controller not available")

        req = SwitchController.Request()
        req.activate_controllers   = ["forward_position_controller"]
        req.deactivate_controllers = to_deactivate
        req.strictness             = SwitchController.Request.STRICT
        req.activate_asap          = True

        fut = self._switch_ctrl.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.result().ok:
            raise RuntimeError("Failed to activate forward_position_controller")
        self.get_logger().info("  forward_position_controller activated.")

    # -------------------------------------------------------------------------
    # Insertion
    # -------------------------------------------------------------------------

    def insert(self, sign=1) -> str:
        self.get_logger().info("\n" + "=" * 56)
        self.get_logger().info(
            "INSERTION — DIY compliance via filtered wrench\n"
            f"  COMPLIANCE_GAIN : {COMPLIANCE_GAIN:.4f} rad/s per Nm\n"
            f"  FILTER_ALPHA    : {FILTER_ALPHA}\n"
            f"  RAMP_TIME       : {RAMP_TIME:.2f} s\n"
            f"  Push speed      : {PUSH_SPEED*1000:.1f} mm/s\n"
            f"  Stop at Fz      : {FORCE_THRESHOLD_N:.1f} N\n"
            f"  Abort lateral   : {JAM_FORCE_N:.1f} N\n"
            f"  Depth limit     : {INSERTION_DEPTH*1000:.0f} mm"
        )
        self.get_logger().info("=" * 56)

        # Reset filters so old data doesn't contaminate
        for f in self._f.values():
            f.reset()

        # Warm up filters with a few readings before moving
        self.get_logger().info("Warming up filters (0.5s)...")
        warmup = time.time() + 0.5
        while time.time() < warmup:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._update_filters()

        self.get_logger().info("Baseline F/T (filtered):")
        self._log_ft(filtered=True)
        self.get_logger().info("Baseline F/T (raw):")
        self._log_ft(filtered=False)

        loop_period = 1.0 / LOOP_HZ
        step_m      = PUSH_SPEED / LOOP_HZ
        depth       = 0.0
        outcome     = "depth_limit"
        log_every   = max(1, LOOP_HZ // 2)
        tick        = 0
        start_time  = time.time()

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                self._update_filters()

                elapsed = time.time() - start_time
                ramp    = min(1.0, elapsed / RAMP_TIME) if RAMP_TIME > 0.0 else 1.0

                fz = self._fz()
                fl = self._f_lateral()

                if tick % log_every == 0:
                    tx = self._deadband(self._f["tx"].value, 0.1)
                    ty = self._deadband(self._f["ty"].value, 0.1)
                    wx =  COMPLIANCE_GAIN * tx * ramp
                    wy =  COMPLIANCE_GAIN * ty * ramp
                    self.get_logger().info(
                        f"  d={depth*1000:5.1f}mm"
                        f"  ramp={ramp:.2f}"
                        f"  Fz={fz:6.2f}N  Flat={fl:5.2f}N"
                        f"  Tx={tx:+5.2f}Nm  Ty={ty:+5.2f}Nm"
                        f"  →  wx={wx:+6.4f}  wy={wy:+6.4f} rad/s"
                    )

                if fz >= FORCE_THRESHOLD_N and sign == 1:
                    self.get_logger().info(
                        f"  ✓ Inserted — Fz={fz:.2f} N >= {FORCE_THRESHOLD_N:.1f} N"
                    )
                    outcome = "success"
                    break

                if fl >= JAM_FORCE_N:
                    self.get_logger().warn(
                        f"  ✗ Jam — lateral={fl:.2f} N >= {JAM_FORCE_N:.1f} N"
                    )
                    outcome = "jam"
                    break

                self._twist_pub.publish(self._compliance_twist(PUSH_SPEED * sign, ramp))
                depth += step_m
                tick  += 1
                time.sleep(loop_period)

        except KeyboardInterrupt:
            outcome = "interrupted"

        finally:
            self._stop_servo()

        return outcome

    # -------------------------------------------------------------------------
    # Retract
    # -------------------------------------------------------------------------

    def retract(self):
        self.get_logger().info("\nRETRACT (no compliance)")
        loop_period = 1.0 / LOOP_HZ
        step_m      = RETRACT_SPEED / LOOP_HZ
        retracted   = 0.0

        try:
            while rclpy.ok() and retracted < RETRACT_DEPTH:
                rclpy.spin_once(self, timeout_sec=0.0)
                # Retract: pure +Z, no angular correction
                msg = TwistStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = COMMAND_FRAME
                msg.twist.linear.z  = RETRACT_SPEED
                self._twist_pub.publish(msg)
                retracted += step_m
                time.sleep(loop_period)
        finally:
            self._stop_servo()

        self.get_logger().info(f"  Retracted {retracted*1000:.1f} mm")

    # -------------------------------------------------------------------------
    # Entry point
    # -------------------------------------------------------------------------

    def run(self):
        self.get_logger().info("Waiting for F/T and joint states...")
        deadline = time.time() + 10.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._wrench is not None and self._joint_states is not None:
                break

        if self._joint_states is None:
            raise RuntimeError("No joint states — is hardware running?")
        if self._wrench is None:
            self.get_logger().warn(
                "No F/T data — force detection won't work. "
                "Check force_torque_sensor_broadcaster is active."
            )

        self.get_logger().info(
            "\n"
            "╔══════════════════════════════════════════════╗\n"
            "║  Tapered Peg Insertion — DIY Compliance      ║\n"
            "╠══════════════════════════════════════════════╣\n"
            "║  Servo  → -Z push                            ║\n"
            "║  Wrench → filtered torque → angular twist    ║\n"
            "║  No admittance controller                    ║\n"
            "╠══════════════════════════════════════════════╣\n"
           f"║  COMPLIANCE_GAIN : {COMPLIANCE_GAIN:.4f} rad/s per Nm\n"
           f"║  FILTER_ALPHA    : {FILTER_ALPHA}\n"
           f"║  RAMP_TIME       : {RAMP_TIME:.2f} s\n"
           f"║  Push speed      : {PUSH_SPEED*1000:.1f} mm/s\n"
           f"║  Force threshold : {FORCE_THRESHOLD_N:.1f} N\n"
           f"║  Jam threshold   : {JAM_FORCE_N:.1f} N\n"
           f"║  Depth limit     : {INSERTION_DEPTH*1000:.0f} mm\n"
           f"║  Retract dist    : {RETRACT_DEPTH*1000:.0f} mm\n"
           f"║  Command frame   : {COMMAND_FRAME}\n"
            "╚══════════════════════════════════════════════╝\n"
        )

        input("Jog peg to hole entry, then press ENTER...")

        self._ensure_fpc_active()
        time.sleep(0.5)

        try:
            outcome = self.insert(sign=1)
        except RuntimeError as e:
            self._stop_servo()
            self.get_logger().error(f"Insertion error: {e}")
            outcome = "error"

        self.get_logger().info(f"\nOutcome: {outcome.upper()}")
        self._log_ft()

        # self.retract()
        self.insert(sign=-1)
        self.get_logger().info("Done.")


def main():
    rclpy.init()
    node = PegInsertionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    except RuntimeError as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()