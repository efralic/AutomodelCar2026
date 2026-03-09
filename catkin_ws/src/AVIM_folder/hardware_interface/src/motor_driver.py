#!/usr/bin/env python3
"""
Motor Driver for Raspberry Pi 4
Controls motors via Encoder Motor Module (I2C address 0x34)
Controls servo for steering via pigpio hardware PWM
GPIO 2 = SDA, GPIO 3 = SCL
"""

import rospy
from std_msgs.msg import Int16, Bool, String
import smbus2
import pigpio
import time

# ── I2C Encoder Motor Module registers ──────────────────────────────────────
I2C_ADDR                    = 0x34
MOTOR_TYPE_ADDR             = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_SPEED_ADDR      = 0x33
MOTOR_FIXED_PWM_ADDR        = 0x1F
MOTOR_ENCODER_TOTAL_ADDR    = 0x3C

# Motor type
MOTOR_TYPE_JGB37_520_12V_110RPM = 3

class MotorDriver:
    def __init__(self):
        rospy.init_node('motor_driver', anonymous=False)

        # ── I2C bus (GPIO 2=SDA, GPIO 3=SCL → bus 1 on RPi) ─────────────
        try:
            self.bus = smbus2.SMBus(1)
            rospy.loginfo("I2C bus opened on /dev/i2c-1")
        except Exception as e:
            rospy.logerr(f"Failed to open I2C bus: {e}")
            exit(1)

        # ── pigpio for servo PWM ─────────────────────────────────────────
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("Failed to connect to pigpio daemon! Run: sudo pigpiod")
            exit(1)

        # ── Servo pin ────────────────────────────────────────────────────
        self.SERVO_PIN    = 13
        self.SERVO_MIN_PULSE  = 1000   # 45 degrees
        self.SERVO_MAX_PULSE  = 2000   # 135 degrees
        self.SERVO_CENTER     = 1500   # 90 degrees

        # ── Vehicle state ────────────────────────────────────────────────
        self.current_speed    = 0
        self.current_angle    = 90
        self.autonomous_mode  = True
        self.emergency_stop   = False

        # ── Initialize hardware ──────────────────────────────────────────
        self.setup_motor_module()
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, self.SERVO_CENTER)
        rospy.loginfo("Servo centered at GPIO 13")

        # ── ROS subscribers ──────────────────────────────────────────────
        rospy.Subscriber('/AutoModelMini/manual_control/speed',    Int16,  self.speed_callback)
        rospy.Subscriber('/AutoModelMini/manual_control/steering', Int16,  self.steering_callback)
        rospy.Subscriber('/vehicle_mode',                          String, self.mode_callback)
        rospy.Subscriber('/emergency_stop',                        Bool,   self.emergency_callback)

        rospy.loginfo("Motor Driver initialized (Encoder Motor Module via I2C)")

    # ── Hardware setup ───────────────────────────────────────────────────────
    def setup_motor_module(self):
        """Configure motor type on the Encoder Motor Module."""
        try:
            # Set motor type: JGB37-520 12V 110RPM
            self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_TYPE_ADDR,
                                          [MOTOR_TYPE_JGB37_520_12V_110RPM])
            time.sleep(0.05)
            # Default encoder polarity
            self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR, [0])
            time.sleep(0.05)
            rospy.loginfo("Encoder Motor Module configured (type JGB37-520)")
        except Exception as e:
            rospy.logerr(f"Motor module setup error: {e}")

    def _write_motor_speeds(self, m1: int, m2: int, m3: int = 0, m4: int = 0):
        """
        Write speeds to all 4 motor channels.
        Values are signed bytes (-100 to 100).
        The module uses fixed-speed (closed-loop) control.
        m1/m2 = left/right drive motors.  m3/m4 unused (set to 0).
        """
        # smbus write_i2c_block_data needs unsigned bytes;
        # convert signed to unsigned two's complement.
        def to_u8(v):
            v = max(-100, min(100, int(v)))
            return v & 0xFF

        data = [to_u8(m1), to_u8(m2), to_u8(m3), to_u8(m4)]
        try:
            self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, data)
        except Exception as e:
            rospy.logerr(f"I2C write error: {e}")

    # ── ROS callbacks ────────────────────────────────────────────────────────
    def speed_callback(self, msg):
        """
        Speed from Master controller.
        Convention (unchanged from original):
          negative → forward,  0 → stop,  positive → reverse
        Range: -500 to 500  (mapped to -100..100 for motor module)
        """
        # BUG FIX: was 'if not self.autonomous_mode and not self.emergency_stop'
        # which blocked commands in autonomous mode.
        if not self.autonomous_mode or self.emergency_stop:
            return

        speed = msg.data
        self.current_speed = speed
        self.set_motor_speed(speed)

    def steering_callback(self, msg):
        """Steering angle 45-135 degrees (90 = center)."""
        # BUG FIX: same inverted condition as speed_callback.
        if not self.autonomous_mode or self.emergency_stop:
            return

        angle = max(45, min(135, int(msg.data)))
        self.current_angle = angle
        self.set_servo_angle(angle)

    def mode_callback(self, msg):
        mode = msg.data
        if mode == "AUTONOMOUS":
            self.autonomous_mode = True
            rospy.loginfo("Switched to AUTONOMOUS mode")
        elif mode == "TELEOP":
            self.autonomous_mode = False
            rospy.loginfo("Switched to TELEOP mode")

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            rospy.logwarn("EMERGENCY STOP ACTIVATED!")
            self.stop_motors()

    # ── Actuator helpers ─────────────────────────────────────────────────────
    def set_motor_speed(self, speed: int):
        """
        Map Master speed (-500..500) to motor module speed (-100..100).
        Negative input  → forward  (both motors positive).
        Positive input  → reverse  (both motors negative).
        Zero            → stop.
        """
        if speed == 0:
            self._write_motor_speeds(0, 0)
            rospy.logdebug("Motors: STOP")
            return

        # Scale -500..500 → -100..100
        motor_val = int((speed / 500.0) * 100)
        motor_val = max(-100, min(100, motor_val))

        # Negative master speed = forward = positive motor command
        m_speed = -motor_val

        self._write_motor_speeds(m_speed, m_speed)
        direction = "FORWARD" if m_speed > 0 else "REVERSE"
        rospy.logdebug(f"Motors: {direction}, value={m_speed}")

    def set_servo_angle(self, angle: int):
        """Map angle (45-135°) to servo pulse width (1000-2000 µs)."""
        pulse = self.map_range(angle, 45, 135,
                               self.SERVO_MIN_PULSE,
                               self.SERVO_MAX_PULSE)
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, int(pulse))
        rospy.logdebug(f"Servo: {angle}°, pulse={int(pulse)}µs")

    def stop_motors(self):
        """Immediate motor stop."""
        self._write_motor_speeds(0, 0)
        rospy.loginfo("Motors stopped")

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # ── Cleanup ──────────────────────────────────────────────────────────────
    def cleanup(self):
        rospy.loginfo("Cleaning up...")
        self.stop_motors()
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, 0)
        time.sleep(0.3)
        self.pi.stop()
        self.bus.close()
        rospy.loginfo("Cleanup complete")

    def run(self):
        rate = rospy.Rate(50)
        try:
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down motor driver...")
        finally:
            self.cleanup()


if __name__ == '__main__':
    try:
        driver = MotorDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
