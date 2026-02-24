#!/usr/bin/env python3
"""
Motor Driver for Raspberry Pi 4
Controls DC motors via L298N driver and servo for steering
Uses pigpio library for hardware PWM
"""

import rospy
from std_msgs.msg import Int16, Bool, String
import pigpio
import time

class MotorDriver:
    def __init__(self):
        rospy.init_node('motor_driver', anonymous=False)
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("Failed to connect to pigpio daemon! Run: sudo pigpiod")
            exit(1)
        
        # GPIO Pin Configuration for L298N Motor Driver
        # Motor Left
        self.MOTOR_LEFT_IN1 = 17  # Direction pin 1
        self.MOTOR_LEFT_IN2 = 18  # Direction pin 2
        self.MOTOR_LEFT_ENA = 27  # PWM speed control
        
        # Motor Right
        self.MOTOR_RIGHT_IN3 = 22  # Direction pin 1
        self.MOTOR_RIGHT_IN4 = 23  # Direction pin 2
        self.MOTOR_RIGHT_ENB = 24  # PWM speed control
        
        # Servo for steering (hardware PWM)
        self.SERVO_PIN = 13  # GPIO 13 supports hardware PWM
        
        # Setup GPIO modes
        self.setup_gpio()
        
        # Vehicle parameters
        self.current_speed = 0
        self.current_angle = 90  # Center position
        self.autonomous_mode = True
        self.emergency_stop = False
        
        # PWM frequency
        self.PWM_FREQ = 1000  # 1kHz for motors
        
        # Servo parameters (typical RC servo: 1000-2000 us pulse width)
        self.SERVO_MIN_PULSE = 1000  # 45 degrees
        self.SERVO_MAX_PULSE = 2000  # 135 degrees
        self.SERVO_CENTER = 1500     # 90 degrees
        
        # Subscribers
        rospy.Subscriber('/AutoModelMini/manual_control/speed', Int16, self.speed_callback)
        rospy.Subscriber('/AutoModelMini/manual_control/steering', Int16, self.steering_callback)
        rospy.Subscriber('/vehicle_mode', String, self.mode_callback)
        rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)
        
        rospy.loginfo("Motor Driver initialized successfully")
        rospy.loginfo("Autonomous mode: ON")
        
    def setup_gpio(self):
        """Configure all GPIO pins"""
        # Motor Left pins
        self.pi.set_mode(self.MOTOR_LEFT_IN1, pigpio.OUTPUT)
        self.pi.set_mode(self.MOTOR_LEFT_IN2, pigpio.OUTPUT)
        self.pi.set_mode(self.MOTOR_LEFT_ENA, pigpio.OUTPUT)
        
        # Motor Right pins
        self.pi.set_mode(self.MOTOR_RIGHT_IN3, pigpio.OUTPUT)
        self.pi.set_mode(self.MOTOR_RIGHT_IN4, pigpio.OUTPUT)
        self.pi.set_mode(self.MOTOR_RIGHT_ENB, pigpio.OUTPUT)
        
        # Initialize all to LOW
        self.pi.write(self.MOTOR_LEFT_IN1, 0)
        self.pi.write(self.MOTOR_LEFT_IN2, 0)
        self.pi.write(self.MOTOR_RIGHT_IN3, 0)
        self.pi.write(self.MOTOR_RIGHT_IN4, 0)
        
        # Set PWM frequency for motor enable pins
        self.pi.set_PWM_frequency(self.MOTOR_LEFT_ENA, self.PWM_FREQ)
        self.pi.set_PWM_frequency(self.MOTOR_RIGHT_ENB, self.PWM_FREQ)
        
        # Set PWM range (0-255)
        self.pi.set_PWM_range(self.MOTOR_LEFT_ENA, 255)
        self.pi.set_PWM_range(self.MOTOR_RIGHT_ENB, 255)
        
        # Initialize servo to center position
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, self.SERVO_CENTER)
        rospy.loginfo("GPIO pins configured")
        
    def speed_callback(self, msg):
        """
        Handle speed commands from Master controller
        Range: -500 to 0 (negative = forward, 0 = stop)
        """
        if not self.autonomous_mode and not self.emergency_stop:
            return
            
        if self.emergency_stop:
            self.stop_motors()
            return
            
        speed = msg.data
        self.current_speed = speed
        self.set_motor_speed(speed)
        
    def steering_callback(self, msg):
        """
        Handle steering commands
        Range: 45-135 degrees (90 = center)
        """
        if not self.autonomous_mode and not self.emergency_stop:
            return
            
        angle = msg.data
        # Clamp angle to safe range
        angle = max(45, min(135, angle))
        self.current_angle = angle
        self.set_servo_angle(angle)
        
    def mode_callback(self, msg):
        """Switch between AUTONOMOUS and TELEOP modes"""
        mode = msg.data
        if mode == "AUTONOMOUS":
            self.autonomous_mode = True
            rospy.loginfo("Switched to AUTONOMOUS mode")
        elif mode == "TELEOP":
            self.autonomous_mode = False
            rospy.loginfo("Switched to TELEOP mode")
            
    def emergency_callback(self, msg):
        """Emergency stop handler"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            rospy.logwarn("EMERGENCY STOP ACTIVATED!")
            self.stop_motors()
            
    def set_motor_speed(self, speed):
        """
        Set motor speed
        speed: -500 to 0 (negative = forward, 0 = stop, positive = reverse)
        """
        if speed < 0:
            # Forward
            direction = "FORWARD"
            pwm_value = abs(speed)
            # Scale from 0-500 to 0-255
            pwm_value = int((pwm_value / 500.0) * 255)
            pwm_value = min(255, pwm_value)
            
            # Set direction: Forward
            self.pi.write(self.MOTOR_LEFT_IN1, 1)
            self.pi.write(self.MOTOR_LEFT_IN2, 0)
            self.pi.write(self.MOTOR_RIGHT_IN3, 1)
            self.pi.write(self.MOTOR_RIGHT_IN4, 0)
            
        elif speed > 0:
            # Reverse
            direction = "REVERSE"
            pwm_value = abs(speed)
            pwm_value = int((pwm_value / 500.0) * 255)
            pwm_value = min(255, pwm_value)
            
            # Set direction: Reverse
            self.pi.write(self.MOTOR_LEFT_IN1, 0)
            self.pi.write(self.MOTOR_LEFT_IN2, 1)
            self.pi.write(self.MOTOR_RIGHT_IN3, 0)
            self.pi.write(self.MOTOR_RIGHT_IN4, 1)
            
        else:
            # Stop
            direction = "STOP"
            pwm_value = 0
            self.pi.write(self.MOTOR_LEFT_IN1, 0)
            self.pi.write(self.MOTOR_LEFT_IN2, 0)
            self.pi.write(self.MOTOR_RIGHT_IN3, 0)
            self.pi.write(self.MOTOR_RIGHT_IN4, 0)
        
        # Set PWM speed
        self.pi.set_PWM_dutycycle(self.MOTOR_LEFT_ENA, pwm_value)
        self.pi.set_PWM_dutycycle(self.MOTOR_RIGHT_ENB, pwm_value)
        
        rospy.logdebug(f"Motor: {direction}, PWM: {pwm_value}")
        
    def set_servo_angle(self, angle):
        """
        Set servo angle
        angle: 45-135 degrees (90 = center)
        """
        # Map angle (45-135) to pulse width (1000-2000 us)
        pulse_width = self.map_range(angle, 45, 135, 
                                     self.SERVO_MIN_PULSE, 
                                     self.SERVO_MAX_PULSE)
        pulse_width = int(pulse_width)
        
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, pulse_width)
        rospy.logdebug(f"Servo: {angle}°, Pulse: {pulse_width}us")
        
    def stop_motors(self):
        """Emergency stop - cut all motor power"""
        self.pi.write(self.MOTOR_LEFT_IN1, 0)
        self.pi.write(self.MOTOR_LEFT_IN2, 0)
        self.pi.write(self.MOTOR_RIGHT_IN3, 0)
        self.pi.write(self.MOTOR_RIGHT_IN4, 0)
        self.pi.set_PWM_dutycycle(self.MOTOR_LEFT_ENA, 0)
        self.pi.set_PWM_dutycycle(self.MOTOR_RIGHT_ENB, 0)
        rospy.loginfo("Motors stopped")
        
    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        """Map value from one range to another"""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
    def cleanup(self):
        """Cleanup GPIO on shutdown"""
        rospy.loginfo("Cleaning up GPIO...")
        self.stop_motors()
        self.pi.set_servo_pulsewidth(self.SERVO_PIN, 0)  # Turn off servo
        time.sleep(0.5)
        self.pi.stop()
        rospy.loginfo("GPIO cleanup complete")
        
    def run(self):
        """Main loop"""
        rate = rospy.Rate(50)  # 50 Hz
        
        try:
            while not rospy.is_shutdown():
                # Monitor temperature (optional but recommended)
                # You can add temperature monitoring here
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
