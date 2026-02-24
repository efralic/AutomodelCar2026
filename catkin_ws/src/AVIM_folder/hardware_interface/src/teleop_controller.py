#!/usr/bin/env python3
"""
Teleop Node for TMR 2026
Control vehicle with Logitech F710 or Xbox controller
Switch between AUTONOMOUS and TELEOP modes
"""

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String, Bool

class TeleopController:
    def __init__(self):
        rospy.init_node('teleop_controller', anonymous=False)
        
        # Mode state
        self.autonomous_mode = True
        self.last_button_state = 0
        
        # Control limits
        self.max_speed = 500  # Maximum speed value
        self.max_angle = 45   # Maximum steering angle from center (±45°)
        
        # Deadzone for joysticks
        self.deadzone = 0.1
        
        # Publishers
        self.speed_pub = rospy.Publisher('/AutoModelMini/manual_control/speed', Int16, queue_size=1)
        self.steering_pub = rospy.Publisher('/AutoModelMini/manual_control/steering', Int16, queue_size=1)
        self.mode_pub = rospy.Publisher('/vehicle_mode', String, queue_size=1)
        self.emergency_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        
        # Subscriber
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)
        
        # Publish initial mode
        self.publish_mode()
        
        rospy.loginfo("Teleop Controller initialized")
        rospy.loginfo("Controls:")
        rospy.loginfo("  Left Stick Vertical: Speed")
        rospy.loginfo("  Left Stick Horizontal: Steering")
        rospy.loginfo("  Start Button: Toggle Autonomous/Teleop")
        rospy.loginfo("  Back Button: Emergency Stop")
        rospy.loginfo(f"Current mode: {'AUTONOMOUS' if self.autonomous_mode else 'TELEOP'}")
        
    def joy_callback(self, joy_msg):
        """
        Process joystick input
        
        Logitech F710 / Xbox Controller mapping:
        Axes:
          0: Left stick horizontal (steering)
          1: Left stick vertical (speed)
          2: Left trigger
          3: Right stick horizontal
          4: Right stick vertical
          5: Right trigger
        
        Buttons:
          0: A
          1: B
          2: X
          3: Y
          4: LB
          5: RB
          6: Back
          7: Start
          8: Logitech/Xbox button
          9: Left stick click
          10: Right stick click
        """
        
        # Button 7: Start - Toggle mode
        if joy_msg.buttons[7] == 1 and self.last_button_state == 0:
            self.toggle_mode()
        self.last_button_state = joy_msg.buttons[7]
        
        # Button 6: Back - Emergency stop
        if joy_msg.buttons[6] == 1:
            self.emergency_stop()
            return
        
        # Only send commands in TELEOP mode
        if not self.autonomous_mode:
            # Get joystick values
            steering_raw = joy_msg.axes[0]  # Left stick horizontal
            speed_raw = joy_msg.axes[1]     # Left stick vertical
            
            # Apply deadzone
            if abs(steering_raw) < self.deadzone:
                steering_raw = 0.0
            if abs(speed_raw) < self.deadzone:
                speed_raw = 0.0
            
            # Convert to command values
            # Steering: -1 to 1 → 45° to 135° (90° = center)
            angle = int(90 + (steering_raw * self.max_angle))
            angle = max(45, min(135, angle))
            
            # Speed: -1 to 1 → -500 to 500 (negative = forward in your system)
            # Invert because joystick up is positive, but your system uses negative for forward
            speed = int(-speed_raw * self.max_speed)
            
            # Publish commands
            self.publish_speed(speed)
            self.publish_steering(angle)
            
            # Debug info (only when moving)
            if speed != 0 or angle != 90:
                rospy.logdebug(f"TELEOP: Speed={speed}, Angle={angle}°")
    
    def toggle_mode(self):
        """Toggle between AUTONOMOUS and TELEOP modes"""
        self.autonomous_mode = not self.autonomous_mode
        self.publish_mode()
        
        if self.autonomous_mode:
            rospy.loginfo("🤖 Switched to AUTONOMOUS mode")
        else:
            rospy.logwarn("🎮 Switched to TELEOP mode - Manual control active")
            
    def emergency_stop(self):
        """Trigger emergency stop"""
        rospy.logwarn("🚨 EMERGENCY STOP TRIGGERED!")
        
        # Stop motors immediately
        self.publish_speed(0)
        self.publish_steering(90)
        
        # Publish emergency stop flag
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        
        # After 1 second, release emergency stop
        rospy.Timer(rospy.Duration(1.0), self.release_emergency, oneshot=True)
        
    def release_emergency(self, event):
        """Release emergency stop"""
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_pub.publish(emergency_msg)
        rospy.loginfo("Emergency stop released")
        
    def publish_speed(self, speed):
        """Publish speed command"""
        msg = Int16()
        msg.data = speed
        self.speed_pub.publish(msg)
        
    def publish_steering(self, angle):
        """Publish steering command"""
        msg = Int16()
        msg.data = angle
        self.steering_pub.publish(msg)
        
    def publish_mode(self):
        """Publish current mode"""
        msg = String()
        msg.data = "AUTONOMOUS" if self.autonomous_mode else "TELEOP"
        self.mode_pub.publish(msg)
        
    def run(self):
        """Main loop"""
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():
            # Periodically republish mode (in case someone missed it)
            self.publish_mode()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TeleopController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
