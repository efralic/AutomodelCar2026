#!/usr/bin/env python3
"""
Lights Controller for TMR 2026
Controls 4 LEDs: left/right blinkers and turn signals
"""

import rospy
from std_msgs.msg import Bool
import pigpio
import threading
import time

class LightsController:
    def __init__(self):
        rospy.init_node('lights_controller')
        
        # GPIO pins for LEDs
        self.LED_LEFT_BLINKER = 14
        self.LED_RIGHT_BLINKER = 15
        self.LED_LEFT_SIGNAL = 25
        self.LED_RIGHT_SIGNAL = 8
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("pigpio daemon not running!")
            exit(1)
        
        # Setup pins
        self.pi.set_mode(self.LED_LEFT_BLINKER, pigpio.OUTPUT)
        self.pi.set_mode(self.LED_RIGHT_BLINKER, pigpio.OUTPUT)
        self.pi.set_mode(self.LED_LEFT_SIGNAL, pigpio.OUTPUT)
        self.pi.set_mode(self.LED_RIGHT_SIGNAL, pigpio.OUTPUT)
        
        # State variables
        self.blinkers_active = False
        self.left_signal_active = False
        self.right_signal_active = False
        
        # Threads
        self.blinker_thread = None
        
        # Subscribers
        rospy.Subscriber('/lights/blinkers', Bool, self.blinkers_callback)
        rospy.Subscriber('/lights/left_signal', Bool, self.left_signal_callback)
        rospy.Subscriber('/lights/right_signal', Bool, self.right_signal_callback)
        
        rospy.loginfo("Lights Controller initialized")
        
    def blinkers_callback(self, msg):
        """Activate/deactivate both blinkers (for STOP sign)"""
        self.blinkers_active = msg.data
        
        if self.blinkers_active:
            if self.blinker_thread is None or not self.blinker_thread.is_alive():
                self.blinker_thread = threading.Thread(target=self.blink_both)
                self.blinker_thread.daemon = True
                self.blinker_thread.start()
        else:
            self.pi.write(self.LED_LEFT_BLINKER, 0)
            self.pi.write(self.LED_RIGHT_BLINKER, 0)
            
    def left_signal_callback(self, msg):
        """Activate left turn signal"""
        self.left_signal_active = msg.data
        self.pi.write(self.LED_LEFT_SIGNAL, 1 if msg.data else 0)
        
    def right_signal_callback(self, msg):
        """Activate right turn signal"""
        self.right_signal_active = msg.data
        self.pi.write(self.LED_RIGHT_SIGNAL, 1 if msg.data else 0)
        
    def blink_both(self):
        """Blink both blinkers at 1 Hz"""
        while self.blinkers_active and not rospy.is_shutdown():
            self.pi.write(self.LED_LEFT_BLINKER, 1)
            self.pi.write(self.LED_RIGHT_BLINKER, 1)
            time.sleep(0.5)
            self.pi.write(self.LED_LEFT_BLINKER, 0)
            self.pi.write(self.LED_RIGHT_BLINKER, 0)
            time.sleep(0.5)
            
    def cleanup(self):
        """Turn off all LEDs"""
        self.pi.write(self.LED_LEFT_BLINKER, 0)
        self.pi.write(self.LED_RIGHT_BLINKER, 0)
        self.pi.write(self.LED_LEFT_SIGNAL, 0)
        self.pi.write(self.LED_RIGHT_SIGNAL, 0)
        self.pi.stop()
        
    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    try:
        controller = LightsController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'controller' in locals():
            controller.cleanup()
