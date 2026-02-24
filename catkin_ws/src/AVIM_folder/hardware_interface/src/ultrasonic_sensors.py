#!/usr/bin/env python3
"""
Ultrasonic Sensors Reader
Reads 4 HC-SR04 sensors and publishes distances
"""

import rospy
from sensor_msgs.msg import Range
import pigpio
import time

class UltrasonicSensors:
    def __init__(self):
        rospy.init_node('ultrasonic_sensors')
        
        # GPIO pins (Trigger, Echo pairs)
        self.sensors = {
            'front': {'trigger': 5, 'echo': 6},
            'back': {'trigger': 16, 'echo': 20},
            'left': {'trigger': 21, 'echo': 26},
            'right': {'trigger': 19, 'echo': 12}
        }
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            rospy.logerr("pigpio not connected!")
            exit(1)
        
        # Setup GPIO
        for name, pins in self.sensors.items():
            self.pi.set_mode(pins['trigger'], pigpio.OUTPUT)
            self.pi.set_mode(pins['echo'], pigpio.INPUT)
            self.pi.write(pins['trigger'], 0)
        
        # Publishers
        self.pubs = {}
        for name in self.sensors.keys():
            self.pubs[name] = rospy.Publisher(
                f'/sensors/ultrasonic/{name}', 
                Range, 
                queue_size=1
            )
        
        rospy.loginfo("Ultrasonic sensors initialized")
        
    def measure_distance(self, trigger_pin, echo_pin):
        """
        Measure distance with HC-SR04
        Returns distance in meters (0.02 to 4.0m range)
        """
        # Send trigger pulse
        self.pi.gpio_trigger(trigger_pin, 10, 1)  # 10us pulse
        
        # Wait for echo
        timeout = time.time() + 0.1  # 100ms timeout
        
        # Wait for echo start
        while self.pi.read(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return -1  # Timeout
        
        # Wait for echo end
        while self.pi.read(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return -1  # Timeout
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound / 2
        distance = distance / 100  # Convert cm to meters
        
        return distance if 0.02 <= distance <= 4.0 else -1
        
    def publish_measurements(self):
        """Read all sensors and publish"""
        for name, pins in self.sensors.items():
            distance = self.measure_distance(pins['trigger'], pins['echo'])
            
            # Create Range message
            msg = Range()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = f"ultrasonic_{name}"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26  # ~15 degrees
            msg.min_range = 0.02
            msg.max_range = 4.0
            msg.range = distance if distance > 0 else msg.max_range
            
            self.pubs[name].publish(msg)
            
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            self.publish_measurements()
            rate.sleep()
            
        self.pi.stop()

if __name__ == '__main__':
    try:
        sensors = UltrasonicSensors()
        sensors.run()
    except rospy.ROSInterruptException:
        pass
