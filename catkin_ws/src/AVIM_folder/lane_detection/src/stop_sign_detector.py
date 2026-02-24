#!/usr/bin/env python3
"""
STOP Sign Detector for TMR 2026
Two detection methods:
1. Color + Shape (simple, fast, works without training)
2. YOLO (more robust, requires training)

Uses method 1 by default, can switch to YOLO if available
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

class StopSignDetector:
    def __init__(self):
        rospy.init_node('stop_sign_detector', anonymous=False)
        
        # Detection method
        self.use_yolo = rospy.get_param('~use_yolo', False)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Detection state
        self.stop_detected = False
        self.detection_threshold = 3  # Consecutive frames to confirm
        self.detection_counter = 0
        
        # YOLO model (if enabled)
        self.yolo_net = None
        if self.use_yolo:
            self.load_yolo_model()
        
        # Publishers
        self.detection_pub = rospy.Publisher('/stop_sign_detected', Bool, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/stop_sign_detector/debug_image', Image, queue_size=1)
        
        # Subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1)
        
        rospy.loginfo(f"STOP Sign Detector initialized (Method: {'YOLO' if self.use_yolo else 'Color+Shape'})")
        
    def load_yolo_model(self):
        """Load YOLO model for STOP sign detection"""
        try:
            # Paths to YOLO files (you need to train and provide these)
            model_config = rospy.get_param('~yolo_config', '/home/claude/yolo_stop/yolov5s.cfg')
            model_weights = rospy.get_param('~yolo_weights', '/home/claude/yolo_stop/yolov5s.weights')
            class_names = rospy.get_param('~yolo_classes', '/home/claude/yolo_stop/classes.names')
            
            self.yolo_net = cv2.dnn.readNetFromDarknet(model_config, model_weights)
            self.yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            
            with open(class_names, 'r') as f:
                self.yolo_classes = [line.strip() for line in f.readlines()]
                
            rospy.loginfo("YOLO model loaded successfully")
            
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            rospy.logwarn("Falling back to Color+Shape detection")
            self.use_yolo = False
            
    def image_callback(self, msg):
        """Process incoming image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect STOP sign
            if self.use_yolo:
                detected, debug_image = self.detect_yolo(cv_image)
            else:
                detected, debug_image = self.detect_color_shape(cv_image)
            
            # Confirmation logic (require N consecutive detections)
            if detected:
                self.detection_counter += 1
                if self.detection_counter >= self.detection_threshold:
                    if not self.stop_detected:
                        self.stop_detected = True
                        self.publish_detection(True)
                        rospy.loginfo("🛑 STOP SIGN CONFIRMED!")
            else:
                if self.detection_counter > 0:
                    self.detection_counter -= 1
                if self.stop_detected and self.detection_counter == 0:
                    self.stop_detected = False
                    self.publish_detection(False)
                    rospy.loginfo("✅ STOP sign no longer visible")
            
            # Publish debug image
            if debug_image is not None:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                    self.debug_image_pub.publish(debug_msg)
                except CvBridgeError as e:
                    rospy.logerr(f"Debug image publish error: {e}")
                    
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            
    def detect_color_shape(self, image):
        """
        Detect STOP sign using color (red) and shape (octagon)
        Returns: (detected: bool, debug_image: np.array)
        """
        debug_image = image.copy()
        detected = False
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color (STOP signs are red)
        # Red wraps around in HSV, so we need two ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphological operations to remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Filter by area
            area = cv2.contourArea(contour)
            if area < 500:  # Minimum area threshold
                continue
            
            # Approximate contour to polygon
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Check if it's an octagon (8 sides) or close to it (6-10 sides)
            num_sides = len(approx)
            
            if 6 <= num_sides <= 10:
                # Additional check: circularity (octagons are quite circular)
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                
                if circularity > 0.5:  # Reasonably circular
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h
                    
                    # STOP signs are roughly square
                    if 0.7 < aspect_ratio < 1.3:
                        detected = True
                        
                        # Draw detection on debug image
                        cv2.drawContours(debug_image, [approx], 0, (0, 255, 0), 3)
                        cv2.rectangle(debug_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.putText(debug_image, "STOP", (x, y-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        
                        # Log details
                        rospy.logdebug(f"STOP candidate: sides={num_sides}, area={area:.0f}, "
                                     f"circularity={circularity:.2f}, aspect={aspect_ratio:.2f}")
        
        # Draw status on debug image
        status_text = "STOP DETECTED!" if detected else "Scanning..."
        color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(debug_image, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(debug_image, f"Counter: {self.detection_counter}/{self.detection_threshold}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return detected, debug_image
        
    def detect_yolo(self, image):
        """
        Detect STOP sign using YOLO
        Returns: (detected: bool, debug_image: np.array)
        """
        debug_image = image.copy()
        detected = False
        
        if self.yolo_net is None:
            return False, debug_image
        
        # Prepare image for YOLO
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.yolo_net.setInput(blob)
        
        # Get output layer names
        layer_names = self.yolo_net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.yolo_net.getUnconnectedOutLayers()]
        
        # Forward pass
        outputs = self.yolo_net.forward(output_layers)
        
        # Process detections
        height, width = image.shape[:2]
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                # Filter by confidence and class (assuming class 0 is STOP sign)
                if confidence > 0.5 and class_id == 0:
                    detected = True
                    
                    # Get bounding box
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    
                    # Draw on debug image
                    cv2.rectangle(debug_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    label = f"STOP {confidence:.2f}"
                    cv2.putText(debug_image, label, (x, y-10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return detected, debug_image
        
    def publish_detection(self, detected):
        """Publish detection status"""
        msg = Bool()
        msg.data = detected
        self.detection_pub.publish(msg)
        
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = StopSignDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
