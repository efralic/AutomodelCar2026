#!/usr/bin/env python3
"""
STOP Sign Detector for TMR 2026
Two detection methods:
1. Color + Shape (simple, fast, works without training)
2. YOLO (more robust, requires training)
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

        self.use_yolo = rospy.get_param('~use_yolo', False)
        self.bridge   = CvBridge()

        self.stop_detected       = False
        self.detection_threshold = 3
        self.detection_counter   = 0

        self.yolo_net = None
        if self.use_yolo:
            self.load_yolo_model()

        self.detection_pub  = rospy.Publisher('/stop_sign_detected', Bool,  queue_size=1)
        self.debug_image_pub= rospy.Publisher('/stop_sign_detector/debug_image',
                                              Image, queue_size=1)

        # ── FIX: topic para cámara web USB ───────────────────────────────
        cam_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        self.image_sub = rospy.Subscriber(cam_topic, Image,
                                          self.image_callback, queue_size=1)

        rospy.loginfo(f"STOP Sign Detector initialized "
                      f"(Method: {'YOLO' if self.use_yolo else 'Color+Shape'})")
        rospy.loginfo(f"Subscribed to camera: {cam_topic}")

    def load_yolo_model(self):
        try:
            model_config  = rospy.get_param('~yolo_config',
                            '/home/ubuntu/yolo_stop/yolov5s.cfg')
            model_weights = rospy.get_param('~yolo_weights',
                            '/home/ubuntu/yolo_stop/yolov5s.weights')
            class_names   = rospy.get_param('~yolo_classes',
                            '/home/ubuntu/yolo_stop/classes.names')
            self.yolo_net = cv2.dnn.readNetFromDarknet(model_config, model_weights)
            self.yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            with open(class_names,'r') as f:
                self.yolo_classes = [l.strip() for l in f.readlines()]
            rospy.loginfo("YOLO model loaded")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO: {e}")
            rospy.logwarn("Falling back to Color+Shape detection")
            self.use_yolo = False

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        if self.use_yolo:
            detected, debug_image = self.detect_yolo(cv_image)
        else:
            detected, debug_image = self.detect_color_shape(cv_image)

        if detected:
            self.detection_counter += 1
            if self.detection_counter >= self.detection_threshold:
                if not self.stop_detected:
                    self.stop_detected = True
                    self.publish_detection(True)
                    rospy.loginfo("STOP SIGN CONFIRMED!")
        else:
            if self.detection_counter > 0:
                self.detection_counter -= 1
            if self.stop_detected and self.detection_counter == 0:
                self.stop_detected = False
                self.publish_detection(False)
                rospy.loginfo("STOP sign no longer visible")

        if debug_image is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Debug image error: {e}")

    def detect_color_shape(self, image):
        debug_image = image.copy()
        detected    = False
        hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask1= cv2.inRange(hsv, np.array([0,100,100]),   np.array([10,255,255]))
        mask2= cv2.inRange(hsv, np.array([160,100,100]), np.array([180,255,255]))
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500: continue
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx  = cv2.approxPolyDP(contour, epsilon, True)
            num_sides = len(approx)
            if 6 <= num_sides <= 10:
                perimeter    = cv2.arcLength(contour, True)
                circularity  = 4*np.pi*area/(perimeter*perimeter)
                if circularity > 0.5:
                    x,y,w,h = cv2.boundingRect(contour)
                    aspect  = float(w)/h
                    if 0.7 < aspect < 1.3:
                        detected = True
                        cv2.drawContours(debug_image,[approx],0,(0,255,0),3)
                        cv2.rectangle(debug_image,(x,y),(x+w,y+h),(0,255,0),2)
                        cv2.putText(debug_image,"STOP",(x,y-10),
                                    cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,0),2)
        status = "STOP DETECTED!" if detected else "Scanning..."
        color  = (0,255,0) if detected else (0,0,255)
        cv2.putText(debug_image, status,(10,30),
                    cv2.FONT_HERSHEY_SIMPLEX,1,color,2)
        cv2.putText(debug_image,
                    f"Counter:{self.detection_counter}/{self.detection_threshold}",
                    (10,60), cv2.FONT_HERSHEY_SIMPLEX,0.7,color,2)
        return detected, debug_image

    def detect_yolo(self, image):
        debug_image = image.copy()
        detected    = False
        if self.yolo_net is None: return False, debug_image
        blob = cv2.dnn.blobFromImage(image,1/255.0,(416,416),
                                     swapRB=True,crop=False)
        self.yolo_net.setInput(blob)
        layer_names  = self.yolo_net.getLayerNames()
        output_layers= [layer_names[i-1]
                        for i in self.yolo_net.getUnconnectedOutLayers()]
        outputs = self.yolo_net.forward(output_layers)
        h,w = image.shape[:2]
        for output in outputs:
            for detection in output:
                scores     = detection[5:]
                class_id   = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and class_id == 0:
                    detected   = True
                    cx = int(detection[0]*w); cy = int(detection[1]*h)
                    bw = int(detection[2]*w); bh = int(detection[3]*h)
                    x  = int(cx-bw/2);        y  = int(cy-bh/2)
                    cv2.rectangle(debug_image,(x,y),(x+bw,y+bh),(0,255,0),2)
                    cv2.putText(debug_image,f"STOP {confidence:.2f}",
                                (x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
        return detected, debug_image

    def publish_detection(self, detected):
        msg = Bool(); msg.data = detected
        self.detection_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = StopSignDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
