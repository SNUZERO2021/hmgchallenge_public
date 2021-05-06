#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ctypes import *
import random
import os
import time
import darknet
import argparse
from threading import Thread, enumerate
import rospkg

def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection")
    
    parser.add_argument("--weights", default="yolov4.weights",
                        help="yolo weights path")
   
    parser.add_argument("--config_file", default="./cfg/yolov4.cfg",
                        help="path to config file")
    parser.add_argument("--data_file", default="./cfg/coco.data",
                        help="path to data file")
    parser.add_argument("--thresh", type=float, default=.25,
                        help="remove detections with confidence below this value")
    return parser.parse_args()


def str2int(video_path):
    """
    argparse returns and string althout webcam uses int (0, 1 ...)
    Cast to int if needed
    """
    try:
        return int(video_path)
    except ValueError:
        return video_path


def check_arguments_errors(args):
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
 
class ImageConverter:
    def __init__(self):
        self.image_sub =  rospy.Subscriber("/color1/image_raw",Image, self.callback)
        self.pub = rospy.Publisher('/tmp', Int32, queue_size=10)
        self.bridge = CvBridge()
        
        r = rospkg.RosPack()
        path = r.get_path('yolo') + '/config/libdarknet.so'
        self.config_path = r.get_path('yolo') + '/config/yolo-obj.cfg'
        self.weight_path = r.get_path('yolo') + '/config/yolo.weights'
        self.data_path = r.get_path('yolo') + '/config/obj.data'
     
        self.network, self.class_names, self.class_colors = darknet.load_network(
                self.config_path,
                self.data_path,
                self.weight_path,
                batch_size=1
            )
        self.thresh = 0.50
        
        self.width = darknet.network_width(self.network)
        self.height = darknet.network_height(self.network)
        print('complete to load')
        
    def callback(self, msg):
        begin = time.clock()
        frame=self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #cv2.imshow("frame_rgb", frame_rgb)
        frame_resized = cv2.resize(frame_rgb, (self.width, self.height),
                                   interpolation=cv2.INTER_LINEAR)
        #cv2.imshow("frame_resized", frame_resized)
        img_for_detect = darknet.make_image(self.width, self.height, 3)
        darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
        #print(img_for_detect.__dict__)
        detections = darknet.detect_image(self.network, self.class_names, img_for_detect, thresh = self.thresh)
        #print(detections)
        
        image = darknet.draw_boxes(detections, frame_resized, self.class_colors)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        end = time.clock()
        print(end-begin)

        cv2.imshow('Inference', image)
        cv2.imshow("Image Window", frame)
        cv2.waitKey(3)


def main(args):
    print('start node')
    image_converter = ImageConverter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)