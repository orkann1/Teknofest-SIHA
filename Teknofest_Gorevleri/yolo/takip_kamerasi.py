# Sena Berra Soydugan

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import torch
import time
import datetime

class PerceptionNode:
    def __init__(self, model):
        self.model = model
        self.bridge = CvBridge()
        self.last_capture_time = None
        self.capture_interval = rospy.Duration.from_sec(0.03)

        self.center_x = None
        self.center_y = None
        self.size_x = None
        self.size_y = None

        self.last_position = None 
        self.last_time = 0.0

        self.start_time = time.time()
        self.show_time = time.time() - 3
        self.successful_locks = 0

        # ROS subscribers
        self.image_sub = rospy.Subscriber('/plane0/usb_cam/image_raw', Image, self.camera_callback)

        # ROS publishers
        self.center_pub = rospy.Publisher('/center_coordinates', Int32MultiArray, queue_size=20)
        self.size_pub = rospy.Publisher('/size', Float32MultiArray, queue_size=20)

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            current_time = rospy.Time.now()
            if self.last_capture_time is None or (current_time - self.last_capture_time) >= self.capture_interval:
                detect_img = self.detect_image(cv_image)
                self.last_capture_time = current_time
                self.show_image(detect_img)

        except Exception as e:
            rospy.logerr('Error processing image: %s' % str(e))

    def detect_image(self, cv_image):
        try:
            results = self.model([cv_image], size=640)
            detected_image = cv_image.copy()
            screen_height, screen_width, _ = detected_image.shape

            for pred in results.pred[0]:
                x1, y1, x2, y2, conf, cls = pred.tolist()
                x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)

                if conf < 0.50:
                    continue

                if x1 is not None and x2 is not None and y1 is not None and y2 is not None:
                    self.center_x = int((x1 + x2) / 2)
                    self.center_y = int((y1 + y2) / 2)
                    self.size_x = abs(x2 - x1)
                    self.size_y = abs(y2 - y1)

                    center_msg = Int32MultiArray()
                    size_msg = Float32MultiArray()

                    center_msg.data = [self.center_x, self.center_y]
                    size_msg.data = [self.size_x, self.size_y]

                    self.center_pub.publish(center_msg)
                    self.size_pub.publish(size_msg)

                    cv2.rectangle(detected_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

                    label = f"UAV, {conf:.2f}"
                    cv2.putText(detected_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 1)
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

            return detected_image

        except Exception as e:
            rospy.logerr('Error detecting image: %s' % str(e))
            return cv_image

    def show_image(self, cv_image):
        try:
            current_time = datetime.datetime.now()
            current_time_str = current_time.strftime("%H:%M:%S.%f")[:-3]
            cv2.putText(cv_image, f"Time: {current_time_str}", (1120, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                            2)            
            screen_height = cv_image.shape[0]
            screen_width = cv_image.shape[1]

            margin_percentage_x = 0.25
            margin_percentage_y = 0.10

            x1 = int(screen_width * margin_percentage_x)
            y1 = int(screen_height * margin_percentage_y)
            x2 = screen_width - x1
            y2 = screen_height - y1

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if all((self.center_x, self.center_y, self.size_x, self.size_y)):
                cv2.line(cv_image, (int(screen_width / 2), int(screen_height / 2)),
                         (int(self.center_x), int(self.center_y)), (0, 0, 255), 2)
                cv2.putText(cv_image, f"Size X: {self.size_x:.2f} ", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255), 2)
                cv2.putText(cv_image, f"Size Y: {self.size_y:.2f} ", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 255), 2)
                cv2.putText(cv_image, f"Center X: {self.center_x} ", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)
                cv2.putText(cv_image, f"Center Y: {self.center_y} ", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)
                cv2.putText(cv_image, f"Succesful Locks: {self.successful_locks}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)

            if self.size_x is not None and self.size_y is not None:
                timer_duration = 4

                if (x1 <= self.center_x - (self.size_x/2) and self.center_x + (self.size_x/2)  <= x2 and y1 <= self.center_y - (self.size_y/2) and self.center_y + (self.size_y/2) <= y2
                    and self.size_x >= screen_width * 0.05 and self.size_y >= screen_height * 0.05):

                    elapsed_time = int(time.time() - self.start_time)

                    if timer_duration - elapsed_time < 0:
                        self.start_time = time.time()
                        self.show_time = time.time()
                        self.successful_locks += 1
                    else:
                        cv2.putText(cv_image, f"{timer_duration - elapsed_time}", (x1 - 10, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    self.start_time = time.time()
                    cv2.putText(cv_image, "Basarisiz", (x1 - 10, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                if 3 >= int(time.time() - self.show_time):
                    cv2.putText(cv_image, f"Basarili Kilitlenme", (570, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow('wamv_image', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr('Error showing image: %s' % str(e))

def main(args=None):
    rospy.init_node("yolo_detect")

    try:
        model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
    except Exception as e:
        rospy.logerr(f'Error loading YOLOv5 model: {str(e)}')
        return

    detect_image = PerceptionNode(model)

    rospy.spin()

if __name__ == '__main__':
    main()
