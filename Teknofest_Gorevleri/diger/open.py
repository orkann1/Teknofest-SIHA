#!/usr/bin/env python3.8
# -*- coding: UTF-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
import datetime
import torch
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32MultiArray, Float32, Float32MultiArray

class PerceptionNode():
    def __init__(self, model):
        # Kamera verilerine abone ol, her fotoğrafta camera_callback fonksiyonunu çalıştır
        self.image_sub = rospy.Subscriber('/plane0/usb_cam/image_raw', Image, self.camera_callback)

        self.model = model

        # ROS-OpenCV köprüleme
        self.bridge = CvBridge()
        self.last_capture_time = None
        # Kaç saniyede bir ekrandaki frame değişecekse aşağıdan belirt.
        self.capture_interval = datetime.timedelta(seconds=0.03)
        self.show_screen = True

        # detect_image içinde class any türünden 0 1 2 alıyoruz, bunu mb_marker_seklinde yayınlamamız gerekiyor 
        self.detected_object_name = None

        # detect_image içinde hesaplanacak olan değerlerin saklanacağı özellikler
        self.center_x = None
        self.center_y = None
        self.size_x = None
        self.size_y = None

        #Pitch, Roll, Thrust ve Altitude göstergesi
        self.pitch = 0.0
        self.roll = 0.0
        self.thrust = 0.0
        self.altitude = 0.0

        # Publisher node
        self.center_pub = rospy.Publisher('/detected_center', Int32MultiArray, queue_size=20)
        self.box_size_pub = rospy.Publisher('/box_size', Float32, queue_size=10)

        # Velocity subscriber
        self.velo_sub = rospy.Subscriber('/uav0/mavros/local_position/velocity_body', TwistStamped, self.velo_callback)

        # Extra subscribers
        self.status_sub = rospy.Subscriber('/plane0/status', Float32MultiArray, self.status_callback)

    def status_callback(self, msg):
        self.pitch = msg.data[0]
        self.roll = msg.data[1]
        self.thrust = msg.data[2]
        self.altitude = msg.data[3]
        #Aracımızın Hızı eklenecek

    def velo_callback(self, vel_data):
        # Gelen TwistStamped mesajından hız bilgisini al
        self.vel_x = vel_data.twist.linear.x
        # Hız bilgisini ekrana yazdır

    # Kameradan gelen her görüntü için aşağıdaki kısım çalışacak
    def camera_callback(self, data):
        try:
            # Görüntüyü bgr8 türüne çevir
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            current_time = datetime.datetime.now()
            # Kaç saniyede bir tarama yapmak istiyorsan aşağıdan ayarla
            if (self.last_capture_time is None or (current_time - self.last_capture_time) >= self.capture_interval):
                # detect_image sayesinde yolo modeli ile fotoğraf üzerinde tarama yap ve sonuçları detect_img içinde tut
                detect_img = self.detect_image(cv_image)
                self.last_capture_time = current_time
                # Detect edilmiş fotoyu ekrana bastır
                self.show_image(detect_img)

        except Exception as e:
            rospy.logerr('Error processing image: %s' % str(e))

    def detect_image(self, cv_image):
        results = self.model([cv_image], size=640)

        detected_image = cv_image.copy()

        for pred in results.pred[0]:
            x1, y1, x2, y2, conf, cls = pred.tolist()
            x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)  # Convert to float32 for publish

            # Compute center and size of the bounding box
            self.center_x = int((x1 + x2) / 2)
            self.center_y = int((y1 + y2) / 2)
            self.size_x = abs(x2 - x1)
            self.size_y = abs(y2 - y1)

            # Publish center coordinates
            center_msg = Int32MultiArray()
            size_msg = Float32()
            
            center_msg.data = [self.center_x, self.center_y]
            size_msg.data = float(self.size_x * self.size_y)
            
            self.center_pub.publish(center_msg)
            self.box_size_pub.publish(size_msg)

            cv2.rectangle(detected_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            label = f"UAV, {conf:.2f}"
            cv2.putText(detected_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        return detected_image

    def show_image(self, cv_image):
        if self.show_screen:
            # Sunucu Saati Verisi gelecek
            current_time = datetime.datetime.now()
            current_time_str = current_time.strftime("%H:%M:%S")  # Saat, dakika ve saniyeyi metin olarak al
            cv2.putText(cv_image, f"Time: {current_time_str}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.putText(cv_image, "Pitch: {:.2f}".format(self.pitch), (1140, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(cv_image, "Roll: {:.2f}".format(self.roll), (1140, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(cv_image, "Thrust: {:.2f}".format(self.thrust), (1140, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(cv_image, "Altitude: {:.2f}".format(self.altitude), (1140, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            screen_width = cv_image.shape[1]
            screen_height = cv_image.shape[0]

            # Boşluk oranları (%5 sağ-sol, %10 üst-alt)
            margin_percentage_x = 0.25
            margin_percentage_y = 0.10

            # Boşlukların piksel değerleri hesaplanıyor
            x1 = int(screen_width * margin_percentage_x)
            y1 = int(screen_height * margin_percentage_y)
            x2 = screen_width - x1
            y2 = screen_height - y1

            # Dikdörtgeni çiz
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            
            # Velocity bilgisini ekrana yazdır
            if hasattr(self, 'vel_x'):  # Eğer vel_x değeri varsa, yani hız bilgisi alınmışsa
                cv2.putText(cv_image, "Velocity X: {:.2f}".format(self.vel_x), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # detect_image içinde hesaplanan değerleri ekrana yazdır
            if all((self.center_x, self.center_y, self.size_x, self.size_y)):
                cv2.line(cv_image, (int(screen_width/2), int(screen_height/2)), (int(self.center_x), int(self.center_y)), (0, 0, 255), 2)
                cv2.putText(cv_image, "Size x:{:.2f} ".format(self.size_x), (10,120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(cv_image, "Size y:{:.2f} ".format(self.size_y), (10,140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            if self.size_x != None:   
                if self.size_x <= screen_width*0.05 and self.size_y <= screen_height*0.05:
                    cv2.putText(cv_image, "Basarisiz", (x1-10,y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                else:
                    cv2.putText(cv_image, "Basarili", (x1-10,y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('wamv_image', cv_image)
            cv2.waitKey(1)

def main(args=None):
    rospy.init_node("yolo_detect")

    model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt', device='cpu')
    detect_image = PerceptionNode(model)

    rospy.spin()

if __name__ == '__main__':
    main()
