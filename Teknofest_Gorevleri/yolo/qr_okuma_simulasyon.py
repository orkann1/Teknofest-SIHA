# Sena Berra Soydugan

import cv2
import requests
from pyzbar.pyzbar import decode
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class QRCodeDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/plane0/usb_cam/image_raw', Image, self.camera_callback)
        rospy.init_node('qr_code_detector', anonymous=True)

    def camera_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        decoded_objects = decode(frame)

        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            if qr_data.startswith('http://') or qr_data.startswith('https://'):
                print("URL found:", qr_data)
                try:
                    # URL içeriğini al
                    response = requests.get(qr_data)
                    # Alınan içeriği yazdır
                    print("Content:", response.text)
                except Exception as e:
                    print("Error fetching content:", e)
            else:
                print("QR Code:", qr_data)
            print("Position:", obj.rect)

            cv2.rectangle(frame, obj.rect, (0, 255, 0), 2)
            cv2.putText(frame, qr_data, (obj.rect[0], obj.rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow('QR Code Detector', frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        QRCodeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()