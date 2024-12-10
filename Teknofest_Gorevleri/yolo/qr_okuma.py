# Sena Berra Soydugan

import cv2
import requests
from pyzbar.pyzbar import decode

def qr_code_detector():

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        decoded_objects = decode(frame)

        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            # QR kodundaki veriyi kontrol edip içinde URL olup olmadığını belirle
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

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    qr_code_detector()