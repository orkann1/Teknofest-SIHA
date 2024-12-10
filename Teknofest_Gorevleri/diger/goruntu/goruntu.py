import cv2
import numpy as np

def resize_image(image, width=640, height=480):
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

# Görüntüyü yükleyin
image_path = '/home/orkan/Desktop/VSC/MavSDK/goruntu/bir.png'
image = cv2.imread(image_path)

# Görüntünün başarılı bir şekilde yüklenip yüklenmediğini kontrol edin
if image is None:
    print(f"Görüntü yüklenemedi: {image_path}")
else:
    print(f"Görüntü başariyla yüklendi: {image_path}")

    # Görüntüyü yeniden boyutlandırma
    resized_image = resize_image(image)

    # Görüntüyü gri tonlamaya çevirin
    gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

    # Kenar algılama
    edges = cv2.Canny(gray_image, 50, 150)

    # Kenarları genişletmek için dilate işlemi
    kernel = np.ones((5, 5), np.uint8)
    dilated_edges = cv2.dilate(edges, kernel, iterations=1)

    # Konturları bulma
    contours, _ = cv2.findContours(dilated_edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # En büyük iki konturu seçme
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

    # Kanat uç noktalarını belirleme
    kanat_noktalari = []
    for contour in contours:
        leftmost = tuple(contour[contour[:, :, 0].argmin()][0])
        rightmost = tuple(contour[contour[:, :, 0].argmax()][0])
        topmost = tuple(contour[contour[:, :, 1].argmin()][0])
        bottommost = tuple(contour[contour[:, :, 1].argmax()][0])
        kanat_noktalari.extend([leftmost, rightmost, topmost, bottommost])

    # Kanat uç noktalarını filtreleme (yalnızca x koordinatlarına göre)
    kanat_uç_noktalari = sorted(kanat_noktalari, key=lambda x: x[0])
    kanat_uç_noktasi_1 = kanat_uç_noktalari[0]
    kanat_uç_noktasi_2 = kanat_uç_noktalari[-1]

    # İHA'nın merkezini belirleme
    iha_merkezi = ((kanat_uç_noktasi_1[0] + kanat_uç_noktasi_2[0]) // 2, (kanat_uç_noktasi_1[1] + kanat_uç_noktasi_2[1]) // 2)

    def hesapla_aci(nokta1, nokta2):
        delta_y = nokta2[1] - nokta1[1]
        delta_x = nokta2[0] - nokta1[0]
        aci = np.arctan2(delta_y, delta_x) * (180 / np.pi)
        return aci

    # İHA'nın merkezine göre kanatların açısını hesaplayın
    aci_kanat_1 = hesapla_aci(iha_merkezi, kanat_uç_noktasi_1)
    aci_kanat_2 = hesapla_aci(iha_merkezi, kanat_uç_noktasi_2)

    print(f"Kanat 1 açisi: {aci_kanat_1} derece")
    print(f"Kanat 2 açisi: {aci_kanat_2} derece")

    # Kanat uç noktalarını ve merkezi işaretleyin
    cv2.circle(resized_image, kanat_uç_noktasi_1, 5, (0, 255, 0), -1)
    cv2.circle(resized_image, kanat_uç_noktasi_2, 5, (0, 255, 0), -1)
    cv2.circle(resized_image, iha_merkezi, 5, (0, 0, 255), -1)

    # Görüntüyü göster
    cv2.imshow('İHA Açisi', resized_image)
    cv2.waitKey(1)
    cv2.destroyAllWindows()
