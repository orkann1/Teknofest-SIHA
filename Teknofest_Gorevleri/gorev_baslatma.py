#Sena Berra Soydugan

import os
import time
import subprocess
import tkinter as tk

pencere = tk.Tk()
pencere.title("UFK TAKIMI")

"""def birinci_butona_tikla():
    os.chdir("/home/orkan")
    subprocess.Popen(['gnome-terminal', '--', './sa.sh'])"""

def ikinci_butona_tikla():
    takip_kamerasi_path = "/home/orkan/Desktop/VSC/MavSDK/yolo/takip_kamerasi.py"
    takip_kodu_path = "/home/orkan/Desktop/VSC/MavSDK/takip_kodu.py"
    os.chdir("/home/orkan/Desktop/VSC/MavSDK/yolo/")
    subprocess.Popen(['gnome-terminal', '--', 'python3', takip_kamerasi_path])
    time.sleep(2)
    subprocess.Popen(['gnome-terminal', '--tab', '--', 'python3', takip_kodu_path])

def ucuncu_butona_tikla():
    takip_kodu_path = "/home/orkan/Desktop/VSC/MavSDK/takip_kodu.py"
    subprocess.Popen(['gnome-terminal', '--tab', '--', 'python3', takip_kodu_path])

def dorduncu_butona_tikla():
    qr_okuma_simulasyon_path = "/home/orkan/Desktop/VSC/MavSDK/yolo/qr_okuma_simulasyon.py"
    kamikaze_path = "/home/orkan/Desktop/VSC/MavSDK/kamikaze.py"
    os.chdir("/home/orkan/Desktop/VSC/MavSDK/yolo/")
    subprocess.Popen(['gnome-terminal', '--', 'python3', qr_okuma_simulasyon_path])
    time.sleep(2)
    subprocess.Popen(['gnome-terminal', '--tab', '--', 'python3', kamikaze_path])

def besinci_buton_tikla():
    kamikaze_path = "/home/orkan/Desktop/VSC/MavSDK/kamikaze.py"
    subprocess.Popen(['gnome-terminal', '--tab', '--', 'python3', kamikaze_path])


"""buton1= tk.Button(pencere, text="Simülasyonu Başlat", command=birinci_butona_tikla)
buton1.pack(pady=20)"""

buton2 = tk.Button(pencere, text="Takip Görevi (Görev + Kamera)", command=ikinci_butona_tikla)
buton2.pack(pady=20)

buton3 = tk.Button(pencere, text="Takip Görevi (Görev)", command=ucuncu_butona_tikla)
buton3.pack(pady=20)

buton4 = tk.Button(pencere, text="Kamikaze (Görev + Kamera)", command=dorduncu_butona_tikla)
buton4.pack(pady=20)

buton5 = tk.Button(pencere, text="Kamikaze (Görev)", command=besinci_buton_tikla)
buton5.pack(pady=20)

pencere.mainloop()
