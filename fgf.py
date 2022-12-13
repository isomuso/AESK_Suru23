import cv2
import numpy as np


# from time import sleep


#   #   #   #   #   #   #   #   #   #   #
#  Tübitak İHA Yarışması 1.görev uçuşu  #
#   #   #   #   #   #   #   #   #   #   #


# Odak uzaklığını hesaplayacak fonksiyon
# _________________________________________________________________
def FocalLength(measured_distance, real_width, width_in_rf_image):
	'''
	1.parametre = Referans fotoğrafındaki nesne ile kamera arasındaki mesafe
	2.parametre = Nesnenin gerçek hayattaki genişliği ( rf fotosunda )
	3.parametre = Referans fotoğrafta nesnenin kadrajdaki genişliği
	'''
	focal_length = (width_in_rf_image * measured_distance) / real_width
	return focal_length


# _________________________________________________________________


# Uzaklık hesaplayacak olan fonksiyon
# ____________________________________________________________
def Distance_finder(Focal_Length, real_width, width_in_frame):
	'''
	1.parametre = FocalLength fonksiyonunun çıktısı
	2.parametre = Gerçek hayattaki genişlik
	3.parametre = görüntüdeki genişlik
	'''
	distance = (real_width * Focal_Length) / width_in_frame
	return distance


# ____________________________________________________________


# Trackbar oluştururken lazım oluyor
# ______________
def nothing(x):
	pass


# ______________


# Görmek istediğimiz ekran seçimi / Maske kontrol edilmek istenirse açılır
# ___________________
def show(main, mask):
	cv2.imshow("MASK", mask)
	cv2.imshow("MAIN", main)


# ___________________

# Trackbarlar oluşturuluyor
cv2.namedWindow("HSV CONTROL")

cv2.createTrackbar("L - H", "HSV CONTROL", 0, 180, nothing)
cv2.createTrackbar("L - S", "HSV CONTROL", 0, 255, nothing)
cv2.createTrackbar("L - V", "HSV CONTROL", 0, 255, nothing)
cv2.createTrackbar("H - H", "HSV CONTROL", 180, 180, nothing)
cv2.createTrackbar("H - S", "HSV CONTROL", 255, 255, nothing)
cv2.createTrackbar("H - V", "HSV CONTROL", 255, 255, nothing)

cap = cv2.VideoCapture(0)

while True:  # sonsuz döngü ile frame alınacak

	# Maske oluşturulabilmesi için değişkenlere atanıyor
	l_h = cv2.getTrackbarPos("L - H", "HSV CONTROL")
	l_s = cv2.getTrackbarPos("L - S", "HSV CONTROL")
	l_v = cv2.getTrackbarPos("L - V", "HSV CONTROL")
	h_h = cv2.getTrackbarPos("H - H", "HSV CONTROL")
	h_s = cv2.getTrackbarPos("H - S", "HSV CONTROL")
	h_v = cv2.getTrackbarPos("H - V", "HSV CONTROL")

	_, frame = cap.read()

	#frame = cv2.resize(frame, (640, 480))
	cv2.circle(frame, (320, 240), 7, (0, 255, 0), 3)

	#frame = cv2.flip(frame, 1)

	HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # RGB renk uzayını HSV renk uzayına çevirir

	# MASKE
	#############################################
	low_color = np.array([l_h, l_s, l_v], np.uint8)
	high_color = np.array([h_h, h_s, h_v], np.uint8)
	mask_color = cv2.inRange(HSV, low_color, high_color)  # belirlenen renk aralığı için maske oluşturuluyor
	kernel = np.ones((3, 3), np.uint8)  # morph için gerekli
	mask_color = cv2.morphologyEx(mask_color, cv2.MORPH_OPEN,
	                              kernel)  # NOT : farklı kombinasyonlarla daha net görüntü elde edilebilir.

	contours, _ = cv2.findContours(mask_color, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

	for cnt in contours:

		(x, y, w, h) = cv2.boundingRect(cnt)

		object_width_in_frame = w  # karşılaştıralacak olan veri * * * referans bilgisi

		if object_width_in_frame != 0:  # Eğer ekranda istenen nitelikte bir nesne varsa döngüye gir

			cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Dörtgeni oluşturur

		show(frame, mask_color)

		if cv2.waitKey(13) & 0xFF == ord("q"):
			break

		break  # bu break olmadan da patlıyor
	show(frame, mask_color)

	if cv2.waitKey(13) & 0xFF == ord("q"):
		break

cap.release()  # kamerayı serbest bırak
cv2.destroyAllWindows()  # pencereleri kapat