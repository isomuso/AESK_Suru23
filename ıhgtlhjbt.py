import cv2
import numpy as np
import imutils
import time
#from planner_test import drone_sayi
#cap=cv2.VideoCapture("xd.mp4",0)
#cap=cv2.VideoCapture(0)
cap=cv2.VideoCapture(1)
#cap.set(3,1280)
#cap.set(4,720)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
font = cv2.FONT_HERSHEY_COMPLEX                ##Font style for writing text on video frame
width = cap.get(3)
height = cap.get(4)
#width = width / 2
#height = height / 2
width = 359
height = 671
print("SHAPEEE 1",width)
print("SHAPEEE 2",height)
down=0
k=0
#print(t)
my_another_real_x = np.array([1])
my_another_real_y = np.array([1])
my_arr_x = np.array([2])
my_arr_y = np.array([1])
my_real_x= np.array([2])
my_real_y= np.array([2])
real_x = np.array([1])
real_y = np.array([2])
cm_x = np.array([1])
cm_y = np.array([1])
my_real_location_x= np.array([2], dtype=float)
my_real_location_y= np.array([2], dtype=float)
safe_dist_point_x = np.array([1])
safe_dist_point_y = np.array([1])
real_safe_dist_point_x = np.array([1])
real_safe_dist_point_y = np.array([2])
safe_cm_x = np.array([1])
safe_cm_y = np.array([1])
drone_sayisi = 4
#z = input("mesafe giriniz: ")
z = 150

r=(1280/width)

z = r * (z) / 1000

#drone_sayisi = drone_sayi
frame=0
_, frame = cap.read()
left_right_image = np.split(frame, 2, axis=1)
#cv2.imshow("right", left_right_image[0])
hsv = cv2.cvtColor(left_right_image[0], cv2.COLOR_BGR2HSV)
time.sleep(2)
while True:
	cv2.imshow("right", left_right_image[0])
	print(left_right_image[0].shape)
	if cv2.waitKey(30) >= 0:
		break

while True:

	for bn in range(10):
		#frame = cap.read()
		#hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		lower_range = np.array([104, 0, 153])
		upper_range = np.array([179, 255, 255])
		mask = cv2.inRange(hsv, lower_range, upper_range)
		kernel1 = np.ones((5, 5), np.uint8)  # Morfolojik işlem yapmak için gerekli
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)  # Morfolohik işlem uygulanıyor
		#mask=cv2.dilate(mask,kernel1,iterations=1)
		cns = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		cns = imutils.grab_contours(cns)

		""""
		while True:
			ret, videoGoruntu = cap.read()
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			lower_range = np.array([155, 25, 0])
			upper_range = np.array([179, 255, 255])
			mask = cv2.inRange(hsv, lower_range, upper_range)
			kernel1 = np.ones((5, 5), np.uint8)  # Morfolojik işlem yapmak için gerekli
			mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)  # Morfolohik işlem uygulanıyor
			cns = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			cns = imutils.grab_contours(cns)
			cv2.imshow("Bilgisayar Kamerasi", mask)
			if cv2.waitKey(50) & 0xFF == ord('x'):
				break
		"""""

	print("buraya gridi1")

	for c in cns:
		area = cv2.contourArea(c)
		yu = len(c)
		print("buraya gridi2")

		M = cv2.moments(c)
		#cx = int(M["m10"] / M["m00"])
		#cy = int(M["m01"] / M["m00"])
		#real_x = cx - 640
		#real_y = 360 - cy
		#cm_x = real_x * z
		#cm_y = real_y * z
		#world_x = real_x / 10
		#world_y = real_y / 10
		if area > 500:
			print("buraya gridi3")

			y = 0
			k = 0
			i = 30
			j = 30
			iterasyon = 0
# if mask[i+1][j]==255 or mask[i-1][j] or mask[i][j+1] or mask[i][j-1]:
			for i in range(int(height-1)):
				for j in range(int(width-1)):
					#print("buraya gridi4")

					if mask[i][j] == 255:
						if i>10 and j>10:
							if (mask[i+1][j] == 255 and mask[i+2][j] == 255 and mask[i+3][j] == 255 and mask[i+4][j] == 255 and mask[i+5][j] == 255 and mask[i+6][j] == 255 and mask[i+7][j] == 255 and mask[i+8][j] == 255 and mask[i+9][j] == 255 and mask[i+10][j] == 255 and mask[i+11][j] == 255 and mask[i+12][j] == 255 and mask[i+13][j] == 255 and mask[i+14][j] == 255 and mask[i+15][j] == 255 and mask[i+16][j] == 255 and mask[i+17][j] == 255) or (mask[i-1][j] == 255 and mask[i-2][j] == 255 and mask[i-3][j] == 255 and mask[i-4][j] == 255 and mask[i-5][j] == 255 and mask[i-6][j] == 255 and mask[i-7][j] == 255 and mask[i-8][j] == 255 and mask[i-9][j] == 255 and mask[i-10][j] == 255 and mask[i-11][j] == 255 and mask[i-12][j] == 255 and mask[i-13][j] == 255 and mask[i-14][j] == 255 and mask[i-15][j] == 255 and mask[i-16][j] == 255) or (mask[i][j+1] == 255 and mask[i][j+2] == 255 and mask[i][j+3] == 255 and mask[i][j+4] == 255 and mask[i][j+5] == 255 and mask[i][j+6] == 255 and mask[i][j+7] == 255 and mask[i][j+8] == 255 and mask[i][j+9] == 255 and mask[i][j+10] == 255 and mask[i][j+11] == 255 and mask[i][j+12] == 255 and mask[i][j+13] == 255 and mask[i][j+14] == 255 and mask[i][j+15] == 255 and mask[i][j+16] == 255) or (mask[i][j-1] == 255 and mask[i][j-2] == 255 and mask[i][j-3] == 255 and mask[i][j-4] == 255 and mask[i][j-5] == 255 and mask[i][j-6] == 255 and mask[i][j-7] == 255 and mask[i][j-8] == 255 and mask[i][j-9] == 255 and mask[i][j-10] == 255 and mask[i][j-11] == 255 and mask[i][j-12] == 255 and mask[i][j-13] == 255 and mask[i][j-14] == 255 and mask[i][j-15] == 255 and mask[i][j-16] == 255):
								y = y + 1
								if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
									j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or \
										mask[i - 1][j + 1] == 0:
									if k == 0:
										my_arr_x.resize((1, 1))
										my_arr_y.resize((1, 1))
									if k != 0:
										my_arr_x.resize((1, k + 1))
										my_arr_y.resize((1, k + 1))

									my_arr_x[0][k] = j
									my_arr_y[0][k] = i
									print("FIRST X VALUE: ",my_arr_x[0][0])
									print("FIRST Y VALUE: ",my_arr_y[0][0])
									print("i,j values",my_arr_x[0][0],my_arr_y[0][0])
									k = k + 1
									if k == 1:
										break
							else:
								mask[i][j] = 0
				if k == 1:
					break

			sayi = 0
			cns = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			cns = imutils.grab_contours(cns)

			for c in cns:
				M = cv2.moments(c)
				cx = int(M["m10"] / M["m00"])
				cy = int(M["m01"] / M["m00"])
				print("IMMMMMM NICE:", cx)
				print("IMMMMMMMMM NICE ", cy)

			while True:
				#print("buraya gridi5")
				pas = 0
				if k == 1:
					sayi = sayi + 1
					if mask[i + 1][j] == 255:
						i = i + 1
						if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
							j - 1] == 0 or \
								mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or mask[i - 1][
							j + 1] == 0:
							if my_arr_x[0][k - 1] == j and my_arr_y[0][k - 1] == i:
								i = i - 1
							else:
								pas = 1
								my_arr_x.resize((1, k + 1))
								my_arr_y.resize((1, k + 1))
								my_arr_x[0][k] = j
								my_arr_y[0][k] = i
								k = k + 1
						else:
							i = i - 1
					if pas != 1:
						if mask[i - 1][j] == 255:
							i = i - 1
							if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
								j - 1] == 0 or \
									mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or \
									mask[i - 1][
										j + 1] == 0:
								if my_arr_x[0][k - 1] == j and my_arr_y[0][k - 1] == i:
									i = i - 1

								else:
									pas = 1
									my_arr_x.resize((1, k + 1))
									my_arr_y.resize((1, k + 1))
									my_arr_x[0][k] = j
									my_arr_y[0][k] = i
									k = k + 1
							else:
								i = i + 1
					if pas != 1:
						if mask[i][j + 1] == 255:
							j = j + 1
							if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
								j - 1] == 0 or \
									mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or \
									mask[i - 1][
										j + 1] == 0:
								if my_arr_x[0][k - 1] == j and my_arr_y[0][k - 1] == i:
									i = i - 1
								else:
									pas = 1
									my_arr_x.resize((1, k + 1))
									my_arr_y.resize((1, k + 1))
									my_arr_x[0][k] = j
									my_arr_y[0][k] = i
									k = k + 1
							else:
								j = j - 1
					if pas != 1:
						if mask[i][j - 1] == 255:
							j = j - 1
							if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
								j - 1] == 0 or \
									mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or \
									mask[i - 1][
										j + 1] == 0:
								if my_arr_x[0][k - 1] == j and my_arr_y[0][k - 1] == i:
									i = i - 1
								else:
									pas = 1
									my_arr_x.resize((1, k + 1))
									my_arr_y.resize((1, k + 1))
									my_arr_x[0][k] = j
									my_arr_y[0][k] = i
									k = k + 1
							else:
								j = j + 1

				pas = 0
				if k > 1:
					if mask[i + 1][j] == 255:
						i = i + 1
						if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
							j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or \
								mask[i - 1][j + 1] == 0:
							if my_arr_x[0][k - 2] == j and my_arr_y[0][k - 2] == i:
								i = i - 1
							else:
								pas = 1
								my_arr_x.resize((1, k + 1))
								my_arr_y.resize((1, k + 1))
								my_arr_x[0][k] = j
								my_arr_y[0][k] = i
								k = k + 1
						else:
							i = i - 1
					if pas != 1:
						if mask[i - 1][j] == 255:
							i = i - 1
							if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
								j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or \
									mask[i - 1][j + 1] == 0:
								if my_arr_x[0][k - 2] == j and my_arr_y[0][k - 2] == i:
									i = i + 1

								else:
									pas = 1
									my_arr_x.resize((1, k + 1))
									my_arr_y.resize((1, k + 1))
									my_arr_x[0][k] = j
									my_arr_y[0][k] = i
									k = k + 1
							else:
								i = i + 1
					if pas != 1:
						if mask[i][j + 1] == 255:
							j = j + 1
							if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
								j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or \
									mask[i - 1][j + 1] == 0:
								if my_arr_x[0][k - 2] == j and my_arr_y[0][k - 2] == i:
									j = j - 1
								else:
									pas = 1
									my_arr_x.resize((1, k + 1))
									my_arr_y.resize((1, k + 1))
									my_arr_x[0][k] = j
									my_arr_y[0][k] = i
									k = k + 1
							else:
								j = j - 1
					if pas != 1:
						if mask[i][j - 1] == 255:
							j = j - 1
							if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or mask[i + 1][
								j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][j + 1] == 0 or \
									mask[i - 1][j + 1] == 0:
								if my_arr_x[0][k - 2] == j and my_arr_y[0][k - 2] == i:
									j = j + 1
								else:
									pas = 1
									my_arr_x.resize((1, k + 1))
									my_arr_y.resize((1, k + 1))
									my_arr_x[0][k] = j
									my_arr_y[0][k] = i
									k = k + 1
							else:
								j = j + 1
					#print("K VALUE IS:",k)
					if k > 20:
						if (my_arr_x[0][k - 1] + 3 == my_arr_x[0][0] and my_arr_y[0][k - 1] == my_arr_y[0][0]):
							print("İŞLEM BAŞARILI")
							down=1
							break
						if (my_arr_x[0][k - 1] - 3 == my_arr_x[0][0] and my_arr_y[0][k - 1] == my_arr_y[0][0]):
							print("İŞLEM BAŞARILI")
							down=1
							break
						if (my_arr_y[0][k - 1] - 3 == my_arr_y[0][0] and my_arr_x[0][k - 1] == my_arr_x[0][0]):
							print("İŞLEM BAŞARILI")
							down=1
							break
						if (my_arr_y[0][k - 1] + 3 == my_arr_y[0][0] and my_arr_x[0][k - 1] == my_arr_x[0][0]):
							print("İŞLEM BAŞARILI")
							down=1
							break
	if down==1:
		break

print("buraya gridi6")

h = int(k/drone_sayisi)
print(h)
print("K VALUE",k)
for hj in range(k):
	print("LOKASYONLAR",my_arr_x[0][hj],my_arr_y[0][hj])
r=0
safe_dist = 0.5

real_cx = (cx -  int(width / 2)) * z
real_cy = (int(height / 2) - cy) * z
print("CX-CY VALUE:",cx,cy)
for b in range(drone_sayisi):

	my_real_x.resize((1, b + 1))
	my_real_y.resize((1, b + 1))
	my_real_location_x.resize((1, b + 1))
	my_real_location_y.resize((1, b + 1))
	my_another_real_x.resize((1, b + 1))
	my_another_real_y.resize((1, b + 1))
	safe_dist_point_x.resize((1, b + 1))
	safe_dist_point_y.resize((1, b + 1))
	real_safe_dist_point_x.resize((1, b + 1))
	real_safe_dist_point_y.resize((1, b + 1))
	safe_cm_x.resize((1, b + 1))
	safe_cm_y.resize((1, b + 1))
	real_x.resize((1, b + 1))
	real_y.resize((1, b + 1))
	cm_x.resize((1, b + 1))
	cm_y.resize((1, b + 1))

	safe_dist_point_x[0][b] = (my_arr_x[0][r] - cx) * safe_dist + my_arr_x[0][r]
	safe_dist_point_y[0][b] = (my_arr_y[0][r] - cy) * safe_dist + my_arr_y[0][r]


	my_another_real_x[0][b] =  my_arr_x[0][r] - int(width / 2)
	my_another_real_y[0][b] = int(height / 2) - my_arr_y[0][r]

	my_real_x[0][b] = my_arr_x[0][r] - 960
	my_real_y[0][b] = 540 -  my_arr_y[0][r]

	real_safe_dist_point_x[0][b] = safe_dist_point_x[0][b] - (width / 2)
	real_safe_dist_point_y[0][b] = (height / 2) - safe_dist_point_y[0][b]
	safe_cm_x[0][b] = real_safe_dist_point_x[0][b] * z #mesafeli gerçek dünya
	safe_cm_y[0][b] = real_safe_dist_point_y[0][b] * z #mesafeli gerçek dünya

	#print("Locations x-y camera", my_real_x[0][b], my_real_y[0][b])

	my_real_location_x[0][b] = (my_real_x[0][b] - 40)/60
	my_real_location_y[0][b] =( my_real_y[0][b] )/60
	#print("Most real location",float(my_real_location_x[0][b]), float(my_real_location_y[0][b]))
	print("LOCATIONS:",my_arr_x[0][r], my_arr_y[0][r])
	print("DIST LOCATIONS",safe_dist_point_x[0][b], safe_dist_point_y[0][b])

	real_x[0][b] = (my_arr_x[0][r]) - (width / 2)
	real_y[0][b] = (height / 2) - my_arr_y[0][r]
	cm_x[0][b] = real_x[0][b] * z
	cm_y[0][b] = real_y[0][b] * z
	print("FARK DEĞERLERİ",real_x[0][b],real_y[0][b])
	print("REAL LOCATION IN REAL WORLD:",cm_x[0][b],cm_y[0][b])
	cv2.circle(mask, (my_arr_x[0][r], my_arr_y[0][r]), 3, (255, 255, 255), 3)

	r = r + h

#	my_arr_y[0][r]


###### 2ND SCENARIOO ########


#cx
	#cy
cv2.imshow("xde",mask)
cv2.waitKey(100000000)
#print("REAL VALUE X:", world_x)
#print("REAL VALUE Y:", world_y)

#print("CENTER"cx,cy)