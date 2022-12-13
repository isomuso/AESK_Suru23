import cv2
import numpy as np
import imutils
import time

camera_matrix = np.array([[609.7764, 0, 320.29864], [0, 610.75706, 250.77745], [0.0, 0.0, 1.0]],
                         np.float32)
# from planner_test import drone_sayi
# cap=cv2.VideoCapture("xd.mp4",0)

# cap=cv2.VideoCapture("drive.mp4")
cm_hava_x = []
cm_hava_y = []
cm_kara_x = []
cm_kara_y = []


def main(hava_araci_sayisi, kara_araci_sayisi):
    global cm_hava_x
    global cm_hava_y
    global cm_kara_x
    global cm_kara_y

    # cap=cv2.VideoCapture("xd.mp4",0)
    # cap=cv2.VideoCapture(0)
    #cap = cv2.VideoCapture(0)
    frame = cv2.imread("olcayisoyasin.jpeg")

    # cap.set(3,1280)
    # cap.set(4,720)
    font = cv2.FONT_HERSHEY_COMPLEX  ##Font style for writing text on video frame
    #width = cap.get(3)
    #height = cap.get(4)
    width = frame.shape[1]
    height = frame.shape[0]

    print("SHAPEEE 1", width)
    print("SHAPEEE 2", height)
    down = 0
    k = 0
    # print(t)
    my_another_real_x = np.array([1])
    my_another_real_y = np.array([1])
    my_arr_x = np.array([2], dtype=float)
    my_arr_y = np.array([1], dtype=float)
    cm_kara_x = np.array([2], dtype=float)
    cm_kara_y = np.array([2], dtype=float)
    cm_hava_x = np.array([2], dtype=float)
    cm_hava_y = np.array([2], dtype=float)
    my_real_x = np.array([2])
    my_real_y = np.array([2])
    real_x = np.array([1])
    real_y = np.array([2])
    cm_x = np.array([1], dtype=float)
    cm_y = np.array([1], dtype=float)
    my_real_location_x = np.array([2], dtype=float)
    my_real_location_y = np.array([2], dtype=float)
    safe_dist_point_x = np.array([1])
    safe_dist_point_y = np.array([1])
    real_safe_dist_point_x = np.array([1])
    real_safe_dist_point_y = np.array([2])
    safe_cm_x = np.array([1])
    safe_cm_y = np.array([1])
    # z = input("mesafe giriniz: ")
    z = 70

    r = (1280 / width)

    z = r * (z) / 1000

    # drone_sayisi = drone_sayi
    #frame = 0
    #_, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    time.sleep(2)
    while True:

        for bn in range(10):
            # frame = cap.read()
            # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_range = np.array([155, 26, 0])
            upper_range = np.array([179, 255, 255])
            mask = cv2.inRange(hsv, lower_range, upper_range)
            kernel1 = np.ones((5, 5), np.uint8)  # Morfolojik işlem yapmak için gerekli
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)  # Morfolohik işlem uygulanıyor
            # mask=cv2.dilate(mask,kernel1,iterations=1)
            cns = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cns = imutils.grab_contours(cns)

        """
			while True:
				ret, videoGoruntu = cap.read()
				#hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
				#lower_range = np.array([155, 25, 0])
				#upper_range = np.array([179, 255, 255])
				#mask = cv2.inRange(hsv, lower_range, upper_range)
				#kernel1 = np.ones((5, 5), np.uint8)  # Morfolojik işlem yapmak için gerekli
				#mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)  # Morfolohik işlem uygulanıyor
				#cns = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
				#cns = imutils.grab_contours(cns)
				cv2.imshow("Bilgisayar Kamerasi", mask)
				if cv2.waitKey(50) & 0xFF == ord('x'):
					break
		"""


        for c in cns:
            area = cv2.contourArea(c)
            yu = len(c)

            M = cv2.moments(c)
            # cx = int(M["m10"] / M["m00"])
            # cy = int(M["m01"] / M["m00"])
            # real_x = cx - 640
            # real_y = 360 - cy
            # cm_x = real_x * z
            # cm_y = real_y * z
            # world_x = real_x / 10
            # world_y = real_y / 10
            if area > 100:

                y = 0
                k = 0
                i = 30
                j = 30
                iterasyon = 0
                # if mask[i+1][j]==255 or mask[i-1][j] or mask[i][j+1] or mask[i][j-1]:
                for i in range(int(height - 1)):
                    for j in range(int(width - 1)):
                        # print("buraya gridi4")

                        if mask[i][j] == 255:
                            if i > 10 and j > 10:
                                if (mask[i + 1][j] == 255 and mask[i + 2][j] == 255 and mask[i + 3][j] == 255 and
                                    mask[i + 4][j] == 255 and mask[i + 5][j] == 255 and mask[i + 6][j] == 255 and
                                    mask[i + 7][j] == 255 and mask[i + 8][j] == 255 and mask[i + 9][j] == 255 and
                                    mask[i + 10][j] == 255 and mask[i + 11][j] == 255 and mask[i + 12][j] == 255 and
                                    mask[i + 13][j] == 255 and mask[i + 14][j] == 255 and mask[i + 15][j] == 255 and
                                    mask[i + 16][j] == 255 and mask[i + 17][j] == 255) or (
                                        mask[i - 1][j] == 255 and mask[i - 2][j] == 255 and mask[i - 3][j] == 255 and
                                        mask[i - 4][j] == 255 and mask[i - 5][j] == 255 and mask[i - 6][j] == 255 and
                                        mask[i - 7][j] == 255 and mask[i - 8][j] == 255 and mask[i - 9][j] == 255 and
                                        mask[i - 10][j] == 255 and mask[i - 11][j] == 255 and mask[i - 12][j] == 255 and
                                        mask[i - 13][j] == 255 and mask[i - 14][j] == 255 and mask[i - 15][j] == 255 and
                                        mask[i - 16][j] == 255) or (
                                        mask[i][j + 1] == 255 and mask[i][j + 2] == 255 and mask[i][j + 3] == 255 and
                                        mask[i][j + 4] == 255 and mask[i][j + 5] == 255 and mask[i][j + 6] == 255 and
                                        mask[i][j + 7] == 255 and mask[i][j + 8] == 255 and mask[i][j + 9] == 255 and
                                        mask[i][j + 10] == 255 and mask[i][j + 11] == 255 and mask[i][j + 12] == 255 and
                                        mask[i][j + 13] == 255 and mask[i][j + 14] == 255 and mask[i][j + 15] == 255 and
                                        mask[i][j + 16] == 255) or (
                                        mask[i][j - 1] == 255 and mask[i][j - 2] == 255 and mask[i][j - 3] == 255 and
                                        mask[i][j - 4] == 255 and mask[i][j - 5] == 255 and mask[i][j - 6] == 255 and
                                        mask[i][j - 7] == 255 and mask[i][j - 8] == 255 and mask[i][j - 9] == 255 and
                                        mask[i][j - 10] == 255 and mask[i][j - 11] == 255 and mask[i][j - 12] == 255 and
                                        mask[i][j - 13] == 255 and mask[i][j - 14] == 255 and mask[i][j - 15] == 255 and
                                        mask[i][j - 16] == 255):
                                    y = y + 1
                                    if mask[i - 1][j] == 0 or mask[i - 1][j - 1] == 0 or mask[i][j - 1] == 0 or \
                                            mask[i + 1][
                                                j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or \
                                            mask[i][j + 1] == 0 or \
                                            mask[i - 1][j + 1] == 0:
                                        if k == 0:
                                            my_arr_x.resize((1, 1))
                                            my_arr_y.resize((1, 1))
                                        if k != 0:
                                            my_arr_x.resize((1, k + 1))
                                            my_arr_y.resize((1, k + 1))

                                        my_arr_x[0][k] = j
                                        my_arr_y[0][k] = i
                                        print("FIRST X VALUE: ", my_arr_x[0][0])
                                        print("FIRST Y VALUE: ", my_arr_y[0][0])
                                        print("i,j values", my_arr_x[0][0], my_arr_y[0][0])
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


                while True:

                    pas = 0
                    if k == 1:
                        sayi = sayi + 1
                        if mask[i + 1][j] == 255:
                            i = i + 1
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
                                    j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][
                                    j + 1] == 0 or \
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
                                    j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][
                                    j + 1] == 0 or \
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
                                    j - 1] == 0 or mask[i + 1][j] == 0 or mask[i + 1][j + 1] == 0 or mask[i][
                                    j + 1] == 0 or \
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
                        if k > 20:
                            if (my_arr_x[0][k - 1] + 3 == my_arr_x[0][0] and my_arr_y[0][k - 1] == my_arr_y[0][0]):
                                print("İŞLEM BAŞARILI")
                                down = 1
                                break
                            if (my_arr_x[0][k - 1] - 3 == my_arr_x[0][0] and my_arr_y[0][k - 1] == my_arr_y[0][0]):
                                print("İŞLEM BAŞARILI")
                                down = 1
                                break
                            if (my_arr_y[0][k - 1] - 3 == my_arr_y[0][0] and my_arr_x[0][k - 1] == my_arr_x[0][0]):
                                print("İŞLEM BAŞARILI")
                                down = 1
                                break
                            if (my_arr_y[0][k - 1] + 3 == my_arr_y[0][0] and my_arr_x[0][k - 1] == my_arr_x[0][0]):
                                print("İŞLEM BAŞARILI")
                                down = 1
                                break
        if down == 1:
            break



    #h = int(k / drone_sayisi)
    #print(h)
    print("K VALUE", k)
    # for hj in range(k):
    #	print("LOKASYONLAR", my_arr_x[0][hj], my_arr_y[0][hj])
    r = 0
    safe_dist = 1.5
    print("MERKEZ", cx, cy)
    real_cx = (cx - int(width / 2)) * z
    real_cy = (int(height / 2) - cy) * z
    print("CX-CY VALUE:", cx, cy)

    kara_araci_mesafesi = 50
    # safe_dist_land_vehicle = 50

    length_after_land_vehicle = k - kara_araci_mesafesi * (kara_araci_sayisi + 1)
    length_after_land_vehicle = length_after_land_vehicle / (hava_araci_sayisi - 1)

    for b in range(hava_araci_sayisi + kara_araci_sayisi):
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

        # safe_dist_point_x[0][b] = (my_arr_x[0][r] - cx) * safe_dist + my_arr_x[0][r]
        # safe_dist_point_y[0][b] = (my_arr_y[0][r] - cy) * safe_dist + my_arr_y[0][r]

        # cv2.circle(mask, (my_arr_x[0][r], my_arr_y[0][r]), 4, (0, 255, 0), 3)
        cv2.circle(mask, (320, 240), 7, (0, 255, 0), 3)
        cv2.circle(mask, (cx, cy), 7, (0, 255, 0), 3)

        # real_safe_dist_point_x[0][b] = safe_dist_point_x[0][b] - (width / 2)
        # real_safe_dist_point_y[0][b] = (height / 2) - safe_dist_point_y[0][b]

        # print("Locations x-y camera", my_real_x[0][b], my_real_y[0][b])
        # print("Most real location",float(my_real_location_x[0][b]), float(my_real_location_y[0][b]))
        # print("LOCATIONS:", my_arr_x[0][r], my_arr_y[0][r])
        # print("DIST LOCATIONS", safe_dist_point_x[0][b], safe_dist_point_y[0][b])

        # real_x[0][b] = (my_arr_x[0][r]) - (width / 2)
        # real_y[0][b] = (height / 2) - my_arr_y[0][r]
        # cm_x[0][b] = real_x[0][b] * z
        # cm_y[0][b] = real_y[0][b] * z
        # print("FARK DEĞERLERİ",real_x[0][b],real_y[0][b])

        # img_coordinates = np.array([[real_safe_dist_point_x[0][b]], [real_safe_dist_point_y[0][b]], [107]])
        # camera_matrix = np.array([[653.9445, 0, 322.2569], [0, 650.39051, 248.16122], [0.0, 0.0, 1.0]],
        #						 np.float32)  # kamera matrisi her pcye özel isoda var tmp içinde tar gz dosyası
        # inverse = np.linalg.inv(camera_matrix)
        # g = inverse * img_coordinates
        # g = g * 215  # 103 yerine yükselik neyse o konulcak
        # cm_x[0][b] = float(g[0][0])
        # cm_y[0][b] = float(g[1][1])
        # print("REAL LOCATION IN REAL WORLD:", cm_x[0][b], cm_y[0][b])
        # r = r + h

        if b < kara_araci_sayisi:
            if b == 0:
                cm_kara_x.resize((1, b + 1))
                cm_kara_y.resize((1, b + 1))

                cm_kara_x[0][b] = safe_dist_point_x[0][b] = (my_arr_x[0][r] - cx) * safe_dist + my_arr_x[0][r]
                cm_kara_y[0][b] = safe_dist_point_y[0][b] = (my_arr_y[0][r] - cy) * safe_dist + my_arr_y[0][r]
                print("LOCATION IN IMAGE", cm_kara_x[0][b], cm_kara_y[0][b])

                #print("LOCATION IN IMAGE",my_arr_x[0][r],my_arr_y[0],[r])
                cm_kara_x[0][b] = cm_kara_x[0][b] - int(width / 2)
                cm_kara_y[0][b] = int(height / 2) - cm_kara_y[0][b]

                img_coordinates = np.array([[cm_kara_x[0][b]], [cm_kara_y[0][b]], [107]])
                camera_matrix = np.array([[653.9445, 0, 322.2569], [0, 650.39051, 248.16122], [0.0, 0.0, 1.0]],
                                         np.float32)  # kamera matrisi her pcye özel isoda var tmp içinde tar gz dosyası
                inverse = np.linalg.inv(camera_matrix)
                g = inverse * img_coordinates
                g = g * 215  # 103 yerine yükselik neyse o konulcak
                cm_kara_x[0][b] = float(g[0][0])
                cm_kara_y[0][b] = float(g[1][1])

                print("when b is:", b, cm_kara_x[0][b], cm_kara_y[0][b])
                r = int(r + kara_araci_mesafesi)
            else:
                cm_kara_x.resize((1, b + 1))
                cm_kara_y.resize((1, b + 1))

                cm_kara_x[0][b] = (my_arr_x[0][r] - cx) * safe_dist + my_arr_x[0][r]
                cm_kara_y[0][b] = (my_arr_y[0][r] - cy) * safe_dist + my_arr_y[0][r]
                print("LOCATION IN IMAGE", cm_kara_x[0][b], cm_kara_y[0][b])

                cm_kara_x[0][b] = cm_kara_x[0][b] - int(width / 2)
                cm_kara_y[0][b] = int(height / 2) - cm_kara_y[0][b]

                img_coordinates = np.array([[cm_kara_x[0][b]], [cm_kara_y[0][b]], [107]])
                camera_matrix = np.array([[653.9445, 0, 322.2569], [0, 650.39051, 248.16122], [0.0, 0.0, 1.0]],
                                         np.float32)  # kamera matrisi her pcye özel isoda var tmp içinde tar gz dosyası
                inverse = np.linalg.inv(camera_matrix)
                g = inverse * img_coordinates
                g = g * 215  # 103 yerine yükselik neyse o konulcak
                cm_kara_x[0][b] = float(g[0][0])
                cm_kara_y[0][b] = float(g[1][1])

                r = int(r + kara_araci_mesafesi)
                print("when b is:", b, cm_kara_x[0][b], cm_kara_y[0][b])
        if b >= kara_araci_sayisi:
            cm_hava_x.resize((1, b - kara_araci_sayisi + 1))
            cm_hava_y.resize((1, b - kara_araci_sayisi + 1))

            cm_hava_x[0][b - kara_araci_sayisi] = (my_arr_x[0][r] - cx) * safe_dist + my_arr_x[0][r]
            cm_hava_y[0][b - kara_araci_sayisi] = (my_arr_y[0][r] - cy) * safe_dist + my_arr_y[0][r]
            print("BURDAYIZ LOCATION IN IMAGE", cm_hava_x[0][b - kara_araci_sayisi],   cm_hava_y[0][b - kara_araci_sayisi] )


            cm_hava_x[0][b - kara_araci_sayisi] = cm_hava_x[0][b - kara_araci_sayisi] - int(width / 2)
            cm_hava_y[0][b - kara_araci_sayisi] = int(height / 2) - cm_hava_y[0][b - kara_araci_sayisi]

            img_coordinates = np.array(
                [[cm_hava_x[0][b - kara_araci_sayisi]], [cm_hava_y[0][b - kara_araci_sayisi]], [107]])
            camera_matrix = np.array([[653.9445, 0, 322.2569], [0, 650.39051, 248.16122], [0.0, 0.0, 1.0]],
                                     np.float32)  # kamera matrisi her pcye özel isoda var tmp içinde tar gz dosyası
            inverse = np.linalg.inv(camera_matrix)
            g = inverse * img_coordinates
            g = g * 215  # 103 yerine yükselik neyse o konulcak
            cm_hava_x[0][b - kara_araci_sayisi] = float(g[0][0])
            cm_hava_y[0][b - kara_araci_sayisi] = float(g[1][1])
            print("when b is:", b, cm_hava_x[0][b - kara_araci_sayisi], cm_hava_y[0][b - kara_araci_sayisi])
            r = int(r + length_after_land_vehicle)

    for hj in range(k):
        # print("LOKASYONLAR",my_arr_x[0][hj],my_arr_y[0][hj])

        my_arr_x[0][hj] = my_arr_x[0][hj] - (width / 2)
        my_arr_y[0][hj] = (height / 2) - my_arr_y[0][hj]
        img_coordinates = np.array([[my_arr_x[0][hj]], [my_arr_y[0][hj]], [107]])
        camera_matrix = np.array([[653.9445, 0, 322.2569], [0, 650.39051, 248.16122], [0.0, 0.0, 1.0]],
                                 np.float32)  # kamera matrisi her pcye özel isoda var tmp içinde tar gz dosyası
        inverse = np.linalg.inv(camera_matrix)
        g = inverse * img_coordinates
        g = g * 215  # 103 yerine yükselik neyse o konulcak
        # print("G VALUE:",(g[0][0]))
        my_arr_x[0][hj] = (g[0][0])
        my_arr_y[0][hj] = (g[1][1])

    # print("OBSTACLE VALUE:",my_arr_x[0][hj],my_arr_y[0][hj])

    # print(my_arr_x[0][hj],my_arr_y[0][hj])
    #	my_arr_y[0][r]
    #	my_arr_y[0][r]

    ###### 2ND SCENARIOO ########

    # cx
    # cy
    """cv2.imshow("xde", mask)
    cv2.waitKey(100000000)"""
    # cv2.waitKey(2)
    # print("REAL VALUE X:", world_x)
    # print("REAL VALUE Y:", world_y)

    # print("CENTER"cx,cy)
    print("hava:",cm_hava_x,cm_hava_y)
    print("kara",cm_kara_x,cm_kara_y)
    return my_arr_x, my_arr_y


if __name__ == '__main__':
    main(3,2)
