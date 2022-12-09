import numpy as np
import rospy
import time
import random
import hungarian
import forimage as a
from multiprocessing.dummy import Process
from geometry_msgs.msg import TransformStamped
import tf
import bn_sim as bn
import math
import tf2_ros
import tools

import commander as cmd


class Plan:
    def __init__(
            self,
            threatlist=[],
            ka=1.7,
            kr_d=4,
            safe_dist_d=0.4,
            kv=1.5,
            dt=1 / 10,
            d_h=0.7,
            safe_dist_o=0.8,
            kr_o=0.1,
            drone_list=[]
    ):
        # planı oluşturacak fonksiyonları tutan liste [func1, func2, ...]
        self.plan_list = []
        # droneların birbirinden kaçması için katsayılar
        self.kr_d = kr_d
        self.safe_dist_d = safe_dist_d
        # droneların engellerden kaçması için katsayılar
        self.kv = kv

        # mainloop periyodu
        self.dt = dt
        # preceed fonksiyonu için indis
        self.i = 0
        # kontrol algoritmaları için deadband
        self.deadband = 0.1
        # engelden ne kadar uzaklıkta dronv_r_d = drone.add_repulsive_potential(
        self.flag1 = True
        self.flag2 = False
        self.dp = list
        self.tl = list
        #
        self.kd = 4
        self.threatlist = threatlist
        self.kr_o = kr_o
        self.safe_dist_d = safe_dist_d
        self.safe_dist_o = safe_dist_o
        self.yasin = 0
        self.furkan = 0
        self.minimum = 0


        self.removed_drone_number = 0
        # -----------------------------------------------------------------

        # droneların yerden irtifası
        self.d_h = d_h
        self.formation_radius = 2
        self.hover_list = []
        self.land_formation_asc_first_run = True

        # parametreleri tutan dict
        self.param = dict()

        # crazyflie parametreler
        # move
        self.param['ka_cf'] = 20
        # take_off
        self.param['ka_takeoff_cf'] = 20
        self.param['deadband_takeoff_cf'] = 0.10
        # lamd
        self.param['ka_land_cf'] = 20
        self.param['deadband_land_cf'] = 0.05
        # form_formation
        self.param['ka_form_formation_cf'] = 40
        self.param['deadband_formation_cf'] = 0.1
        # protect position
        self.param[f'ka_hover_cf'] = 10  # 12
        # rotate
        self.param[f'ka_rotate_formation_cf'] = 20

        self.param['ka_obs_cf'] = 3

        self.param['kr_d_cf'] = kr_d
        self.param['kr_o_cf'] = kr_o
        self.param['safe_dist_d_cf'] = safe_dist_d
        self.param['safe_dist_o_cf'] = safe_dist_o
        self.param['kv_cf'] = kv
        self.param['deadband_move_cf'] = 0.1
        self.param['radius_formation_cf'] = 1

        # firefly parametreler

        # takeoff
        self.param['ka_takeoff_ff'] = 15
        self.param['deadband_takeoff_ff'] = 0.1
        # land
        self.param['ka_land_ff'] = 4
        self.param['deadband_land_ff'] = 0.1
        # form_formation
        self.param['ka_form_formation_ff'] = 30
        # protect position
        self.param[f'ka_hover_ff'] = 5
        # rotate
        self.param[f'ka_rotate_formation_ff'] = 20

        self.param['ka_obs_ff'] = 2.5

        self.param['ka_ff'] = 30  # çekici potansiyel k.s
        self.param['kr_d_ff'] = 4  # itici potansiyel k.s
        self.param['kr_o_ff'] = kr_o
        self.param['safe_dist_d_ff'] = 4  # dronelar için safe dist
        self.param['safe_dist_o_ff'] = 0.5  # obsler için safe dist
        self.param['kv_ff'] = 1.5  # virtual repulsive katsayısı
        self.param['deadband_formation_ff'] = 0.1  # formasyon görevi için deadband
        self.param['deadband_move_ff'] = 0.8
        self.param['radius_formation_ff'] = 3

        self.T = 0
        self.time = 0
        self.target_ind = 0
        self.target_speed = 0
        self.state = 0
        self.states = 0
        self.target_course = 0
        self.lastIndex = 0

        self.x_ort = 0
        self.y_ort = 0
        self.yaricap = 0
        self.threat = []

        self.drawing_path = True
        self.proto_first_run = True  # ilerletme ilk defa
        self.first_form_formation = True  # formasyon görevi ilk defa
        self.fire_move_astar_firstrun = True
        self.first_take_off = True
        self.waypoint = []
        self.waypoint_kara = []
        self.fire_points_x = []
        self.fire_points_y = []

        # deneysel, yeni classa taşınabilir
        self.virtual_lead_pos = [0, 0, 1]
        self.take_off_sync_first_run = True
        self.formation = True
        self.formation_points = []  # formasyonu ifade eden noktaları içeren liste
        self.drone_list = drone_list
        self.threatlist = threatlist
        self.fp_list = []
        self.new_formation = True


        self.global_fp_points = []
        self.start_heading = 0
        self.rotate_virtual_lead_first_run = True
        self.virtual_lead_heading= 0
        self.desired_point = []
        self.land11 = True
        self.desired_angle = 0


        self.zumo_list = []

        #object initlemeler
        self.commander = cmd.Commander(self)
        self.first_time_rotate_vl = True
        self.first_rotate = True

        self.desired_ccw_rotation = 0
        

    def hungarian(self, formation_points, return_list=False):
        """
        formation_points: [[x,y,z], ......] find_formation_pointsten aldığı noktaları atar
        çıktı olarak drone_list içerisindeki assigned_fpleri günceller.

        return_list true yapıldığı zaman, formasyon noktalarını içeren liste, dronelistteki droneların sırasına göre
        sıralanıyor. eğer drone listesinin sırası değişirse bir sebepten ötürü istenmeyen sonuçlar olabilir.

        return: [[x, y,z ], [x, y, z], ...]
        """

        cost_matrix = []
        global_fp_list = []

        for drone in self.drone_list:
            for fp in formation_points:
                drone_pos = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
                f_pos = np.array(fp)
                dist_vec = drone_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(cost_matrix, (len(self.drone_list), len(formation_points)))
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())  # Get the element position.
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for drone in self.drone_list:
            index = self.drone_list.index(drone)
            ans_mat1 = ans_mat[index, :]
            ans_mat1 = ans_mat1.tolist()
            max_value = max(ans_mat1)
            index = ans_mat1.index(max_value)

            if return_list:
                global_fp_list.append(formation_points[index])

            else:
                drone.assigned_fp = formation_points[index]

        if return_list:
            return global_fp_list
    
    def find_formation_points(self, fp_list, virtual_lead_pos=[], assignment=False):
        """
        girdi olarak orta noktayı ve orta noktaya göre olan formasyon noktalarını alıp
        çıktı olarak mutlak 0'a göre olan formasyon noktalarını döner

        - fplist = [[x, y, z], [x, y, z], .... ] formasyon noktalarını vl'ye göre içeren liste
        formasyon noktalarını plannerlistte bir değişkende kaydet.
        - vl_pos = [x, y, z] formasyonun oluşturulması istenen nokta
        - assigment true olursa drone.fp'leri güncelliyor, olmazsa direk liste dönüyor
        macar için kullanılabilir.
        - virtual_lead_pos'e bir değer atanırsa o noktada formasonu oluşturur atanmazsa en son
        vl nerede kaldıysa orada oluşturur

        !!!! Yeni formasyon oluşturmak için kullanılacak olursa self.new_formation attributesi True
        yapılmalı
        """

        if self.new_formation:
            self.fp_list = fp_list
            self.new_formation = False

        if len(virtual_lead_pos) != 0:
            self.virtual_lead_pos = virtual_lead_pos

        # print('vl konumu', self.virtual_lead_pos)
        # transformları tutacak buffer
        buffer_core = tf2_ros.BufferCore(
            rospy.Duration(10.0)
        )  # 10 saniye cache süresi galiba

        # mapten orta noktaya transformları buffera koy
        ts1 = self.transform_from_euler(
            self.virtual_lead_pos[0],
            self.virtual_lead_pos[1],
            self.virtual_lead_pos[2],
            0,
            0,
            self.virtual_lead_heading,
            "map",
            "frame1",
        )
        buffer_core.set_transform(ts1, "default_authority")

        # formasyon tanımlamayı buna benzer bir yöntem ile basitleştirebilirsin.

        i = 2
        formation = []
        for fp in self.fp_list:
            # mapten orta noktaya transform
            ts = self.transform_from_euler(
                fp[0],
                fp[1],
                fp[2],
                0,
                0,
                0,
                "frame1",
                f"frame{i}",
            )
            # orta noktadan formasyon noktasına olan transformları buffera koy
            buffer_core.set_transform(ts, "default_authority")

            # bufferdan işlenmiş transformu çek
            fp_transform = buffer_core.lookup_transform_core(
                "map", f"frame{i}", rospy.Time(0))

            fp_global = [
                fp_transform.transform.translation.x,
                fp_transform.transform.translation.y,
                fp_transform.transform.translation.z,
            ]
            # buradan rotation.z çerekerek bütün İHA'ların baş açısını çekip gönderebiliriz

            # çekilen formasyon noktasını listeye kaydet.
            formation.append(fp_global)
            i += 1  # buna gerek olmayabilir

        if assignment:
            i = 0
            for drone in self.drone_list:
                drone.assigned_fp = formation[i]
                i += 1

        else:
            return formation
    
    def move_virtual_lead(self, rate):
        """
        açıklama: orta noktayı ilerletmek vasıtasıyla tüm formasyonu ilerletir.

        desired_point = [x, y, z]
        rate = float: hızı veriyor

        """

        # şartı kontrol et
        dist = np.array(self.virtual_lead_pos) - np.array(self.desired_point)
        mag = np.linalg.norm(dist)

        if mag >= self.deadband:
            #virtual lead pos update
            vel = np.array(self.desired_point) - np.array(self.virtual_lead_pos)
            vel = (vel / np.linalg.norm(vel)) * rate
            self.virtual_lead_pos = np.array(self.virtual_lead_pos)
            self.virtual_lead_pos = self.virtual_lead_pos + vel * self.dt

            # fp atadı
            self.find_formation_points(self.fp_list, self.virtual_lead_pos, True)
            return False
        else:
            return True
    
    def rotate_virtual_lead(self, ang_r):
        """
        açıklama: orta noktayı ilerletmek vasıtasıyla tüm formasyonu ilerletir.

        desired_point = [[x, y, z], ...]
        rate = float: hızı veriyor
        """
        #virtual lead pos güncelle
        if abs(self.virtual_lead_heading - self.start_heading) > self.desired_angle:
            return True
        else:
            #virtual lead oryantason güncelle
            self.virtual_lead_heading += ang_r * self.dt  # derece
            print("vl heading", self.virtual_lead_heading)

            #fp noktalarını atadı
            self.find_formation_points(self.fp_list, self.virtual_lead_pos, True)
            return False
    
    
    def move(self, rate=0.3):
        
        #initializer ekle

        # katsayılara ayar çek
        cond = self.move_virtual_lead(rate)

        if cond:
            return cond
        
        else:

            # görevi uygula
            for drone in self.drone_list:
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'

                try:
                    v_a = drone.add_attractive_potential(drone.assigned_fp, 30)
                    v = np.array(v_a)
                    if np.linalg.norm(v) > 100:
                        v = (v / np.linalg.norm(v)) * 100
                    v.tolist()
                    drone.velocity_control(v, drone.assigned_fp[2])

                except ZeroDivisionError:
                    pass

            return cond
    

    def rotate(self, rate=0.3):

        # katsayılara ayar çek
        cond = self.rotate_virtual_lead(rate)

        if cond:
            return cond
        
        else:

            # görevi uygula
            for drone in self.drone_list:
               
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'

                try:
                    v_a = drone.add_attractive_potential(drone.assigned_fp, 30)
                    v = np.array(v_a)
                    if np.linalg.norm(v) > 100:
                        v = (v / np.linalg.norm(v)) * 100
                    v.tolist()
                    drone.velocity_control(v, drone.assigned_fp[2])

                except ZeroDivisionError:
                    pass

            return cond
    
    def take_off_test1(self, desired_height=1,
                       **kwargs):
        """
        formasyonu ilerletmek için kullanılır

        ka: hızlıca formasyon oluşturmak için normalden büyük olmalı
        kr_d: küçük formasyonlarda dronelar birbirini it mesin diye küçük olmalı,
        macar çalışıyor zaten
        ka, safe_dist_d, kr_d, safe_dist_o, kr_0, kv, lock,
        """

        if self.first_take_off:
            self.d_h = desired_height

            # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
            self.param.update(kwargs)
          
            # yere dizildikleri konumları formasyon olarak algılama, kontrolünü sağlama
            self.commander.initialize_formation()
            self.desired_point = [self.virtual_lead_pos[0], self.virtual_lead_pos[1], self.d_h]
            
            # flagi kapat
            self.first_take_off = False

        return self.move()
    
    def march_test1(self, desired_point,
                       **kwargs):
        """
        formasyonu ilerletmek için kullanılır

        ka: hızlıca formasyon oluşturmak için normalden büyük olmalı
        kr_d: küçük formasyonlarda dronelar birbirini it mesin diye küçük olmalı,
        macar çalışıyor zaten
        ka, safe_dist_d, kr_d, safe_dist_o, kr_0, kv, lock,
        """

        if self.first_take_off:

            # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
            self.param.update(kwargs)
            
            # yere dizildikleri konumları formasyon olarak algılama, kontrolünü sağlama
            self.commander.initialize_formation()

            # yere dizildikleri konumları formasyon olarak algılama, kontrolünü sağlama
            self.desired_point = desired_point
            
            # flagi kapat
            self.first_take_off = False

        result = self.move()
        
        if result:
            self.first_take_off = True
            return result
        else:
            return result
    

    def march_test2(self, desired_heading,
                       **kwargs):
        """
        formasyonu ilerletmek için kullanılır

        ka: hızlıca formasyon oluşturmak için normaldmarchv, lock,
        """

        if self.first_rotate:

            # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
            self.param.update(kwargs)
            
            # yere dizildikleri konumları formasyon olarak algılama, kontrolünü sağlama
            self.commander.initialize_formation()

            self.desired_angle = desired_heading
            self.start_heading = self.virtual_lead_heading

            # yere dizildikleri konumları formasyon olarak algılama, kontrolünü sağlama
            
            # flagi kapat
            self.first_rotate = False
            print("bp2")
          

        result = self.rotate(3)
        
        if result:
            self.first_rotate = True
            return result
        else:
            return result
    
     
    def track_timeout(self, duration, **kwargs):

        self.param.update(kwargs)

        if self.new_function:
            self.start_time = time.time()
            self.new_function = False

        if time.time() < self.start_time + duration:
            return False

        else:
            self.new_function = True
            return True

    # formasyon optimizesi için yeni bir class gerekebilr

    def transform_from_euler(
            self, x, y, z, roll, pitch, yaw, header_frame_id, child_frame_id
    ):
        """
        Creates a new stamped transform from translation and euler-orientation
        :param x: x translation
        :param y: y translation
        :param z: z translation
        :param roll: orientation roll
        :param pitch: orientation pitch
        :param yaw: orientation yaw
        :param header_frame_id: transform from this frame
        :param child_frame_id: transform to this frame
        :returns: transform
        """
        t = TransformStamped()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = tf.transformations.quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.header.frame_id = header_frame_id
        t.child_frame_id = child_frame_id

        return t

    def land_formation_asc_test(self):
        # yeni formasyon sitemine uygun landing
        # iniş için listeyi düzenle 
        if self.land_formation_asc_first_run:
            self.hover_list = self.drone_list.copy()
            self.bubble_sort(self.hover_list)
            self.land_formation_asc_first_run = False

            # bunları r'ye bağlı yap.
            landing_points = [[-1, 1, 0], [1, 0, 0], [1, 1, 0], [1, 0, 0], [0, 0, 0], [-1, 0, 0], [-1, -1, 0],
                              [0, -1, 0], [1, -1, 0]]
            landing_points_global = self.find_formation_points(landing_points)
            self.hungarian(landing_points_global)

        for drone in self.hover_list:
            # uçuş noktalarını belirle
            if drone.first_run_form_land:
                # hover için nokta belirle (cf)
                drone.hover_pos = [drone.current_position_x, drone.current_position_y, drone.current_position_z]
                # inen için nokta belirle
                drone.first_run_form_land = False
            else:
                pass

        # haraketi uygula
        try:

            while len(self.hover_list) > 0:
                cond = False  # iniş tamamlanmamışken
                while not cond:
                    print(len(self.hover_list))
                    cond = self.hover_list[0].land_single(self.param['ka_land_cf'])  # sırası gelen dronu indir

                    # kendine denk geldiysen diğer iterasyona geç
                    for drone in self.hover_list:
                        if self.hover_list[0] == drone:
                            continue

                        # sırası olmayanlara hover gönder
                        if drone.link_uri == 0:
                            drone.position_control(drone.hover_pos[0], drone.hover_pos[1], drone.hover_pos[2], 0)

                        else:
                            drone.send_pose(drone.hover_pos[0], drone.hover_pos[1], drone.hover_pos[2])

                # alttaki satırı silersen zıp zıp formasyon oluyo xd -isomuso -leventi bıçaklayan olcay
                self.hover_list.remove(self.hover_list[0])


        except ZeroDivisionError:
            pass

            # haraketi kontrol et
            b = 1
            if len(self.hover_list) == 1:
                return True
            else:  # b == 0
                return False

    def take_off_sync(self, drone_list, rate):
        # print(self.virtual_lead_pos)

        # İlk defa çalıştığında
        average_vector = np.array([0, 0])
        if self.take_off_sync_first_run:  # kalkışı initialize et

            # ihaların orta noktasını hesapla

            for drone in drone_list:
                drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y])
                average_vector = average_vector + drone_pos_vec
            average_vector = average_vector / len(drone_list)
            self.virtual_lead_pos = average_vector.tolist()
            self.virtual_lead_pos.append(0)

            self.take_off_sync_first_run = False

        else:
            # take_off için böyle
            self.virtual_lead_pos[2] += rate * self.dt  # tüm dtler self.dt'ye atanacak ve bu 1/rate olacak

        # orta noktadan faydalanarak, mevcut dizilimlerini formasyon olarak tanımla

        current_formation_points = []  # vl framine göre formasyon noktası vektörleri
        for drone in drone_list:
            drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y])
            formation_vector = drone_pos_vec - average_vector
            formation_vector = formation_vector.tolist()
            current_formation_points.append(formation_vector)

        # transformları tutacak buffer
        buffer_core = tf2_ros.BufferCore(
            rospy.Duration(10.0)
        )  # 10 saniye cache süresi galiba

        # mapten orta noktaya transformları buffera koy
        ts1 = self.transform_from_euler(
            self.virtual_lead_pos[0],
            self.virtual_lead_pos[1],
            self.virtual_lead_pos[2],
            0,
            0,
            0,
            "map",
            "frame1",
        )
        buffer_core.set_transform(ts1, "default_authority")

        # formasyon tanımlamayı buna benzer bir yöntem ile basitleştirebilirsin.

        i = 2
        formation = []
        for fp in current_formation_points:
            # mapten orta noktaya transform
            ts = self.transform_from_euler(
                fp[0],
                fp[1],
                0,
                0,
                0,
                0,
                "frame1",
                f"frame{i}",
            )
            # orta noktadan formasyon noktasına olan transformları buffera koy
            buffer_core.set_transform(ts, "default_authority")

            # bufferdan işlenmiş transformu çek
            fp_transform = buffer_core.lookup_transform_core(
                "map", f"frame{i}", rospy.Time(0))

            fp_global = [
                fp_transform.transform.translation.x,
                fp_transform.transform.translation.y,
                fp_transform.transform.translation.z,
            ]
            # buradan rotation.z çerekerek bütün İHA'ların baş açısını çekip gönderebiliriz

            # çekilen formasyon noktasını listeye kaydet.
            formation.append(fp_global)
            i += 1  # buna gerek olmayabilir

        # formasyon noktaları, iha listesindeki sıraya göre
        # oluşturulduğu için direk veriyoruz normalde bu atama
        # macar ile yapılmalı. mantıken macarı bir for eksik çağırman lazım

        # haraketi uygulatma kısmı
        b = 1

        print(formation[0])
        # conditiona self.virtual_lead_pos[2] < self.dh gibi bir şey atacaksın.
        for drone, fp in zip(drone_list, formation):
            drone.assigned_fp = fp

            try:
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'

                cond = abs(drone.current_position_z - self.d_h) <= self.param[f'deadband_takeoff_{type}']
                if cond:
                    if drone.link_uri == 0:
                        drone.position_control(
                            fp[0], fp[1], fp[2], 0)

                    else:
                        drone.send_pose(fp[0], fp[1], fp[2])

                    a = 1
                else:

                    v_a = drone.add_attractive_potential_z(fp[2], self.param[f'ka_takeoff_{type}'])

                    drone.velocity_control_z(
                        fp[0], fp[1], v_a
                    )
                    a = 0
                b = b * a


            except ZeroDivisionError:
                pass

        if b == 1:
            return True
        else:  # b == 0
            return False

    def calculate_path2(self, desired_point):

        vl_pos = self.calculate_start_point(self.threatlist, desired_point,
                                            step_size=0.05)  # formasyon dağıtmak için noktayı döner
        print('vl_pos', vl_pos)

        seperation_point = self.give_formation_points(self.fp_list,
                                                      vl_pos)  # dağıtma noktası için formasyon noktalarını bul

        seperation_list = self.hungarian(seperation_point, return_list=True)

        sx = []
        sy = []
        for point in seperation_list:
            sx.append(point[0])
            sy.append(point[1])

        desired_point.append(self.d_h)
        destination_point = self.give_formation_points(self.fp_list, desired_point
                                                       )  # toplanma noktaları için
        destination_list = self.hungarian(destination_point, return_list=True)
        print('destination_list', destination_list)

        # MACARA LİSTE DÖNME EKLE

        # print(wplist)
        vl_pos = vl_pos[0:3]
        astar = Process(target=self.draw_path_for_drones, args=([destination_list, sx, sy]))
        move = Process(target=self.yusuf, args=([vl_pos]))
        # self.draw_path_for_drones(destination_list, sx, sy)

        # move = Process(target=self.move_formation_test1, args=([vl_pos]))
        astar.start()
        move.start()

        # self.take_off_test(1)
        # self.move_formation_test1(vl_pos)

        astar.join()
        move.join()
        return True


    def yusuf(self, vl_pos):
        cond = False
        while not cond:
            cond = self.take_off_formation(self.drone_list)
        cond = False
        while not cond:
            cond = self.form_formation([-1, 0 ,0.6], self.drone_list, 'triangle', r=0.5)
        self.wait(2, self.drone_list)

        # self.form_formation_test_for3(self.fp_list)
        # self.move_formation_test1(vl_pos)
        # self.wait(15, self.drone_list)

  

    def give_formation_points(self, fp_list, virtual_lead_pos):
        """
        girdi olarak orta noktayı ve orta noktaya göre olan formasyon noktalarını alıp
        çıktı olarak mutlak 0'a göre olan formasyon noktalarını döner

        - fplist = [[x, y, z], [x, y, z], .... ] formasyon noktalarını vl'ye göre içeren liste
        formasyon noktalarını plannerlistte bir değişkende kaydet.
        - vl_pos = [x, y, z] formasyonun oluşturulması istenen nokta
        - assigment true olursa drone.fp'leri güncelliyor, olmazsa direk liste dönüyor
        macar için kullanılabilir.
        - virtual_lead_pos'e bir değer atanırsa o noktada formasonu oluşturur atanmazsa en son
        vl nerede kaldıysa orada oluşturur

        !!!! Yeni formasyon oluşturmak için kullanılacak olursa self.new_formation attributesi True
        yapılmalı
        """

        if self.new_formation:
            self.fp_list = fp_list
            self.new_formation = False

        # print('vl konumu', virtual_lead_pos)
        # transformları tutacak buffer
        buffer_core = tf2_ros.BufferCore(
            rospy.Duration(10.0)
        )  # 10 saniye cache süresi galiba

        # mapten orta noktaya transformları buffera koy
        ts1 = self.transform_from_euler(
            virtual_lead_pos[0],
            virtual_lead_pos[1],
            virtual_lead_pos[2],
            0,
            0,
            0,
            "map",
            "frame1",
        )
        buffer_core.set_transform(ts1, "default_authority")

        # formasyon tanımlamayı buna benzer bir yöntem ile basitleştirebilirsin.

        i = 2
        formation = []
        for fp in self.fp_list:
            # mapten orta noktaya transform
            ts = self.transform_from_euler(
                fp[0],
                fp[1],
                fp[2],
                0,
                0,
                0,
                "frame1",
                f"frame{i}",
            )
            # orta noktadan formasyon noktasına olan transformları buffera koy
            buffer_core.set_transform(ts, "default_authority")

            # bufferdan işlenmiş transformu çek
            fp_transform = buffer_core.lookup_transform_core(
                "map", f"frame{i}", rospy.Time(0))

            fp_global = [
                fp_transform.transform.translation.x,
                fp_transform.transform.translation.y,
                fp_transform.transform.translation.z,
            ]
            # buradan rotation.z çerekerek bütün İHA'ların baş açısını çekip gönderebiliriz

            # çekilen formasyon noktasını listeye kaydet.
            formation.append(fp_global)
            i += 1  # buna gerek olmayabilir

        return formation

    def hungarian(self, formation_points, return_list=False):
        """
        formation_points: [[x,y,z], ......] find_formation_pointsten aldığı noktaları atar
        çıktı olarak drone_list içerisindeki assigned_fpleri günceller.

        return_list true yapıldığı zaman, formasyon noktalarını içeren liste, dronelistteki droneların sırasına göre
        sıralanıyor. eğer drone listesinin sırası değişirse bir sebepten ötürü istenmeyen sonuçlar olabilir.

        return: [[x, y,z ], [x, y, z], ...]
        """

        cost_matrix = []
        global_fp_list = []

        for drone in self.drone_list:
            for fp in formation_points:
                drone_pos = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
                f_pos = np.array(fp)
                dist_vec = drone_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(cost_matrix, (len(self.drone_list), len(formation_points)))
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())  # Get the element position.
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for drone in self.drone_list:
            index = self.drone_list.index(drone)
            ans_mat1 = ans_mat[index, :]
            ans_mat1 = ans_mat1.tolist()
            max_value = max(ans_mat1)
            index = ans_mat1.index(max_value)

            if return_list:
                global_fp_list.append(formation_points[index])

            else:
                drone.assigned_fp = formation_points[index]

        if return_list:
            return global_fp_list

    def hungarian2(self, formation_points, drone_list, return_list=False):
        """
        formation_points: [[x,y,z], ......] find_formation_pointsten aldığı noktaları atar
        çıktı olarak drone_list içerisindeki assigned_fpleri günceller.

        return_list true yapıldığı zaman, formasyon noktalarını içeren liste, dronelistteki droneların sırasına göre
        sıralanıyor. eğer drone listesinin sırası değişirse bir sebepten ötürü istenmeyen sonuçlar olabilir.

        return: [[x, y,z ], [x, y, z], ...]
        """

        cost_matrix = []
        global_fp_list = []

        for drone in drone_list:
            for fp in formation_points:
                drone_pos = np.array([drone.current_position_x, drone.current_position_y])
                f_pos = np.array(fp)
                dist_vec = drone_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(cost_matrix, (len(drone_list), len(formation_points)))
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())  # Get the element position.
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for drone in drone_list:
            index = drone_list.index(drone)
            ans_mat1 = ans_mat[index, :]
            ans_mat1 = ans_mat1.tolist()
            max_value = max(ans_mat1)
            index = ans_mat1.index(max_value)

            if return_list:
                global_fp_list.append(formation_points[index])

            else:
                drone.assigned_fp = formation_points[index]

        if return_list:
            return global_fp_list

   
    def land1(self,
                       **kwargs):
        """
        formasyonu ilerletmek için kullanılır

        ka: hızlıca formasyon oluşturmak için normalden büyük olmalı
        kr_d: küçük formasyonlarda dronelar birbirini it mesin diye küçük olmalı,
        macar çalışıyor zaten
        ka, safe_dist_d, kr_d, safe_dist_o, kr_0, kv, lock,
        """

        if self.land11:


            average_vector = np.array([0, 0])
            current_formation_points = []  # vl framine göre formasyon noktası vektörleri
            desired_point = []

            # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
            self.param.update(kwargs)
            # ihaların orta noktasını hesapla

            for drone in self.drone_list:
                drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y])
                average_vector = average_vector + drone_pos_vec
            average_vector = average_vector / len(self.drone_list)

            # self.virtual_lead_pos.append(0)

            # yere dizildikleri konumları formasyon olarak algılama, kontrolünü sağlama
            for drone in self.drone_list:
                drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y])
                formation_vector = drone_pos_vec - average_vector
                formation_vector = formation_vector.tolist()
                formation_vector.append(self.virtual_lead_pos[2])
                current_formation_points.append(formation_vector)
            self.fp_list = current_formation_points

            self.virtual_lead_pos[0:2] = average_vector.tolist()

            self.desired_point = [average_vector[0], average_vector[1], 0]

            self.land11 = False

        cond = self.move_virtual_lead(self.desired_point, 0.3)
        # print(cond)
        # görevi uygula
        for drone in self.drone_list:
            type = str()
            if drone.link_uri == 0:  # if firefly
                type = 'ff'
            else:  # if crazyflie
                type = 'cf'

            try:
                v_a = drone.add_attractive_potential(drone.assigned_fp, 20)
                v = np.array(v_a)
                if np.linalg.norm(v) > 100:
                    v = (v / np.linalg.norm(v)) * 100
                v.tolist()
                drone.velocity_control(v, drone.assigned_fp[2])

            except ZeroDivisionError:
                pass
            if cond:
                """try:
                    drone.command.send_stop_setpoint()
                except:
                    pass"""
                print('indim')

        return cond

   

    def take_off_test(self, desired_height=1,
                      **kwargs):
        """
        formasyonu ilerletmek için kullanılır

        ka: hızlıca formasyon oluşturmak için normalden büyük olmalı
        kr_d: küçük formasyonlarda dronelar birbirini it mesin diye küçük olmalı,
        macar çalışıyor zaten
        ka, safe_dist_d, kr_d, safe_dist_o, kr_0, kv, lock,
        """

        self.d_h = desired_height

        average_vector = np.array([0, 0])
        current_formation_points = []  # vl framine göre formasyon noktası vektörleri
        desired_point = []

        if self.first_take_off_test:
            # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
            self.param.update(kwargs)
            # ihaların orta noktasını hesapla

            for drone in self.drone_list:
                drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y])
                average_vector = average_vector + drone_pos_vec
            average_vector = average_vector / len(self.drone_list)

            # self.virtual_lead_pos.append(0)

            self.first_take_off_test = False

            # yere dizildikleri konumları formasyon olarak algılama, kontrolünü sağlama
            for drone in self.drone_list:
                drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y])
                formation_vector = drone_pos_vec - average_vector
                formation_vector = formation_vector.tolist()
                formation_vector.append(self.virtual_lead_pos[2])
                current_formation_points.append(formation_vector)
            self.fp_list = current_formation_points

            self.virtual_lead_pos[0:2] = average_vector.tolist()

            desired_point = [average_vector[0], average_vector[1], self.d_h]

        cond = False
        while not cond:
            cond = self.move_virtual_lead(desired_point, 0.3)
            print('kalkış')

            # print(cond)
            # görevi uygula
            for drone in self.drone_list:
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'

                try:
                    v_a = drone.add_attractive_potential(drone.assigned_fp, 30)
                    v = np.array(v_a)
                    if np.linalg.norm(v) > 100:
                        v = (v / np.linalg.norm(v)) * 100
                    v.tolist()
                    drone.velocity_control(v, drone.assigned_fp[2])

                except ZeroDivisionError:
                    pass

        self.param.update(kwargs)
        time_start = time.time()
        # time.sleep()

        for drone in self.drone_list:
            drone.hover_pos = [drone.current_position_x, drone.current_position_y, drone.current_position_z]

        duration = 15
        rate = rospy.Rate(10)
        while time.time() < time_start + duration:
            print('bekle')
            for drone in self.drone_list:
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'
                v = drone.add_attractive_potential(drone.hover_pos, self.param[f'ka_hover_{type}'])
                drone.velocity_control(v, drone.hover_pos[2])
            rate.sleep()

        return True

    def move_formation_test1(self, desired_point=[10, 10, 1],
                             **kwargs):
        """
        formasyonu ilerletmek için kullanılır

        ka: hızlıca formasyon oluşturmak için normalden büyük olmalı
        kr_d: küçük formasyonlarda dronelar birbirini it mesin diye küçük olmalı,
        macar çalışıyor zaten
        ka, safe_dist_d, kr_d, safe_dist_o, kr_0, kv, lock,
        """
        print("formasyon oluştur")

        if self.first_form_formation:
            # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
            self.param.update(kwargs)
            self.first_form_formation = False

        cond = False
        while not cond:
            cond = self.move_virtual_lead(desired_point, 0.1)
            # print(cond)
            # görevi uygula
            for drone in self.drone_list:
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'

                try:
                    v_a = drone.add_attractive_potential(drone.assigned_fp, 20)
                    v = np.array(v_a)
                    if np.linalg.norm(v) > 100:
                        v = (v / np.linalg.norm(v)) * 100
                    v.tolist()
                    drone.velocity_control(v, drone.assigned_fp[2])

                except ZeroDivisionError:
                    pass

    def move_formation_test(self, rate, desired_point=[10, 10, 1],
                            **kwargs):
        """
        formasyonu ilerletmek için kullanılır

        ka: hızlıca formasyon oluşturmak için normalden büyük olmalı
        kr_d: küçük formasyonlarda dronelar birbirini it mesin diye küçük olmalı,
        macar çalışıyor zaten
        ka, safe_dist_d, kr_d, safe_dist_o, kr_0, kv, lock,
        """
        print("formasyon oluştur")

        if self.first_form_formation:
            # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
            self.param.update(kwargs)
            self.first_form_formation = False

        cond = self.move_virtual_lead(desired_point, rate)
        # görevi uygula

        for drone in self.drone_list:
            type = str()
            if drone.link_uri == 0:  # if firefly
                type = 'ff'
            else:  # if crazyflie
                type = 'cf'

            try:
                v_a = drone.add_attractive_potential(drone.assigned_fp, self.param[f'ka_form_formation_{type}'])
                v = np.array(v_a)
                if np.linalg.norm(v) > 100:
                    v = (v / np.linalg.norm(v)) * 100
                v.tolist()
                drone.velocity_control(v, drone.assigned_fp[2])

            except ZeroDivisionError:
                pass

        if cond:
            return True
        else:  # b == 0
            return False


    def move_drone(self, drone, desired_point):
        "desired_point: liste: [x,y]"
        v = drone.constant_speed(desired_point, 0.4)
        drone.velocity_control(v, 10)

        b = np.array([drone.current_position_x, drone.current_position_y]) - np.array(desired_point)
        b = np.linalg.norm(b)
        if b < 0.1:
            return True
        else:  # b == 0
            return False


    def update_list(self):
        i = 0
        while i < len(self.drone_list):
            drone = self.drone_list[i]
            if drone.link_uri != 0:
                if drone.current_rpm == 0:
                    self.drone_list.remove(drone)
                    continue

            else:
                i+=1

    def form_formation_test(self, fp_list, virtual_lead_pos=[],
                            **kwargs):
        """
        ############
        desired_point girilirse girilen noktada formasyonu oluşturur. Desired point default değeri olan boş liste
        olarak kalırsa, İHA'ların konumlarının ortlamasını alır ve o noktada formasyon oluşturur.
        formasyon değişimi için çağırılır

        virtyal_lead_pos = [x, y, z]
        """

        self.update_list()

        print("formasyon oluştur")

        average_vector = np.array([0, 0, 0])

        if self.first_form_formation:
            if len(virtual_lead_pos) > 0:
                self.global_fp_points = self.find_formation_points(self.fp_list, virtual_lead_pos)
            else:
                # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
                self.param.update(kwargs)
                self.fp_list = fp_list
                # ihaların orta noktasını hesapla
                for drone in self.drone_list:
                    drone_pos_vec = np.array(
                        [drone.current_position_x, drone.current_position_y, drone.current_position_z])
                    average_vector = average_vector + drone_pos_vec
                average_vector = average_vector / len(self.drone_list)
                self.virtual_lead_pos = average_vector
                self.global_fp_points = self.find_formation_points(self.fp_list, average_vector)

                # self.virtual_lead_pos.append(0)
                self.hungarian(self.global_fp_points)

            self.first_form_formation = False

        # görevi uygula
        for drone in self.drone_list:
            type = str()
            if drone.link_uri == 0:  # if firefly
                type = 'ff'
            else:  # if crazyflie
                type = 'cf'

            try:
                #self.hungarian(self.global_fp_points)
                v_a = drone.add_attractive_potential(drone.assigned_fp, self.param[f'ka_form_formation_{type}'])
                v = np.array(v_a)
                if np.linalg.norm(v) > 100:
                    v = (v / np.linalg.norm(v)) * 100
                v.tolist()
                drone.velocity_control(v, drone.assigned_fp[2])

            except ZeroDivisionError:
                pass

        # görev şartlarını kontrol et.

        b = 1
        for drone in self.drone_list:
            type = str()
            if drone.link_uri == 0:  # if firefly
                type = 'ff'
            else:  # if crazyflie
                type = 'cf'

            a = drone._is_arrived(deadband=self.param[f'deadband_formation_{type}'])  # 0 veya 1 dönüyor
            b = b * a
        if b == 1:
            return True
        else:  # b == 0
            return False

    def form_formation_test_for3(self, fp_list, virtual_lead_pos=[],
                                 **kwargs):
        """
        desired_point girilirse girilen noktada formasyonu oluşturur. Desired point default değeri olan boş liste
        olarak kalırsa, İHA'ların konumlarının ortlamasını alır ve o noktada formasyon oluşturur.
        formasyon değişimi için çağırılır

        virtyal_lead_pos = [x, y, z]
        """
        print("formasyon oluştur")

        average_vector = np.array([0, 0, 0])

        if len(virtual_lead_pos) > 0:
            self.global_fp_points = self.find_formation_points(self.fp_list, virtual_lead_pos)
        else:
            # ilk defa çalıştırmada yapılan ayarlar (argüman paslama, parametre)
            self.param.update(kwargs)
            self.fp_list = fp_list
            # ihaların orta noktasını hesapla
            for drone in self.drone_list:
                drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
                average_vector = average_vector + drone_pos_vec
            average_vector = average_vector / len(self.drone_list)
            self.global_fp_points = self.find_formation_points(self.fp_list, average_vector)

        self.first_form_formation = False

        cond = False
        while cond:
            # görevi uygula
            for drone in self.drone_list:
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'

                try:
                    self.hungarian(self.global_fp_points)
                    v_a = drone.add_attractive_potential(drone.assigned_fp, self.param[f'ka_form_formation_{type}'])
                    v = np.array(v_a)
                    if np.linalg.norm(v) > 100:
                        v = (v / np.linalg.norm(v)) * 100
                    v.tolist()
                    drone.velocity_control(v, drone.assigned_fp[2])

                except ZeroDivisionError:
                    pass

            # görev şartlarını kontrol et.

            b = 1
            for drone in self.drone_list:
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'

                a = drone._is_arrived(deadband=self.param[f'deadband_formation_{type}'])  # 0 veya 1 dönüyor
                b = b * a
            if b == 1:
                cond = True
            else:  # b == 0
                cond = False

    def calculate_start_point(self, threatlist, desired_point, step_size=0.1):
        # init poseu parametrize et.
        vl_init_pos = np.array([0, 0])

        direction_vector = np.array(desired_point) - np.array(vl_init_pos)
        vel_vector = direction_vector / np.linalg.norm(direction_vector)
        vel_vector = vel_vector * step_size  # bunu kontrol et

        formation_radius = 0.5
        safe_distance = 0.2

        for i in range(int(np.linalg.norm(direction_vector) / step_size)):
            vl_init_pos = vl_init_pos + vel_vector
            for threat in threatlist:
                threat_pos = np.array([threat.current_position_x, threat.current_position_y])
                distance = np.linalg.norm(vl_init_pos - threat_pos)
                if distance <= formation_radius + threat.radius + safe_distance:
                    break
            else:  # break gelmezse buraya giriyomuş
                continue
            break  # break gelirse buraya giriyo

        vl_init_pos = vl_init_pos.tolist()
        vl_init_pos.append(self.d_h)
        return vl_init_pos

    def draw_path_for_drones(self, wplist=[], sx=[], sy=[]):
        """
        bu fonksiyon 3. görev için kullanılırsa (default olarak öyle) a* hedef noktaları
        formasyon noktaları oluyor.
        bu fonksiyon 5.görev için kullanılırsa hedef noktaların wplist olark gitmesi lazım, ayrıca
        ona göre thraetlist lazım
        [[x, y], [x, y], ...]
        """

        self.drawing_path = True
        process_list = [None for x in range(len(self.drone_list))]
        print(process_list)
        i = 0

        if len(wplist) == 0:
            for drone in self.drone_list:
                process_list[i] = Process(target=drone.draw_path,
                                          args=([drone.assigned_fp[0], drone.assigned_fp[1], self.threatlist]))
                process_list[i].start()
                i += 1
        else:
            for drone, wp in zip(self.drone_list, wplist):
                if len(sx) == 0 and len(sy) == 0:
                    process_list[i] = Process(target=drone.draw_path,
                                              args=([wp[0], wp[1], self.threatlist, 0.05, 0.09]))
                    process_list[i].start()
                else:
                    process_list[i] = Process(target=drone.draw_path,
                                              args=([wp[0], wp[1], self.threatlist, 0.05, 0.09, sx[i], sy[i]]))
                    process_list[i].start()

                i += 1

        for i in range(len(self.drone_list)):
            process_list[i].join()

        self.drawing_path = False
        print(f'{self.drawing_path}')

    def draw_path_for_drones_kara(self, wplist=[], wp_list2=[]):
        """
        bu fonksiyon 3. görev için kullanılırsa (default olarak öyle) a* hedef noktaları
        formasyon noktaları oluyor.
        bu fonksiyon 5.görev için kullanılırsa hedef noktaların wplist olark gitmesi lazım, ayrıca
        ona göre thraetlist lazım
        [[x, y], [x, y], ...]
        """

        self.drawing_path = True
        process_list = [None for x in range(len(self.zumo_list) + len(self.drone_list))]
        i = 0

        print('processlist', process_list)
        print(self.drone_list)
        print(self.zumo_list)
        print(wplist)
        print(wp_list2)

        for drone,  wp, in zip(self.drone_list, wplist):
            process_list[i] = Process(target=drone.draw_path,
                                      args=([wp[0], wp[1], self.threatlist, 0.05, 0.09]))
            process_list[i].start()
            i += 1

        for zumo, wp2 in zip(self.zumo_list, wp_list2):
            process_list[i] = Process(target=zumo.draw_path,
                                          args=([wp2[0], wp2[1], self.threatlist, 0.05, 0.09]))
            process_list[i].start()

            i += 1

        for i in range(len(self.drone_list) + len(self.zumo_list)):
            process_list[i].join()

        self.drawing_path = False

    def wait_for_astar(self, drone_list, **kwargs):
        """
        python sleep fonksiyonunun işe yarayacağını düşünmüyorum çünkü
        crazyflielara sürekli veri gitmeli gibi
        """
        self.param.update(kwargs)
        print('bekle')
        for drone in drone_list:
            drone.hover_pos = [drone.current_position_x, drone.current_position_y, 1]  # drone.current_position_z]

        rate = rospy.Rate(10)
        while self.drawing_path:
            for drone in drone_list:
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'
                v = drone.add_attractive_potential(drone.hover_pos, self.param[f'ka_hover_{type}'])
                drone.velocity_control(v, drone.hover_pos[2])
            rate.sleep()

        return True

    def calculate_path(self, wplist=[]):

        print(wplist)
        astar = Process(target=self.draw_path_for_drones, args=([wplist, [], []]))
        hover = Process(target=self.wait_for_astar, args=([self.drone_list]))
        astar.start()
        hover.start()

        astar.join()
        hover.join()
        return True

    def calculate_path3(self, wp_list1, wplist2):
        "lütfen bana sövmeyin bu fonksiyon için -minibüsçü osman"
        astar = Process(target=self.draw_path_for_drones_kara, args=([wp_list1, wplist2]))
        hover = Process(target=self.wait_for_astar, args=([self.drone_list]))
        astar.start()
        hover.start()

        astar.join()
        hover.join()
        return True


    def fire_setpoints(self, dronelist, zumolist):

        # görüntüyü işle -yasin xd
        self.fire_points_x, self.fire_points_y = a.main(len(dronelist), len(zumolist))
        self.fire_points_x = self.fire_points_x[0]
        self.fire_points_y = self.fire_points_y[0]



        # iki araç tipi içinde noktaları al

        my_real_location_x = a.cm_hava_x
        my_real_location_y = a.cm_hava_y

        print('hava:', my_real_location_x, my_real_location_y )


        kara_x = a.cm_kara_x
        kara_y = a.cm_kara_y

        print('kara:', kara_x, kara_y)

        #zumo için noktaları işle
        for zumo, i in zip(zumolist, range(len(zumolist))):

            # hedef noktaları ata
            x = kara_x[0][i]
            y = kara_y[0][i]
            self.waypoint_kara.append([x/100, y/100])

            #yangın noktaları ata
            zumo.fire_point_x = self.fire_points_x / 100
            zumo.fire_point_y = self.fire_points_y / 100

        #drone için noktaları işle
        for drone, i in zip(dronelist, range(len(dronelist))):

            #hedef noktaları ata
            self.yasin = my_real_location_x[0][i]
            self.furkan = my_real_location_y[0][i]
            self.waypoint.append([self.yasin / 100, self.furkan / 100])

            #yangın noktaları ata
            drone.fire_point_x = self.fire_points_x / 100
            drone.fire_point_y = self.fire_points_y / 100

        print('waypoint', self.waypoint)
        print('waypoint', self.waypoint_kara)

        return True

    def fire_move_astar(self, drone_list, threatlist=[]):

        # YANGIN NOKTALARININ ATAMASINI YAP
        if self.fire_move_astar_firstrun:
            wp_list = self.waypoint
            astar_wp = []

            cost_matrix = []
            for drone2 in drone_list:
                for fp in wp_list:
                    # dronların noktalara olan mesafelerimi bir arraye koy
                    drone_pos = np.array([drone2.current_position_x, drone2.current_position_y])
                    f_pos = np.array(fp)
                    dist_vec = drone_pos - f_pos
                    dist_mag = np.linalg.norm(dist_vec)
                    cost_matrix.append(dist_mag)

            cost_matrix = np.reshape(cost_matrix, (len(drone_list), len(wp_list)))
            ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())  # Get the element position.
            ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                     ans_pos)  # Get the minimum or maximum value and corresponding matrix.

            for drone in drone_list:
                index = drone_list.index(drone)

                ans_mat1 = ans_mat[index, :]
                ans_mat1 = ans_mat1.tolist()
                max_value = max(ans_mat1)
                index = ans_mat1.index(max_value)

                # ns sıralı mı
                # formasyon noktası assign için bir şey yaz buraya (macar)
                wp = wp_list[index]
                astar_wp.append(wp)

            self.calculate_path(astar_wp)
            self.fire_move_astar_firstrun = False

        a = self.offline(drone_list)

        print('NİYE TRUE DÖNMÜYOR')
        return a

    def fire_move_astar_kara(self):

        # YANGIN NOKTALARININ ATAMASINI YAP
        if self.fire_move_astar_firstrun:

            astar_wp_hava = self.hungarian2(self.waypoint, self.drone_list, return_list=True)
            astar_wp_kara = self.hungarian2(self.waypoint_kara, self.zumo_list, return_list=True)

            print('atanmış kara', astar_wp_kara)
            print('atanmış hava', astar_wp_hava)

            self.calculate_path3(astar_wp_hava, astar_wp_kara)
            self.fire_move_astar_firstrun = False



        robot_list = self.drone_list + self.zumo_list
        a = self.offline(robot_list)

        return a

    def fire_move_potential(self, drone_list):

        mal_furki = 0

        wp_list = self.waypoint
        print(self.waypoint)

        for drone in drone_list:
            cost_matrix = []
            for drone2 in drone_list:
                for fp in wp_list:
                    drone_pos = np.array([drone2.current_position_x, drone2.current_position_y])
                    f_pos = np.array(fp)
                    dist_vec = drone_pos - f_pos
                    dist_mag = np.linalg.norm(dist_vec)
                    cost_matrix.append(dist_mag)
                    mal_furki = mal_furki + 1

            cost_matrix = np.reshape(cost_matrix, (len(drone_list), len(wp_list)))
            ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())  # Get the element position.
            ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                     ans_pos)  # Get the minimum or maximum value and corresponding matrix.

            index = drone_list.index(drone)
            ans_mat = ans_mat[index, :]
            ans_mat = ans_mat.tolist()
            max_value = max(ans_mat)
            index = ans_mat.index(max_value)

            # ns sıralı mı
            # formasyon noktası assign için bir şey yaz buraya (macar)
            wp = wp_list[index]

            # v = drone.add_attractive_potential(wp_list[drone.ns-1], 3)
            v = drone.add_attractive_potential(wp, 3)
            drone.velocity_control(v, self.d_h)

    def offline(self, drone_list):
        """]
        waypoint_list: [[x,y], [x,y], .....]
        """

        a = 1

        for drone in drone_list:
            # yeniden yol çizme şartlarını kontrol et

            if drone.path_i <= len(drone.path) - 1:
                # print(drone.path)
                # print(drone.path_i)

                # dron ara noktaya vardıysa indexi arttır
                if drone.is_arrived(drone.path[drone.path_i]):
                    drone.path_i += 1

                # v_a = drone.add_attractive_potential(drone.path[drone.path_i], ka=15)
                drone.update_dronelist(drone_list)
                drone_threat_list = drone.update_threatlist(0.2)

                v_a = drone.constant_speed(drone.path[drone.path_i], 4)
                v_r_o = drone.add_repulsive_potential(drone_threat_list, 1, 1)

                # dinamik engel kısmı
                drone.update_dronelist(drone_list)

                v = np.array(v_a) + np.array(v_r_o)  # + np.array(v_r) + np.array(v_o)
                v.tolist()

                drone.velocity_control(v, drone.d_h)
                a = a * 0



            else:
                a = a * 1

        # tüm droneların varıp varmadığını kontrol et
        if a == 0:
            return False

        else:  # a ==1
            return True

            # diğer droneların gelmesini bekle
            # dronun noktaya vardığına dair bir şey yap
        # tüm dronelar vardıysa
        # True

        # tüm dronelar dönmediyse
        # False

    def offline2(self, drone_list):
        """]
        waypoint_list: [[x,y], [x,y], .....]
        """

        a = 1

        for drone in drone_list:
            # yeniden yol çizme şartlarını kontrol et

            if drone.path_i <= len(drone.path) - 1:
                # print(drone.path)
                # print(drone.path_i)

                # dron ara noktaya vardıysa indexi arttır
                if drone.is_arrived(drone.path[drone.path_i]):
                    drone.path_i += 1

                # v_a = drone.add_attractive_potential(drone.path[drone.path_i], ka=15)
                drone.update_dronelist(drone_list)
                drone_threat_list = drone.update_threatlist(0.2)

                v_a = drone.constant_speed(drone.path[drone.path_i], 4)
                # v_r_o = drone.add_repulsive_potential(drone_threat_list, 2, 4)

                # dinamik engel kısmı
                drone.update_dronelist(drone_list)

                v = np.array(v_a)  # + np.array(v_r_o)  # + np.array(v_r) + np.array(v_o)
                v.tolist()

                drone.velocity_control(v, self.d_h)
                a = a * 0



            else:
                a = a * 1

        # tüm droneların varıp varmadığını kontrol et
        if a == 0:
            return False

        else:  # a ==1
            return True

            # diğer droneların gelmesini bekle
            # dronun noktaya vardığına dair bir şey yap
        # tüm dronelar vardıysa
        # True

        # tüm dronelar dönmediyse
        # False

    def schedule_gain_2(self, current_rate, max_g, min_g, max_rate, min_rate, transient_band):
        """
        max ve min arasında çapraz geçiş
        """

        mid_point = (max_rate + min_rate) / 2

        if current_rate < mid_point - transient_band:
            return min_g

        elif current_rate > mid_point + transient_band:
            return max_g

        else:
            g = ((max_g - min_g) / (transient_band * 2)) * (current_rate - min_rate) + min_g
            return g

    def schedule_gain_1(self, current_rate, max_g, min_g, max_rate, min_rate):
        """
        max ve min arasında sadece iki tane değer ve arada smooth bir geçiş içeren bir mod
        """

        g = ((min_g - max_g) / (min_rate - max_rate)) * (current_rate - min_rate) + min_g
        return g

    def move_formation_proto(
            self, rate, desired_point, drone_list, threatlist=[], **kwargs
    ):
        self.param.update(kwargs)

        """
        #değişitirlmesilazım

        bu fonksiyon 3 tane fonksiyona dönüşebiliyomuş gibi düşün.
        yani tek başına bir fonksiyon değil ama yeri gelince formasyonu ilerletme,
        yeri gelince geçici varış noktaları belirleme (bunu yaparken droneları durdurabilir
        ama işlemin çok hızlı olması lazım zaten), yeri gelince de geçici noktalara
        ilerleme fonksiyonu oluyor
        """

        # bunları class attributesi yap.

        # bu üç fonksiyon optimizasyon için class altında
        # tek başına tanımlanabilir

        def sm1():
            print("formasyon korunuyor.")
            # attitudei uygula
            a = self.move_formation(
                rate, desired_point, drone_list, threatlist=[], draw_traj=False
            )

            # flag kontrol et (3'ünden 1'ini)

            # bu kısmı ya aşağıdaki şekilde yaparız ya da optimizasyon açısından
            # her bir drone sadece kendine yakın olan engelleri bir listeye append
            # eder her bir drone, her bir engelinkini hesaplamaz
            for drone in drone_list:
                self.tl = drone.f_check_obs(threatlist, 1.0, drone_list)  # f_check_obs < f_temp_dp
            if len(self.tl) >= 1:
                print("engelle karşilaşildi")
                for drone in drone_list:
                    drone.formation_radius = self.formation_radius
                    drone.f_temp_dp(desired_point, self.tl, 3, drone_list, threatlist)
                self.flag1 = False
                self.flag2 = True

            return a

        def sm2():
            print("formasyon bozuldu")

            # attitudei uygula
            # flag kontrol et (3'ünden 1'ini)

            for drone in drone_list:
                print(drone.assigned_fp)
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'

                # dronun yolu yoksa veya gerekiyosa yol çiz,
                if self.proto_first_run:
                    self.calculate_path(drone_list, threatlist)
                    self.proto_first_run = False

                if drone.path_i <= len(drone.path) - 1:
                    print('ismail ve olcayın harikalar diyarı')

                    # dron ara noktaya vardıysa indexi arttır
                    if drone.is_arrived(drone.path[drone.path_i]):
                        drone.path_i += 1

                    # v_a = drone.add_attractive_potential(drone.path[drone.path_i], ka=15)
                    drone.update_dronelist(drone_list)
                    drone_threat_list = drone.update_threatlist(0.2)

                    v_a = drone.constant_speed(drone.path[drone.path_i], 10)
                    v_r_d = drone.add_repulsive_potential(
                        drone_threat_list, self.param[f'safe_dist_d_{type}'], self.param[f'kr_d_{type}']
                    )
                    v_r_o = drone.add_virtual_repulsive_potential(
                        threatlist, self.param[f'safe_dist_o_{type}'], self.param[f'kr_o_{type}'],
                        self.param[f'kv_{type}'], False
                    )
                    v = np.array(v_a) + np.array(v_r_d) + np.array(v_r_o)
                    if np.linalg.norm(v) > 40:
                        v = (v / np.linalg.norm(v)) * 40
                    v.tolist()
                    drone.velocity_control(v, drone.assigned_fp[2])

                else:
                    drone.position_control(drone.assigned_fp[0], drone.assigned_fp[1], self.d_h, 0)

            b = 1
            for drone in drone_list:
                type = str()
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'
                a = drone._is_arrived(self.param[f'deadband_move_{type}'])  # 0 veya 1 dönüyor
                b = b * a
            if b == 1:
                self.flag1 = True
                self.flag2 = False
                self.tl = []

        if self.flag1:
            return sm1
        elif self.flag2:
            return sm2


    def find_distance(self, drone_list, drone_list2):

        x_tot, y_tot = 0, 0

        for drone in drone_list2:
            x = drone.current_position_x
            y = drone.current_position_y
            x_tot = x_tot + x
            y_tot = y_tot + y

        x_ort_r = x_tot / len(drone_list2)
        y_ort_r = y_tot / len(drone_list2)

        x_tot, y_tot = 0, 0

        for drone in drone_list:
            x = drone.current_position_x
            y = drone.current_position_y
            x_tot = x_tot + x
            y_tot = y_tot + y

        self.x_ort = x_tot / len(drone_list)
        self.y_ort = y_tot / len(drone_list)

        ort_vector = np.array([self.x_ort, self.y_ort])
        ort_vector_r = np.array([x_ort_r, y_ort_r])
        magnitude = np.linalg.norm(ort_vector_r - ort_vector)

        return magnitude


    def bubble_sort(self, our_list):
        # We go through the list as many times as there are elements
        for i in range(len(our_list)):
            # We want the last pair of adjacent elements to be (n-2, n-1)
            for j in range(len(our_list) - 1):
                if our_list[j].current_position_z > our_list[j + 1].current_position_z:
                    # Swap
                    our_list[j], our_list[j + 1] = our_list[j + 1], our_list[j]
        return our_list


    def preceed(self, plan_list):
        """
        döngü içine konulmasi lazim.
        plan_list : elemanlari fonksiyon olan liste: [görev1(), görev2()]
        """
        try:

            a = plan_list[self.i]()

            # normal haraket
            if type(a) is bool:

                if a:
                    self.i += 1
                if self.i > len(plan_list) + 1:
                    rospy.on_shutdown()

            # compound haraket
            elif callable(a):
                b = a()

                if b:
                    self.i += 1
                if self.i > len(plan_list) + 1:
                    rospy.on_shutdown()
                    return False

            return True
        except IndexError:
            return False
################################################################################################################################
""" 

    LEGACY



    def waitchange(self, duration, drone_list, drone_list2, **kwargs):
        
        python sleep fonksiyonunun işe yarayacağını düşünmüyorum çünkü
        crazyflielara sürekli veri gitmeli gibi
       
        self.param.update(kwargs)
        time_start = time.time()
        # time.sleep()
        for drone in drone_list:
            drone.hover_pos = [drone.current_position_x, drone.current_position_y, drone.current_position_z]

        rate = rospy.Rate(10)
        while time.time() < time_start + duration:
            for drone in drone_list:
                if drone.link_uri == 0:  # if firefly
                    type = 'ff'
                else:  # if crazyflie
                    type = 'cf'
                self.change2(drone_list, drone_list2)
                print(drone.ns)
                v = drone.add_attractive_potential(drone.hover_pos, self.param[f'ka_hover_{type}'])
                drone.velocity_control(v, drone.hover_pos[2])
            rate.sleep()



        return True

    def change2(self, dronelist, dronelist2, **kwargs):

        #self.çıkarılan_iha_sayısı = 0

        self.param.update(kwargs)

        for drone in dronelist:
            if abs(drone.current_roll) > 15 or abs(drone.current_pitch) > 15:
                index = dronelist.index(drone)
                dronelist.pop(index)
                print(len(dronelist))
                dronelist.append(dronelist2[0])
                dronelist2.pop(0)
                #self.removed_drone_number += 1



    def look_for_change(self, drone_list2):
       
        drone_list2, görev değişimi algılandığında listeye sokulacak olan ihalar.
        

        i = 0
        while i < len(self.drone_list):
            if abs(self.drone_list[i].current_roll) > 190 or abs(self.drone_list[i].current_pitch) > 190:
                self.drone_list.remove(self.drone_list[i])
                self.drone_list.append(drone_list2[0])
                print('drone_list: ',len(self.drone_list))
                drone_list2.pop(0)
                self.removed_drone_number += 1

                print('removeddrone',self.removed_drone_number)
            else:
                i += 1


    def changefinal(self, drone_list2, **kwargs):

        self.param.update(kwargs)

        # çıkarılmayı kontrol edip varsa çıkardık
        self.look_for_change(drone_list2)

        a = self.form_formation(self.drone_list[0].virtual_lead_pos, self.drone_list, 'triangle', ka_ff=14, ka_cf=5,
                                         deadband_formation_cf=0.1)  # formasyon ve oluşturduğu noktayı parametrik yap.
        if a == True:
            if self.removed_drone_number == 2:# vl ortalama değer problemi çözülünce form formation buraya eklenecek
                return True
        else:
            return False


    def change(self, dronelist, dronelist2, **kwargs):

        self.param.update(kwargs)

        i = random.randint(0, len(dronelist))
        if dronelist[i].link_uri == 0:
            dronelist[i].position_control(dronelist[i].current_position_x, dronelist[i].current_position_y, 0, 0)
        else:
            dronelist[i].send_pose(dronelist[i].current_position_x, dronelist[i].current_position_y, 0)

        A = dronelist.pop(i)

        # j = random.randint(0, len(dronelist))
        # dronelist[j].position_control(dronelist[j].current_position_x, dronelist[j].current_position_y, 0, 0)
        # dronelist.pop(j)

        for drone in dronelist2:
            dronelist.append(drone)

        for drone in dronelist:
            print(drone.ns)

        change = False
        while not change:
            change = self.form_formation(dronelist[0].virtual_lead_pos, dronelist, 'arrow', ka_ff=14, ka_cf=5,
                                         deadband_formation_cf=0.1)  # formasyon ve oluşturduğu noktayı parametrik yap.
            A.land_single()

        # vl ortalama değer problemi çözülünce form formation buraya eklenecek
        return True
  




4. görev testleri için kullanılmış preceed func

    def preceed2(self, plan_list, plan_list_o):
        
        döngü içine konulmasi lazim.
        plan_list : elemanlari fonksiyon olan liste: [görev1(), görev2()]
        
        try:

 pe(a) is bool:

                if a and c:
                    self.i += 1
                return True

            # compound haraket
            elif callable(a):
                print("girdi")
                b = a()
                print(b)

                if b:
                    self.i += 1

                return True
        except IndexError:

            return False
"""