import numpy as np
import hungarian


import tf2_ros
import rospy

class Formation():
    def __init__(self, mission):
        self.virtual_lead_pos = [0.0, 0.0, 0.0, 0.0] #X, Y, Z, Heading
        self.fp_list = list()
        self.mission = mission #mission objesi
    
    def initFormation(self):

        """
        İHA'lar kalkmadan önce İHA'ların yerdeki mevcut dizilimini formasyon olarak tanımlar.
        İHA'ların konumlarının ortalamalarını orta nokta olarak init eder.

        """

        print('Formasyon başlatılıyor...')

        self.fp_list = []

        #orta nokta vektörü
        average_vector = np.array([0, 0, 0])

       
        # konumların ortalaması alındı
        for drone in self.mission.drone_list:
            drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
            average_vector = average_vector + drone_pos_vec
        average_vector = average_vector / len(self.mission.drone_list)

        self.virtual_lead_pos = average_vector.tolist()

        #heading eklendi 
        self.virtual_lead_pos.append(0.0)
        
        #fp_list'i halleder

        # formasyon noktalarını güncelle
        for drone in self.mission.drone_list:
            drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
            formation_vector = drone_pos_vec - average_vector
            formation_vector = formation_vector.tolist()
            self.fp_list.append(formation_vector)
        
    
    def assignment(self, formation_points, return_list=False):
        """
        formation_points: [[x,y,z], ......] find_formation_pointsten aldığı noktaları atar
        çıktı olarak mission.drone_list içerisindeki assigned_fpleri günceller.

        return_list true yapıldığı zaman, formasyon noktalarını içeren liste, dronelistteki droneların sırasına göre
        sıralanıyor. eğer drone listesinin sırası değişirse bir sebepten ötürü istenmeyen sonuçlar olabilir.

        return: [[x, y,z ], [x, y, z], ...]
        """
        print("Atama gerçekleştiriliyor...")
        cost_matrix = []
        global_fp_list = []

        for drone in self.mission.drone_list:
            for fp in formation_points:
                drone_pos = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
                f_pos = np.array(fp)
                dist_vec = drone_pos - f_pos
                dist_mag = np.linalg.norm(dist_vec)
                cost_matrix.append(dist_mag)

        cost_matrix = np.reshape(cost_matrix, (len(self.mission.drone_list), len(formation_points)))
        ans_pos = hungarian.hungarian_algorithm(cost_matrix.copy())  # Get the element position.
        ans, ans_mat = hungarian.ans_calculation(cost_matrix,
                                                 ans_pos)  # Get the minimum or maximum value and corresponding matrix.

        for drone in self.mission.drone_list:
            index = self.mission.drone_list.index(drone)
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
    

    def findFormationPoints(self, vl_pos=[], assignment=False):
        """
        girdi olarak orta noktayı ve orta noktaya göre olan formasyon noktalarını alıp
        çıktı olarak mutlak 0'a göre olan formasyon noktalarını döner

        - fplist = [[x, y, z], [x, y, z], .... ] formasyon noktalarını vl'ye göre içeren liste
        formasyon noktalarını plannerlistte bir değişkende kaydmissiontenen nself.virtual_lead_heading
        - virtual_lead_pos'e bir değer atanırsa o noktada formasonu oluşturur atanmazsa en son
        vl nerede kaldıysa orada oluşturur

        !!!! Yeni formasyon oluşturmak için kullanılacak olursa self.new_formation attributesi True
        yapılmalımission
        """

        if len(vl_pos) == 0:
            vl_pos = self.virtual_lead_pos
        else:
            pass


        # print('vl konumu', self.virtual_lead_pos)
        # transformları tutacak buffer
        buffer_core = tf2_ros.BufferCore(
            rospy.Duration(10.0)
        )  # 10 saniye cache süresi galiba
        
        # mapten orta noktaya transformları buffera koy
        ts1 = self.transform_from_euler(
            vl_pos[0],
            vl_pos[1],
            vl_pos[2],
            0,
            0,
            vl_pos[3],
            "map",
            "frame1",
        )
        buffer_core.set_transform(ts1, "default_authority")
      
        # formasyon tanımlamayı buna benzer bir yöntem ile basitleştirebilirsin.

        i = 2
        formation = []
        for fp in self.fp_list:
            # orta noktadan formasyon noktalarına olan transform
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
            i += 1  # buna gerek olmayabilirist()
            i = 0
            for drone in self.mission.drone_list:
                drone.assigned_fp = formation[i]
                i += 1

        if assignment:
            i = 0
            for drone in self.mission.drone_list:
                drone.assigned_fp = formation[i]
                i += 1
               

        else:
            return formation
    
    
    
    def updateVirtualLead(self, x_vel, y_vel, z_vel, heading_vel):
        """
        sürü merkezi referansını aldığı hız girdileri doğrultusunda öteler

        float: x_vel       : m/s
        float: y_vel       : m/s
        float: z_vel       : m/s
        float: heading_vel :derece/s

        """

        self.virtual_lead_pos[0] += x_vel       * self.mission.dt 
        self.virtual_lead_pos[1] += y_vel       * self.mission.dt
        self.virtual_lead_pos[2] += z_vel       * self.mission.dt
        self.virtual_lead_pos[3] += heading_vel * self.mission.dt

    def sendCommand(self):
        """
        İHA'ları eşleştirilmiş olduklara noktalara öteletmek için tek döngülük komut gönderir. 
        Assigned fpler güncellenmezse hover atılır. 
        """

        # görevi uygula
        for drone in self.mission.drone_list:
            try:
                if drone.link_uri == 0:  # if firefly
                    drone.position_control(drone.assigned_fp[0], drone.assigned_fp[1], drone.assigned_fp[2], 0)

                else:  # if crazyflie
                    drone.send_pose(drone.assigned_fp[0], drone.assigned_fp[1], drone.assigned_fp[2], 0)

            except ZeroDivisionError:
                pass

