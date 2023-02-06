import numpy as np
import rospy
import formation

DRONE_ARRIVAL_TOLERANCE = 0.2
VL_ARRIVAL_TOLERANCE = 0.2

class Plan:
    def __init__(self, drone_list, dt):

            self.dt = dt
            self.drone_list = drone_list
            self.formation = formation.Formation(self)
            self.height = 1.0

            #görev indexi   
            self.i=0

            #first run flagler
            self.takeoff_first_run = True
            self.change_formation_first_run = True
            
    
    def _droneArrival(self):
        """
        Her bir İHA'nın eşleşmiş oldukları formasyon noktalarına varıp varmadıklarını kontrol eder.
        Vardılarsa true değeri dönülür.
        """
        arrival = True

        for drone in self.drone_list:
            pos_vec = np.array([drone.current_position_x,drone.current_position_y,drone.current_position_z])
            target_vec = np.array(drone.assigned_fp)

            if np.linalg.norm(target_vec - pos_vec) > DRONE_ARRIVAL_TOLERANCE:
                arrival = False

        return arrival

    def _virtualLeadArrival(self, target_pos_vec):
        if (np.linalg.norm(np.array(target_pos_vec) - np.array(self.formation.virtual_lead_pos[0:3])) > VL_ARRIVAL_TOLERANCE):
            return False
        else:
            return True
    
    
    #FORMASYON GÖREVLERİ

    def takeOff(self, takeoff_vel, desired_height):
        """
        desired_height: ihaların varsayılan irtifası
        float: takeoff_vel: ihaların kalkış hızı
        """

        if self.takeoff_first_run:
            #kalkışı init et
            self.height = desired_height
            #fp_list ve vl konumunu init et
            self.formation.initFormation()
            self.takeoff_first_run = False

        
        target_pos_vec = [self.formation.virtual_lead_pos[0],
                          self.formation.virtual_lead_pos[1],
                                              desired_height]

        
        if not (self._droneArrival() and self._virtualLeadArrival(target_pos_vec)):

            #orta nokta öteleme hızını belirle (bir fonksiyona çekilebilir)
            vel_sp = np.array(target_pos_vec) - np.array(self.formation.virtual_lead_pos[0:3])
            vel_sp = takeoff_vel * (vel_sp / np.linalg.norm(vel_sp)) 
            vel_sp = vel_sp.tolist()

            self.formation.updateVirtualLead(0.0, 0.0, vel_sp[2], 0.0)
            self.formation.findFormationPoints(assignment=True)
            self.formation.sendCommand()
            return False
        else:
            print("Kalkış gerçekleştirildi")
            return True


    def land(self, land_vel):
        """
        float: land_vel: iniş için gerekli hız
        """

        

        target_pos_vec = [self.formation.virtual_lead_pos[0],
                          self.formation.virtual_lead_pos[1],
                                                         0.0]

       
        if not (self._droneArrival() and self._virtualLeadArrival(target_pos_vec)):
            
            #orta nokta öteleme hızını belirle (bir fonksiyona çekilebilir)
            vel_sp = np.array(target_pos_vec) - np.array(self.formation.virtual_lead_pos[0:3])
            vel_sp = land_vel * (vel_sp / np.linalg.norm(vel_sp)) 
            vel_sp = vel_sp.tolist()

            print(vel_sp)
            
            self.formation.updateVirtualLead(0.0, 0.0, vel_sp[2], 0.0)
            self.formation.findFormationPoints(assignment=True)
            self.formation.sendCommand()
            return False
        else:
            print("bitti ^_^ ")
            #motorları kapat komutu gönder burada
            return True

    
    def changeFormation(self, fp_list):

        if self.change_formation_first_run:
            self.formation.fp_list = fp_list
            self.formation.findFormationPoints()
            self.formation.assignment()
            self.change_formation_first_run = False
            print("formasyon değiştiriliyor...")
        
        if not self._droneArrival():
            self.formation.sendPotentialCommand()
            return False
        else:
            print("formasyon değiştirildi")
            return True



    def moveFormation(self, desired_x, desired_y, move_vel):
        """
        float: land_vel: iniş için gerekli hız
        """

        target_pos_vec = [desired_x, desired_y, self.height]


        if not (self._droneArrival() and self._virtualLeadArrival(target_pos_vec)):
            
            #orta nokta öteleme hızını belirle (bir fonksiyona çekilebilir)
            vel_sp = np.array(target_pos_vec) - np.array(self.formation.virtual_lead_pos[0:3])
            vel_sp = move_vel * (vel_sp / np.linalg.norm(vel_sp)) 
            vel_sp = vel_sp.tolist()

            print(vel_sp)
            
            self.formation.updateVirtualLead(vel_sp[0], vel_sp[1], 0.0, 0.0)
            self.formation.findFormationPoints(assignment=True)
            self.formation.sendCommand()
            return False
        else:
            print("formasyon hedefe vardı")
            #motorları kapat komutu gönder burada
            return True


    def rotateFormation(self):
        pass

    
    def landAsynch(self):
        # yeni formasyon sitemine uygun landing
        # iniş için listeyi düzenle 
        if self.land_formation_asc_first_run:
            self.hover_list = self.drone_list.copy()
            self.bubble_sort(self.hover_list)
            self.land_formation_asc_first_run = False

            # bunları r'ye bağlı yap.
            landing_points = [[-1, 1, 0], [1, 0, 0], [1, 1, 0]]
            self.formation.fp_list = landing_points
            landing_points_global = self.find_formation_points() # formasyon noktalarını bulur
            self.hungarian(landing_points_global) # eşleşirme ve atama işlemini yapar.

            for drone in self.hover_list:
                # uçuş noktalarını belirle
              
                # hover için nokta belirle (cf)
                drone.hover_pos = [drone.current_position_x, drone.current_position_y, drone.current_position_z]
                # inen için nokta belirle
            
            self.land_formation_asc_first_run = False

        # haraketi uygula
        try:
                
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

                if cond :
                    print('bitmez çile')
                    # alttaki satırı silersen zıp zıp formasyon oluyo xd -isomuso -leventi bıçaklayan olcay
                    self.hover_list.remove(self.hover_list[0])


        except ZeroDivisionError:
            pass

            # haraketi kontrol et
            if len(self.hover_list) == 1:
                return True
            else:  # b == 0
                return False

   

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

