import numpy as np
import tools


class Commander():
    def __init__(self, plan):
        self.plan = plan

    def set_formation(self, formation_points):

        """
        verilen noktaları formasyon noktaları olarak ayarlar.
        formation_points: liste: [ [x,y,z], [x,y,z], ... ]
        """
        pass
    

    def initialize_formation(self, desired_fp_list=[], vl_pos=[]):

        """
        fp_list'i ve virtual_lead_pos'u günceller.

        droneların mevcut konumlarını formasyon noktası olarak atar.
        formation_points: liste: [ [x,y,z], [x,y,z], ... ]
        """

        self.plan.fp_list = []

        #vl_posu halleder

        if len(vl_pos) == 0:
            # ihaların orta noktasını hesapla
            average_vector = tools.find_average_pos(self.plan.drone_list)
            # orta noktanın konumunu güncelle
            self.plan.virtual_lead_pos = average_vector.tolist()
        else: 
            average_vector = tools.find_average_pos(self.plan.drone_list)
            self.plan.virtual_lead_pos = vl_pos
        
        #fp_list'i halleder

        if len(desired_fp_list) == 0:
            # formasyon noktalarını güncelle
            for drone in self.plan.drone_list:
                drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
                formation_vector = drone_pos_vec - average_vector
                formation_vector = formation_vector.tolist()
                self.plan.fp_list.append(formation_vector)
        
        else:
            self.plan.fp_list = desired_fp_list
        
    
       

