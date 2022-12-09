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
    

    def initialize_formation(self):

        """
        droneların mevcut konumlarını formasyon noktası olarak atar.
        formation_points: liste: [ [x,y,z], [x,y,z], ... ]
        """

        self.plan.fp_list = []

        # ihaların orta noktasını hesapla
        average_vector = tools.find_average_pos(self.plan.drone_list)

        # orta noktanın konumunu güncelle
        self.plan.virtual_lead_pos = average_vector.tolist()

        # formasyon noktalarını güncelle
        for drone in self.plan.drone_list:
            drone_pos_vec = np.array([drone.current_position_x, drone.current_position_y, drone.current_position_z])
            formation_vector = drone_pos_vec - average_vector
            formation_vector = formation_vector.tolist()
            self.plan.fp_list.append(formation_vector)
       

