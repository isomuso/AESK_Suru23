import numpy as np

class Map():

    def __init__(self):

        self.map_coordinate_x = list()
        self.map_coordinate_y = list()
        self.map_bound_topright = list()
        self.map_bound_bottomleft = list()
        self.map_area_size = np.array(self.map_bound_bottomleft) * np.array(self.map_bound_topright)
        self.step_size = int()
        self.resoulution = int()
        
    
    def add_circular_obstacle(self):
        pass

    def configure_map(self):
        self.step_size = self.map #set step_size

  

    
