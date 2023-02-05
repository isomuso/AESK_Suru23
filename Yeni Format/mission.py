import numpy as np
import rospy
import formation

DRONE_ARRIVAL_TOLERANCE = 0.1
VL_ARRIVAL_TOLERANCE = 0.1

class Plan:
    def __init__(self, drone_list, dt):
            self.dt = dt
            self.drone_list = drone_list
            self.formation = formation.Formation(self)
            self.height = 1.0

            self.i=0



            #first run flagler

            self.takeoff_first_run = True
            
    
    def checkDroneArrival(self):
        """
        Her bir İHA'nın eşleşmiş oldukları formasyon noktalarına varıp varmadıklarını kontrol eder.
        Vardılarsa true değeri dönülür.
        """
        arrival = True

        for drone in self.drone_list:
            pos_vec = np.array([drone.current_position_x,drone.current_position_y,drone.current_position_z])
            target_vec = np.array([drone.assigned_fp[0], drone.assigned_fp[1],drone.assigned_fp[2]])

            if(np.linalg.norm(target_vec - pos_vec) > DRONE_ARRIVAL_TOLERANCE):
                arrival = False

        return arrival

    def checkVirtualLeadArrival(self, target_pos_vec):
        if (np.linalg.norm(np.array(target_pos_vec) - np.array(self.formation.virtual_lead_pos)) > VL_ARRIVAL_TOLERANCE):
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

        if not(self.checkDroneArrival() and 
                self.checkVirtualLeadArrival([self.formation.virtual_lead[0], 
                self.formation.virtual_lead[1], desired_height, 0])):

            self.formation.updateVirtualLead(0.0, 0.0, takeoff_vel, 0.0)
            self.formation.findFormationPoints(assignment=True)
            self.formation.sendCommand()
            return False
        else:
            return True


    def land():
        pass
    
    def changeFormation():
        pass

    def moveFormation():
        pass


    def rotateFormation():
        pass

    
    def landAsynch():
        pass

   

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
