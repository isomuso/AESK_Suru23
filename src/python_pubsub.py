#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray



class Drone:
    def __init__(self):
        self.pos_x = float()
        self.pos_y = float()
        self.pos_z = float()

        self.first_run_take_off = True
        self.take_off_x = float()
        self.take_off_y = float()

        rospy.Subscriber("get_position", Float32MultiArray, self.position_callback)
        self.pub = rospy.Publisher('move_drone', Float32MultiArray, queue_size=10)
    
    def position_callback(self, msg):
        self.pos_x = msg.data[0]
        self.pos_y = msg.data[1]
        self.pos_z = msg.data[2]

        print("x: ", self.pos_x)
        print("y: ", self.pos_y)
        print("z: ", self.pos_z)
    
    
    def set_speed_setpoint(self, control_setpoint):
        """
        control_setpoint :: liste :: [float x, float y, float z]
        """
        msg = Float32MultiArray()
        msg.data = control_setpoint
        print(control_setpoint)
        self.pub.publish(msg)
        

    def take_off(self, desired_height):
        if self.first_run_take_off:
            self.take_off_x = self.pos_x
            self.take_off_y = self.pos_y
            self.first_run_take_off = False
            return False
        else: 

            if self.pos_z < desired_height:
                self.set_speed_setpoint([0, 0, 1])
                return False

            else:
                return True
            

    def wait(self):
        pass

    def land(self):
        if self.pos_z > 0:
            self.set_speed_setpoint([0, 0, -1])
            return False

        else:
            return True


    
 
rospy.init_node('gcs', anonymous=True)
rate = rospy.Rate(10) # 10hz

crazyflie = Drone()

a = False

def talker():
    global a
    while not rospy.is_shutdown():

        while not a:
            a = crazyflie.take_off(4)
            rate.sleep()
        
        a = False
        while not a:
            a = crazyflie.land()
            rate.sleep()

        rate.sleep()

if __name__ == '__main__':
    talker()