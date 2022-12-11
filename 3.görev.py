import bn_sim as bn
import mission_planner_sim as mp
import rospy
import time
from cflib.utils import uri_helper
import cflib.crtp

rospy.init_node("liteunant")

cflib.crtp.init_drivers()




#threatlist = [bn.Obstacle(1,1,0.5)]  # [bn.Obstacle(4, 6, [0.5, 0.5]), bn.Obstacle(6, 4, [0.5, 0.5]), bn.Obstacle(8, 8, [0.5, 0.5]), bn.Obstacle(3, 3, [0.5, 0.5])]



mission = mp.Plan()


drone_list = [bn.Drone(1, link_uri=0), bn.Drone(2, link_uri=0), bn.Drone(3, link_uri=0)]

#3. görev
mission.drone_list = drone_list
mission.threatlist = []


d1 = lambda : mission.march_test1([5, 5, mission.virtual_lead_pos[2]])

d2 = lambda : mission.march_test1([mission.virtual_lead_pos[0],
                                    mission.virtual_lead_pos[1],
                                    3])

d3 = lambda : mission.march_test1([mission.virtual_lead_pos[0],
                                    mission.virtual_lead_pos[1],
                                    0])

d4 = lambda: mission.march_test2(60)
d5 = lambda: mission.land_formation_asc_test()
d6 = lambda: mission.march_test3([[1,1,0], [2,3,1], [3,1,-0.5]])
d7 = lambda: mission.march_test3([[1,-1,0], [2,-3,1], [1,1,0.5]])


#m_list = [d2, d4]
m_list = [d2, d6, d7, d1, d4, d5, d2, d3]



a =1

if a == 1:
    # ana döngüyü initialize et
    frequency = 30  # döngü frekansı
    r = rospy.Rate(frequency)
    for drone in drone_list:
        drone.dt = 1 / frequency
    run = True

    time.sleep(2)

    while not rospy.is_shutdown() or run:
        print(mission.virtual_lead_pos)
        run = mission.preceed(m_list)
        r.sleep()
