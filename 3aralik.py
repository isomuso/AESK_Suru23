
import bn_sim as bn
import mission_planner_sim as mp
import rospy
import time
from cflib.utils import uri_helper
import cflib.crtp

rospy.init_node("liteunant")

cflib.crtp.init_drivers()
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# parameters

#dronelist = [bn.Drone(1, link_uri=uri, sim=False)]

dronelist = [bn.Drone(1, link_uri=0)]

a = 1

threatlist = []  # [bn.Obstacle(4, 6, [0.5, 0.5]), bn.Obstacle(6, 4, [0.5, 0.5]), bn.Obstacle(8, 8, [0.5, 0.5]), bn.Obstacle(3, 3, [0.5, 0.5])]




mission = mp.Plan(threatlist)

for drone in dronelist:  # takeoff eklenince buna gerek olmayabilir
    drone.d_h = mission.d_h


# m_list = [mf0, mf1, mf4, mf5]


#mission.fp_list = [[0.01, 0.01, 0]]
mission.drone_list = dronelist
mission.threatlist = threatlist
mission.fp_list = [[1,1], [-1, -1], [1, 0]]
mission.virtual_lead_pos = [0,0,0]
mission.dt = 1/30


#9 drone 1. görev
# alan 5x5

k1 = lambda : mission.take_off_test(0.6)
k2 = lambda : mission.move_drone(dronelist[0], [1, 0])
k9 = lambda : mission.land_formation(dronelist)

m_list = [k1, k2, k9]

time.sleep(1)
if a == 1:
    # ana döngüyü initialize et
    frequency = 40  # döngü frekansı
    r = rospy.Rate(frequency)
    for drone in dronelist:
        drone.dt = 1 / frequency
    run = True

    time.sleep(2)
    while not rospy.is_shutdown() or run:
        run = mission.preceed(m_list)
        r.sleep()
