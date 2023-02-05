import drone as d
import mission as p

import rospy
import time


rospy.init_node("liteunant")
FREQUENCY = 30



drone_list = [d.Drone(1, link_uri=0), d.Drone(2, link_uri=0), d.Drone(3, link_uri=0), d.Drone(4, link_uri=0), 
                        d.Drone(5, link_uri=0), d.Drone(6, link_uri=0), d.Drone(7, link_uri=0), d.Drone(8, link_uri=0), d.Drone(9, link_uri=0)]
plan = p.Plan(drone_list, 1/FREQUENCY)

d1 = lambda : plan.takeOff(1, 1)
m_list = [d1]

a =1

if a == 1:
    # ana döngüyü initialize et
    FREQUENCY = 30  # döngü frekansı
    r = rospy.Rate(FREQUENCY)
    for drone in drone_list:
        drone.dt = 1 / FREQUENCY
    run = True

    time.sleep(2)

    while not rospy.is_shutdown() and run:
        run = plan.preceed(m_list)
        r.sleep()
