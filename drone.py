#!/usr/bin/env python

# from hibrit_c1 import hungarian
import matplotlib.pyplot as plt
from geometry_msgs.msg import TransformStamped
import rospy
import math
import time
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform, Quaternion

# from geometry_msgs.msg import Point32
import std_msgs.msg
from geometry_msgs.msg import Point
import tf
from nav_msgs.msg import Odometry
import numpy as np

# import logging


# import cflib.crtp  # noqa
#from cflib.crazyflie import Crazyflie
#from cflib.crazyflie.log import LogConfig
#from cflib.crazyflie.commander import Commander
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import astar as a

#from geometry_msgs.msg import Point
#from custom_msg.msg import obstacle
#from custom_msg.msg import general_parameters #import custom_msg which is in the workspace

class Drone:
    def __init__(self, ns, link_uri=0, sim=False, zumo=False):
        self.link_uri = link_uri
        self.ns = ns
        self.sim = sim
        self.zumo = zumo

        if self.zumo:
            self.d_h = 0

            #burada diğer sub, pub olayları için gerekenleri yaz

        if type(self.link_uri) == str and self.sim == True:
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            except rospy.ServiceException:
                print("Service call failed")

            self.state_msg = ModelState()
            # self.state_msg.model_name = 'firefly'
            self.state_msg.model_name = f'firefly{self.ns}'

            # drone namespace numarası

        if type(self.link_uri) == str:
            # cflib
            self._cf = Crazyflie(rw_cache="./cache")
            self._cf.connected.add_callback(self._connected)
            self._cf.disconnected.add_callback(self._disconnected)
            self._cf.connection_failed.add_callback(self._connection_failed)
            self._cf.connection_lost.add_callback(self._connection_lost)
            time.sleep(1)
            print("Connecting to %s" % link_uri)
            # Try to connect to the Crazyflie
            self._cf.open_link(link_uri)
            # Variable used to keep main loop occupied until disconnect
            self.is_connected = True
            self.command = Commander(self._cf)
            self.crazyswarm_pose = []

        if (type(self.link_uri) == str) and (self.zumo == True):
            uri = link_uri[-2:]
            rospy.Subscriber(f"general_parameters/ika_{uri}", general_parameters, self.callback_zumo)

        # ----------------------------------------------------------

        else:
            rospy.Subscriber(
                f"/firefly{ns}/odometry_sensor1/odometry", Odometry, self.callback
            )

        # dronenun mape göre mevcut konumu
        self.current_position_x = 0
        self.current_position_y = 0
        self.current_position_z = 0


        self.current_roll = 0
        self.current_yaw = 0
        self.current_pitch = 0
        # varılması istenen noktalar (vel control ve pos control kullanıyor)
        self.desired_x = 0
        self.desired_y = 0
        self.desired_z = 0
        # droneların formasyon noktalarını bulunduran liste [f1=[x,y,z], f2=[x,y,z], .. ]
        self.formation = []
        # sürüdeki diğer droneların nesnelerini bulundurur
        self.detected_coworkers = []
        # vl'nin konumunu içeren liste [x, y, z] !!z olmak zorunda
        self.virtual_lead_pos = [0, 0, 0]
        # dronea atanmış formasyon noktasının konumunu içeren liste [x, y, z]
        self.assigned_fp = []
        # dronun initial heading'den ne kadar döndüğünü veren açı (flag kontrolü için)
        self.rotated_angle = 0
        # hangi formasyon haraketinin gerçekleştiğini ifade eden flagler
        self.formed_formation = False
        self.rotating = False
        # vl framenin, map frameine göre, saat yönünün tersinde derece cinsinden kaç
        # derece döndüğününü veren açı
        self.current_heading = 0
        # eş zamanlı atama algoritmasından gelen noktaları içeren liste
        self.paired_points = []
        # eş zamanlı atama noktasına publish edilecek noktaları içeren liste
        self.formation_points = []
        # default irtifa
        self.d_h = 1
        #
        self.dx = 0
        self.dy = 0

        # drone kalkışını initialize etme
        self.first_run_take_off = True  # drone kaldırırken, ilk noktasını korumak için flag
        self.first_run_land = True  # droneları indirme için kullanılan flag
        self.take_off_pos = []  # dronun ilk bağlantı kurduğu noktası.
        self.land_pos = []  # dronun inmek için koruyacağı nokta.
        self.hover_pos = []

        # --------------------- PROTOTIP ---------------------------
        self.liquid_formation = False
        self.obs_list_i = []  # hedef noktası ile engel arasında bulunan engeller (check_obs_intersection)
        # -----------------------------------------------------

        # eski
        self.desired_heading = []
        self.angle = 0

        # Formasyon
        self.formation_name = ''
        self.formation_radius = 2.0
        self.first_run_form_land = True

        # ASTAR
        self.path = []  # [[x,y], [x,y], .....] a star hedef noktalar
        self.route = True  # yol çizme talimatı için flag
        self.path_i = 1  # self.path index

        # DÖNGÜ
        self.dt = 1 / 30

        # lol
        self.fire_point_x = []
        self.fire_point_y = []

        self.current_thrust = 0

        self.potansiyel_flag = False
        self.varış_noktası = []

    """
                    ---CALLBACK FONKSİYONLARI---
    """

    def callback_zumo(self, data):
        self.current_position_x = data.pose.x
        self.current_position_y = data.pose.y

    def callback(self, msg):  # GAZEBO POSE
        self.current_position_x = msg.pose.pose.position.x
        self.current_position_y = msg.pose.pose.position.y
        self.current_position_z = msg.pose.pose.position.z
        self.current_roll = msg.pose.pose.orientation.x * 360
        self.current_pitch = msg.pose.pose.orientation.y * 360
        self.current_yaw = msg.pose.pose.orientation.z * 360
        """print(self.current_roll, 'ROL')
        print(self.current_pitch, 'PİTCH')"""
        # rospy.loginfo('Firefly{} x: {}, y: {}, z: {}'.format(self.ns,
        # self.current_position_x, self.current_position_y, self.current_position_z))

    """
                ---CFLİB FONKSİYONLARI---
    """

    def _connected(self, link_uri):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link_uri)


        #OPTİMİZASYON İÇİN STABILIZER VE PM KAPATILABİLİR

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        self._lg_stab.add_variable('stabilizer.thrust', 'float')


        #______________

        self._lg_stab1 = LogConfig(name="stateEstimate", period_in_ms=10)
        self._lg_stab1.add_variable("stateEstimate.x", "float")
        self._lg_stab1.add_variable("stateEstimate.y", "float")
        self._lg_stab1.add_variable("stateEstimate.z", "float")

        #______________

        self._lg_stab2 = LogConfig(name="pm", period_in_ms=10)
        self._lg_stab2.add_variable("pm.batteryLevel", "float")
        #--------------------------------------------------------

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            self._cf.log.add_config(self._lg_stab1)
            self._cf.log.add_config(self._lg_stab2)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            self._lg_stab1.data_received_cb.add_callback(self._stab_log_data1)
            self._lg_stab2.data_received_cb.add_callback(self._stab_log_data2)

            # Start the logging
            self._lg_stab.start()
            self._lg_stab1.start()
            self._lg_stab2.start()

        except KeyError as e:
            print(
                "Could not start log configuration,"
                "{} not found in TOC".format(str(e))
            )
        except AttributeError:
            print("Could not add Stabilizer log config, bad configuration.")

    def _stab_log_data2(self, timestamp, data, logconf):
        self.current_battery = data['pm.batteryLevel']
        #print('batarya', self.current_battery)

    def _stab_log_data1(self, timestamp, data, logconf):
        self.current_position_x = data['stateEstimate.x']
        self.current_position_y = data['stateEstimate.y']
        self.current_position_z = data['stateEstimate.z']

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""


        self.current_roll = data['stabilizer.roll']
        self.current_pitch = data['stabilizer.pitch']
        self.current_yaw = data['stabilizer.yaw']
        self.current_thrust = data['stabilizer.thrust']



        if self.sim:
            self.state_msg.pose.position.x = self.current_position_x
            self.state_msg.pose.position.y = self.current_position_y
            self.state_msg.pose.position.z = self.current_position_z + 0.08
            self.state_msg.pose.orientation.w = 1
            self.state_msg.pose.orientation.x = self.current_pitch / 360
            self.state_msg.pose.orientation.z = self.current_yaw / 360
            self.state_msg.pose.orientation.y = self.current_roll / 360
            resp = self.set_state(self.state_msg)

        """rospy.loginfo(
            f" POSITION x:  {self.current_position_x:3.3f},  y: {self.current_position_y:3.3f},  z: {self.current_position_z:3.3f}"
        )
        rospy.loginfo(
            f" ORIENTATION x: {self.current_roll:3.3f},  y: {self.current_pitch:3.3f},  z: {self.current_yaw:3.3f}"
        )"""

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)
        self.is_connected = False




    """
                    --- KONTROLCÜLER ---
    """

    def send_pose(self, x, y, z):
        self.command.send_position_setpoint(x, y, z, 0)

    def velocity_control_land(self, desired_velocity):  # x hız, y hız, z konum
        """
        x y girdisi olarak hız vektörü z girdisi olarak konum alıyor.
        desired_velocity: list: [float:x, float:y, float:z]:
        dt: float:
        """
        try:

            self.desired_x = self.current_position_x + desired_velocity[0] * self.dt
            self.desired_y = self.current_position_y + desired_velocity[1] * self.dt
            self.desired_z = self.current_position_z + desired_velocity[2] * self.dt
            # rospy.loginfo("x:{} y:{} z:{}".format(self.desired_x,self.desired_y,self.desired_z))
            if self.link_uri == 0:  # eğer cf ise
                self.position_control(
                    self.desired_x, self.desired_y, self.desired_z, 0)
            else:  # eğer reel ise
                self.send_pose(self.desired_x, self.desired_y, self.desired_z)

        except:
            pass

    def velocity_control(self, desired_velocity, desired_height):  # x hız, y hız, z konum
        """
        x y girdisi olarak hız vektörü z girdisi olarak konum alıyor.
        desired_velocity: list: [int:x, int:y]
        desired_height: int:
        dt: int:
        """
        try:
            self.desired_x = self.current_position_x + desired_velocity[0] * self.dt
            self.desired_y = self.current_position_y + desired_velocity[1] * self.dt
            # rospy.loginfo("x:{} y:{} z:{}".format(self.desired_x,self.desired_y,self.desired_z))
            if self.link_uri == 0:  # eğer cf ise
                self.position_control(
                    self.desired_x, self.desired_y, desired_height, 0)
            elif self.zumo: #ZUMOYSAN GİR İSMAİL ANLASIN DİYE YAZILDI
                self.zumo_move(self.desired_x, self.desired_y, 0)
            else:  # eğer reel ise
                self.send_pose(self.desired_x, self.desired_y, desired_height)

        except:
            pass

    def velocity_control_z(self, x, y, v_z):
        """
        x, y girdisi olarak konum alıyor, z girdisi olarak hız vektörü
        desired_height: int:
        dt: int:
        """
        try:
            self.desired_z = self.current_position_z + v_z * self.dt
            # rospy.loginfo("x:{} y:{} z:{}".format(self.desired_x,self.desired_y,self.desired_z))

            if self.link_uri == 0:  # eğer cf değil ise
                self.position_control(
                    x, y, self.desired_z, 0)
            else:  # eğer reel ise
                self.send_pose(x, y, self.desired_z)
        except:
            pass

    def zumo_move(self, x, y, z):
        uri = self.link_uri[-2:]
        pub = rospy.Publisher(f"goal_pose/ika_{uri}", Point, queue_size = 10)
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        pub.publish(point)


    def position_control(
            self, desired_x_to_go, desired_y_to_go, desired_z_to_go, desired_yaw
    ):
        firefly_command_publisher = rospy.Publisher(
            "/firefly{}/command/trajectory".format(self.ns),
            MultiDOFJointTrajectory,
            queue_size=10,
        )
        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, math.radians(desired_yaw)
        )  # roll,yaw pitch için
        traj = MultiDOFJointTrajectory()  # kontrolcüye gönderilecek mesaj
        # mesaja istenen parametrelerin aktarılması (header)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time()
        header.frame_id = "frame"
        traj.joint_names.append("base_link")
        traj.header = header
        # istenen nokta için dönüşümler
        transforms = Transform(
            translation=Point(
                desired_x_to_go, desired_y_to_go, desired_z_to_go),
            rotation=Quaternion(
                quaternion[0], quaternion[1], quaternion[2], quaternion[3]
            ),
        )
        velocities = Twist()
        accelerations = Twist()
        point = MultiDOFJointTrajectoryPoint(
            [transforms], [velocities], [accelerations], rospy.Time(2)
        )
        traj.points.append(point)
        firefly_command_publisher.publish(traj)

    # -------------------------------------------------------------------------------------------------------------------------------------
    #                            ANTI LOKAL MINIMA
    # ------------------------------------------------------------------------------------------------------------------------------------

    def add_cohesion(self, drone_list, kc):
        pos = np.array([0, 0])
        for drone in drone_list:
            x = drone.current_position_x
            y = drone.current_position_y
            pos_vec = np.array([x, y])
            pos = pos + pos_vec
        pos = kc * (pos / len(drone_list))
        return pos

    # -------------------------------------------------------------------------------------------------------------------------------------
    #                            ATTRACTIVE FONKSIYONLAR
    # ------------------------------------------------------------------------------------------------------------------------------------

    def constant_speed(self, wp, speed):
        """
        wp : [x,y]
        speed: float
        """
        dp = np.array(wp)
        cp = np.array([self.current_position_x, self.current_position_y])

        v = dp - cp
        v = (v / np.linalg.norm(v)) * speed
        return v

    def draw_path(self, gx, gy, threatlist, grid_size=1, robot_radius=0.8, sx=[], sy=[], destination_list = []):

        # engelleri ekleyecek fonksiyonı çağır, ox oy ata
        if sx == [] and sy == []:
            sx = self.current_position_x
            sy = self.current_position_y

        min_x = -2.5
        min_y = -2.5

        max_x = 2.5
        max_y = 2.5

        ox = []
        oy = []

        i = 0
        for x, y in zip(self.fire_point_x, self.fire_point_y):
            if i % 5 == 0:
                ox.append(x)
                oy.append(y)
            i += 1

        # alt yatay sınır
        for i in np.arange(min_x, max_x, grid_size * 10):
            ox.append(i)
            oy.append(min_y)
        # üst yatay sınır
        for i in np.arange(min_x, max_x, grid_size * 10):
            ox.append(i)
            oy.append(max_y)
            # alt yatay sınır
        for i in np.arange(min_y, max_y, grid_size * 10):
            ox.append(min_x)
            oy.append(i)
            # üst yatay sınır
        for i in np.arange(min_y, max_y, grid_size * 10):
            ox.append(max_x)
            oy.append(i)

        for threat in threatlist:

            # daire çiz
            x_center = threat.current_position_x
            y_center = threat.current_position_y
            #radius = threat.width
            radius = threat.radius
            for angle in np.arange(0, 360, 0.2):
                x = x_center + radius * np.cos(angle)
                y = y_center + radius * np.sin(angle)

                point = [x, y]

                ox.append(point[0])
                oy.append(point[1])

        a_star = a.AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy)

        rx.reverse()
        ry.reverse()

        """if True:  # pragma: no cover
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(rx, ry, "-r")
            plt.pause(0.0001)
            plt.show()"""

        for x, y in zip(rx, ry):
            point = [x, y]
            self.path.append(point)

        if rx == [gx]:
            self.varış_noktası = [gx, gy]
            self.potansiyel_flag = True



        return rx, ry


    def is_arrived_list(self, desired_point_list, deadband=0.40):
        """
        desired_point_list = [[x,y], [x,y], [x,y], .....]

        """

        drone_pos_vec = np.array(
            [
                self.current_position_x,
                self.current_position_y,
            ]
        )

        for dp in desired_point_list[self.path_i:]:
            desired_pos_vec = np.array(dp)

            dist = drone_pos_vec - desired_pos_vec
            dist_mag = np.linalg.norm(dist)

            print(dist_mag)

            if abs(dist_mag) > deadband:
                return False
            else:
                return True

    def is_arrived(self, desired_point, deadband=0.3):
        """
        desired_point = [x, y, z]
        """

        drone_pos_vec = np.array([self.current_position_x,
                                  self.current_position_y
                                  ]
                                 )

        desired_pos_vec = np.array(desired_point)

        dist = drone_pos_vec - desired_pos_vec
        dist_mag = np.linalg.norm(dist)

        print(dist_mag)

        if abs(dist_mag) > deadband:
            return False
        else:
            return True

    def add_attractive_potential(self, desired_point, ka):
        v_at_x = -ka * (self.current_position_x - desired_point[0])
        v_at_y = -ka * (self.current_position_y - desired_point[1])
        vel = [v_at_x, v_at_y]
        return vel

    def add_attractive_potential_z(self, desired_altitude, ka):
        set_point_z = -ka * (self.current_position_z - desired_altitude)
        return set_point_z

    def add_constant_speed_z(self, desired_altitude, ka):
        set_point_z = -ka * (self.current_position_z - desired_altitude)
        return set_point_z

    def add_attractive_potential_vl(self, desired_point, ka):
        v_at_x = -ka * (self.virtual_lead_pos[0] - desired_point[0])
        v_at_y = -ka * (self.virtual_lead_pos[1] - desired_point[1])
        vel = [v_at_x, v_at_y]
        return vel

    # ---------------------------------------------------------------------------------------------------------------------------------------
    #                                 REPULSIVE FONKSIYONLAR
    # --------------------------------------------------------------------------------------------------------------------------------------

    # threat argümanı: obstacle obje listesini veya drone obje listesini gir

    def add_repulsive_potential(
            self, threat_list, safe_dist, kr
    ):  # droneların birbirinden kaçınması için kullanılıyor. z ekseni için eklemeler yapılabilir
        v_repp = np.array([0, 0])
        for threat in threat_list:
            x_dist = -self.current_position_x + threat.current_position_x
            y_dist = -self.current_position_y + threat.current_position_y
            dist = math.sqrt((x_dist ** 2) + (y_dist ** 2))
            if dist <= safe_dist:
                vec_x = -kr * (1 - (dist / safe_dist)) * (x_dist / (dist ** 3))
                vec_y = -kr * (1 - (dist / safe_dist)) * (y_dist / (dist ** 3))
                vec = np.array([vec_x, vec_y])
                v_repp = v_repp + vec
            elif dist > safe_dist:
                pass
        v_repp.tolist()
        return v_repp

    def add_virtual_repulsive_potential(
            self, threat_list, kd, kr, kv, proportional
    ):  # engellerden kaçınma için alternatif
        v_repp = np.array([0, 0])
        for threat in threat_list:
            # desired_point liste, gitmek istediğin noktanın listesi x y z
            # fonksiyon sana istenilen nokta ile arandaki mesafeyi veriyor

            x_dist = -self.current_position_x + threat.current_position_x
            y_dist = -self.current_position_y + threat.current_position_y
            dist = math.sqrt((x_dist ** 2) + (y_dist ** 2))
            width = threat.width
            height = threat.height

            if threat.two_dim == True:
                if x_dist == 0:
                    r = threat.heigth / 2
                elif y_dist == 0:
                    r = width / 2
                elif abs(y_dist) < abs(x_dist):
                    a = (width / 2) * (abs(y_dist) / abs(x_dist))
                    r = math.sqrt(a ** 2 + (width / 2) ** 2)
                elif abs(y_dist) > abs(x_dist):
                    a = (height / 2) * (abs(x_dist) / abs(y_dist))
                    r = math.sqrt(a ** 2 + (height / 2) ** 2)
            else:
                r = threat.radius

            if proportional == True:
                safe_dist = r * kd
            else:
                safe_dist = kd

            dist_vec = [x_dist, y_dist]
            unit_vec = np.array(dist_vec) / dist
            dist_vec = dist_vec - (unit_vec * r)
            dist_vec.tolist()

            dist_mag = math.sqrt(dist_vec[0] ** 2 + dist_vec[1] ** 2)
            if dist_mag <= 0:
                v_repp_vir_x = -x_dist * 15
                v_repp_vir_y = -y_dist * 15

            elif dist_mag <= safe_dist:
                v_repp_x = (
                        -kr * (1 - (dist_mag / safe_dist)) *
                        (dist_vec[0] / (dist_mag ** 3))
                )
                v_repp_y = (
                        -kr * (1 - (dist_mag / safe_dist)) *
                        (dist_vec[1] / (dist_mag ** 3))
                )
                v_repp_vir_x = v_repp_x - (
                        kv
                        * (
                                dist_vec[0]
                                / math.sqrt((dist_vec[0] ** 2 + dist_vec[1] ** 2) ** 3)
                        )
                )
                v_repp_vir_y = v_repp_y - (
                        kv
                        * (
                                dist_vec[1]
                                / math.sqrt((dist_vec[0] ** 2 + dist_vec[1] ** 2) ** 3)
                        )
                )
                vec = np.array([v_repp_vir_x, v_repp_vir_y])
                v_repp = v_repp + vec
            elif dist_mag > safe_dist:
                pass
        v_repp.tolist()
        return v_repp


    def transform_from_euler(
            self, x, y, z, roll, pitch, yaw, header_frame_id, child_frame_id
    ):
        """
        Creates a new stamped transform from translation and euler-orientation

        :param x: x translation
        :param y: y translation
        :param z: z translation
        :param roll: orientation roll
        :param pitch: orientation pitch
        :param yaw: orientation yaw
        :param header_frame_id: transform from this frame
        :param child_frame_id: transform to this frame

        :returns: transform
        """
        t = TransformStamped()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = tf.transformations.quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.header.frame_id = header_frame_id
        t.child_frame_id = child_frame_id

        return t

    """
    FORMASYON FONKSIYONLARI
    """

    def land_single(self, kp=5.01):
        # haraketi uygula

        # iniş için koruyacağı noktayı belirle
        if self.first_run_land:
            self.land_pos = [self.assigned_fp[0], self.assigned_fp[1]]
            self.first_run_land = False
        else:
            pass

            # indir
        try:

            vx = (self.land_pos[0] - self.current_position_x) * kp
            vy = (self.land_pos[1] - self.current_position_y) * kp
            vz = (0 - self.current_position_z) * kp

            v = [vx, vy, vz]

            self.velocity_control_land(v)

        except ZeroDivisionError:
            pass

        # haraketi kontrol et
        b = 1

        cond = self.current_position_z <= 0.2  # 0 veya 1 dönüyor

        if cond:

            if self.link_uri == 0:
                pass
            else:
                self.command.send_stop_setpoint()
            return True
        else:  # b == 0
            return False



class Obstacle:
    def __init__(self, current_position_x=0, current_position_y=0, obs_type=0.07, obstacle_name=0):
        self.team_name = "zerowing"
        if type(obstacle_name) is str:
            #rospy.Subscriber(f"/obstacle/obstacle_{obstacle_name}", obstacle, self.callback)

            rospy.Subscriber(f"/general_parameters/{self.team_name}_{obstacle_name}", general_parameters, self.callback)

        self.current_position_x = current_position_x
        self.current_position_y = current_position_y
        if type(obs_type) is list:
            self.width = obs_type[0]  # x ekseninde uzunluk
            self.height = obs_type[1]  # y ekseninde uzunluk
            self.two_dim = True
        if type(obs_type) is float:
            self.radius = obs_type
            self.two_dim = False


    def callback(self, data):
        self.current_position_x = data.pose.x
        self.current_position_y = data.pose.y


