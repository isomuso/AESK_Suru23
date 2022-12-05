#include "ros/ros.h"
#include <sstream>

#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"


class Drone{
   //class dışından erişilemiyor ama içeriden istenildiği gibi kullanılabilir
  private:

    //class attributeleri
    float dt {0.1};
    float pos_x, pos_y, pos_z;
    ros::Publisher AHRS_pub;
    ros::Subscriber position_sub;


  public:
  
  //constructor
    Drone(ros::NodeHandle *nh, float Pos_x, float Pos_y, float Pos_z){
      pos_x = Pos_x;
      pos_y = Pos_y;
      pos_z = Pos_z;

      AHRS_pub = nh->advertise<std_msgs::Float32MultiArray>("/get_position", 10);
      position_sub = nh->subscribe("/move_drone", 1000, &Drone::speed_controller, this);

    }

    void speed_controller(const std_msgs::Float32MultiArray &speed_vec){
      /*
      pos_vec:: std double vec [x, y, z]
      */
      std::cout << speed_vec.data[0] << std::endl;
      pos_x += speed_vec.data[0] * dt;
      pos_y += speed_vec.data[1] * dt;
      pos_z += speed_vec.data[2] * dt;
    }

    void AHRS(){
      std_msgs::Float32MultiArray msg;
      std::vector<float> pos_vec {pos_x, pos_y, pos_z};
      msg.data = pos_vec;
      AHRS_pub.publish(msg); 
      
    }
  
};

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int main(int argc, char **argv)
{
  //init fonksiyonu
  ros::init(argc, argv, "sim_envionment");

  ros::NodeHandle n;

  //loopu belli bir frekansta döndürmek için.
  ros::Rate loop_rate(10);

  //clası initialize et.
  Drone crazyflie(&n, 0.0, 0.0, 0.0);

  while (ros::ok())
  {
    //pozisyon publishle
    crazyflie.AHRS();

    //bu olmadan callbackler çalışmıyor
    ros::spinOnce();

    //belli bir hertzde döndürmek için
    loop_rate.sleep();

  }


  return 0;
}
