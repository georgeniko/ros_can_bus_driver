#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>
#include "rclcpp/time_source.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "turtle_actuate/msg/actuation.hpp"
#include "turtle_interfaces/msg/mission.hpp"
#include "turtle_actuate/msg/lap_counter.hpp"
#include "turtle_actuate/msg/cones_count.hpp"

#include <iostream>
using namespace std;
using std::placeholders::_1;

/////////////////////////
//CAN MESSAGES ID DEFINITION
//CAN MESSAGES Hz is determined by the publisher's timer on its callback function
//reminder that DL DUTY CYCLE 100ms
/////////////////////////////

#define ACTUATE_ID 0x1F8
#define MISSION_ID 0x208
#define LAP_COUNTER 0x005
#define CONES_COUNT 0x1F9


union trans
{
  uint8_t canMes;    //Function for message packing
  int8_t scanMes; //signed can message
  int16_t val;
}transformer;

class Ros2Can : public rclcpp::Node
{
  public:
    Ros2Can()
    : Node("canbus_listener")
    {
      subscription_actuation = this->create_subscription<turtle_actuate::msg::Actuation>(  //Subscriber Declaration
      "Actuation", 10, std::bind(&Ros2Can::actuation_topic_callback,this,  _1));

      subscription_mission = this->create_subscription<turtle_interfaces::msg::Mission>(  //Subscriber Declaration
      "Mission", 10, std::bind(&Ros2Can::mission_topic_callback,this,  _1));

      subscription_lap_counter = this->create_subscription<turtle_actuate::msg::LapCounter>(  //Subscriber Declaration
      "LapCounter", 10, std::bind(&Ros2Can::lap_counter_topic_callback,this,  _1));

      subscription_cones_counter = this->create_subscription<turtle_actuate::msg::ConesCount>(  //Subscriber Declaration
      "ConesCounter", 10, std::bind(&Ros2Can::cones_counter_topic_callback,this,  _1));

      strcpy(ifr.ifr_name, "vcan0" );         //CAN Socket Initialization  //MUST CAHNGE TO CAN0 FOR PROPER USE
      set_s( socket(PF_CAN, SOCK_RAW, CAN_RAW));
      ioctl(s, SIOCGIFINDEX, &ifr);
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      bind(get_s(), (struct sockaddr *)&addr, sizeof(addr));

      actuate.can_id=ACTUATE_ID;
      actuate.can_dlc=6;
      actuate.data[0]=0x00; actuate.data[1]=0x00;
      actuate.data[2]=0x00; actuate.data[3]=0x00;
      actuate.data[4]=0x00; actuate.data[5]=0x00;

      mission.can_id=MISSION_ID;
      mission.can_dlc=1;
      mission.data[0]=0x00;

      lap_can.can_id=LAP_COUNTER;
      lap_can.can_dlc=1;
      lap_can.data[0]=0x00;

      cones.can_id=CONES_COUNT;
      cones.can_dlc=2;
      cones.data[0]=0x00;
      cones.data[1]=0x00;

    }
    void set_s(int a)    //Setters and getters because callback function is a diva. Not used functions are kept for debugging purposes
    {
      this->s = a;
    }
    int get_s()   //USED
    {
      return this->s;
    }
    can_frame get_actuation_frame()  //NOT USED
    {
      return this->actuate;
    }
    void set_can_data_actuate(int8_t *d, int starting_b) //USED
    {
      this->actuate.data[starting_b]=d[1];
      this->actuate.data[starting_b+1]=d[0];
    }
    void set_can_data_mission(uint8_t *d) // USED
    {
      this->mission.data[0]=d[0];
    }
    void set_can_data_lap(uint8_t *d) // USED
    {
      this->lap_can.data[0]=d[0];
    }
    void set_can_data_cones(uint8_t *d, int starting_b)
    {
      this->cones.data[starting_b]=d[1];
      this->cones.data[starting_b+1]=d[0];
    }
    void set_actuation_id() //NOT USED
    {
      this->actuate.can_id= ACTUATE_ID;
    }
    can_frame* get_actuation_frame_address()   //NOT USED
    {
      return &(this->actuate);
    }
    can_frame* get_can_frame_address_mission() //USED
    {
      return &(this->mission);
    }
    can_frame* get_can_frame_address_lap() //USED
    {
      return &(this->lap_can);
    }
    can_frame* get_can_frame_address_cones() //USED
    {
      return &(this->cones);
    }
    void set_mission(int a) //USED
    {
      this->mis=a;
    }
    void set_lap(int a) //USED
    {
      this->lap=a;
    }
    void set_steering_command(float a)//USED
    {
      this->steering_command=a;
    }
    void set_braking_command(float b)//USED
    {
      this->braking_command=b;
    }
    void set_throttle_command(float c)//USED
    {
      this->throttle_command=c;
    }
    void set_cones_count_actuall(float c)//USED
    {
      this->cones_count_actuall=c;
    }
    void set_cones_count_all(float c)//USED
    {
      this->cones_count_all=c;
    }
    void set_datas(uint8_t *data_matrix)//USED
    {

      this->datas= data_matrix;

    }
    void set_datas_signed(int8_t *data_matrix)//USED
    {

      this->datas_signed= data_matrix;

    }
    void  clear_datas()
    {
      this->datas= NULL;
    }

    void actuation_topic_callback(const turtle_actuate::msg::Actuation::SharedPtr msg)
    {
      set_steering_command(msg->target_steering_angle);
      set_braking_command(msg->target_brake);
      set_throttle_command(msg->target_throttle);

      transformer.val =steering_command/0.001;
      set_datas_signed(&transformer.scanMes);
      set_can_data_actuate(datas_signed,0);

      transformer.val =braking_command/0.001;
      set_datas_signed(&transformer.scanMes);
      set_can_data_actuate(datas_signed,2);

      transformer.val =throttle_command/0.001;
      set_datas_signed(&transformer.scanMes);
      set_can_data_actuate(datas_signed,4);
      //printf("data:: %d %d, %d %d,%d %d\n",actuate.data[0],actuate.data[1],actuate.data[2],actuate.data[3],actuate.data[4],actuate.data[5]);
      write(get_s(), get_actuation_frame_address(), sizeof(struct can_frame));
    }
    void mission_topic_callback(const turtle_interfaces::msg::Mission::SharedPtr mis_msg)
    {
      set_mission(mis_msg->mission);
      set_datas(&mis);
      set_can_data_mission(datas);
      write(get_s(), get_can_frame_address_mission(), sizeof(struct can_frame));
    }
    void lap_counter_topic_callback(const turtle_actuate::msg::LapCounter::SharedPtr lap_msg)
    {
      set_lap(lap_msg->lapcounter);
      set_datas(&lap);
      set_can_data_lap(datas);
      write(get_s(), get_can_frame_address_lap(), sizeof(struct can_frame));
      //clear_datas();
    }
    void cones_counter_topic_callback(const turtle_actuate::msg::ConesCount::SharedPtr cones_msg)
    {
      set_cones_count_actuall(cones_msg->actualcount);
      set_cones_count_all(cones_msg->allcount);

      set_datas(&cones_count_actuall);
      set_can_data_cones(datas,0);

      set_datas(&cones_count_all);
      set_can_data_cones(datas,2);
      //printf("all: %d . actual: %d\n",cones_count_all,cones_count_actuall);
      write(get_s(), get_can_frame_address_cones(), sizeof(struct can_frame));
      //clear_datas();
    }
    rclcpp::Subscription<turtle_actuate::msg::Actuation>::SharedPtr subscription_actuation;
    rclcpp::Subscription<turtle_interfaces::msg::Mission>::SharedPtr subscription_mission;
    rclcpp::Subscription<turtle_actuate::msg::ConesCount>::SharedPtr subscription_cones_counter;
    rclcpp::Subscription<turtle_actuate::msg::LapCounter>::SharedPtr subscription_lap_counter;

  private:    //Variables Decleration

      struct sockaddr_can addr;
      struct ifreq ifr;
      int s;
      uint8_t mis;
      uint8_t lap;
      uint8_t cones_count_actuall;
      uint8_t cones_count_all;

      uint8_t *datas;
      int8_t *datas_signed;
      float steering_command;
      float braking_command;
      float throttle_command;
      struct can_frame actuate;
      struct can_frame mission;
      struct can_frame lap_can;
      struct can_frame cones;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //Start the node and wait for messages through callbacks
  rclcpp::spin(std::make_shared<Ros2Can>());
  rclcpp::shutdown();
  return 0;
}
