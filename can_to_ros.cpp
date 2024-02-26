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
#include <sys/time.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "turtle_interfaces/msg/ekf_euler.hpp"
#include "turtle_interfaces/msg/ekf_vel.hpp"
#include "turtle_interfaces/msg/ekf_pos.hpp"
#include "turtle_interfaces/msg/accelerometer.hpp"
#include "turtle_interfaces/msg/gyro.hpp"
#include "turtle_interfaces/msg/ek_feul_acc.hpp"
#include "turtle_interfaces/msg/ek_fvel_acc.hpp"
#include "turtle_interfaces/msg/ek_fpos_acc.hpp"
#include "turtle_interfaces/msg/im_uinfo.hpp"
#include "turtle_interfaces/msg/ek_finfo.hpp"
#include "turtle_interfaces/msg/mission.hpp"
#include "turtle_interfaces/msg/steering_actual.hpp"
#include "turtle_interfaces/msg/brake_actual.hpp"
#include "turtle_interfaces/msg/throttle_actual.hpp"
#include "turtle_interfaces/msg/eb_sstate.hpp"
#include "turtle_interfaces/msg/re_scommunication.hpp"
#include "turtle_interfaces/msg/steering_state.hpp"
#include "turtle_interfaces/msg/service_brake_state.hpp"



//#include <iostream>
//using namespace std;
/////////////////////////
//CAN MESSAGES ID DEFINITION

#define IMU_ACCEL 0x1F8
#define IMU_GYRO 0x1F9
#define UTC0 0x1F7
#define EKF_POS 0x200
#define EKF_EULER 0x1FA
#define EKF_VEL_BODY 0x204 //ACTUALLY VEL NED
#define IMU_INFO 0x1FF
#define EKF_INFO 0x035
#define EKF_ORIENTATION_ACC 0x20B
#define EKF_VEL_NED_ACC  0x205
#define EKF_POS_ACC 0x203
#define STEERING_ACTUAL 0x304//float
#define BRAKE_ACTUAL 0x206//float
#define THROTTLE_ACTUAL 0x207//float
#define MISSION_ID 0x208//int
#define EBS_STATE 0x209//int
#define RES_COMMUNICATION 0x210// bool
#define STEERING_STATE 0x211// int
#define SERVICE_BRAKE_STATE 0x212// int
/////////////////////////


// MESSAGES UNPACKING FUNCTIONS & STRUCTS
void print_frame(struct can_frame* f);
float unpack_16(uint8_t data[8],int starting_byte, float factor);
float unpack_32(uint8_t data[8],int starting_byte, float factor);
uint32_t unpack_time_stamp(uint8_t data[8],int starting_byte);
uint32_t imu_time=0;
uint32_t ekf_time=0;
union trans
{
	uint8_t canMes[2];    //FOR SIGNED DATA THAT COME IN 16b packet
	int16_t val;
}transformer;

union trans2
{
	uint8_t canMes2[4];    //FOR SINGED THAT COME IN 32b packet
	int32_t val2;
}transformer2;

union trans3
{
	uint8_t canMes3[4];    //FOR UNSIGNED DATA THAT COME IN 32b packet
	uint32_t val3; //or 16?
}transformer3;

union trans4
{
	uint8_t canMes4[2];    //FOR UNSIGNED DATA THAT COME IN 16b packet
	uint16_t  val4;
}transformer4;

union trans5
{
	uint8_t canMes5[2];    //FOR UNSIGNED DATA THAT COME IN 16b packet
	bool  val5;
}transformer5;



float unpack_16( uint8_t data[8],int starting_byte,float factor)
{
	transformer.canMes[1] = data[starting_byte+1];
	transformer.canMes[0] = data[starting_byte];
	return (transformer.val*factor);
}

float unpack_32(uint8_t data[8],int starting_byte, float factor)
{
	transformer2.canMes2[3] =data[starting_byte+3];
	transformer2.canMes2[2] =data[starting_byte+2];
	transformer2.canMes2[1] = data[starting_byte+1];
	transformer2.canMes2[0] = data[starting_byte];
	return (transformer2.val2*factor);
}
bool unpack_bool (uint8_t data[8],int starting_byte)
{
  transformer5.canMes5[0]=data[starting_byte];
	transformer5.canMes5[1]=data[starting_byte+1];
	return transformer5.val5;
}
uint32_t unpack_time_stamp(uint8_t data[8],int starting_byte)
{

	transformer3.canMes3[3] =data[starting_byte+3];
	transformer3.canMes3[2] =data[starting_byte+2];
	transformer3.canMes3[1] =data[starting_byte+1];
	transformer3.canMes3[0] =data[starting_byte+0];
	return transformer3.val3;
}

/////////////////////////

int main(int argc, char * argv[])
{
   //CAN Socket Initialization
   struct can_frame frame;
   struct sockaddr_can addr;
   struct ifreq ifr;
   // struct timeval tv;//for timestamp
   auto s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
   int nbytes;
	 int ioctl_test=0;
	 int bind_test=0;
   strcpy(ifr.ifr_name, "vcan0" ); //	********	MUST SWITCH TO THE ACTUAL CHANNEL BEFORE USING	********'
	 ioctl_test= ioctl(s, SIOCGIFINDEX, &ifr);
   while (ioctl_test==-1)
	 {
		// printf("Error ioctl. Trying to reset ioctl...\r");
		 ioctl_test=ioctl(s, SIOCGIFINDEX, &ifr);
	 }

   //socklen_t len = sizeof(addr); // Only used by the blocking function
   addr.can_family = AF_CAN;
   addr.can_ifindex = ifr.ifr_ifindex;
	 bind_test=bind(s, (struct sockaddr *)&addr, sizeof(addr));
   //if (bind_test==-1){perror("Error bind\n");}
	 while (bind_test==-1)
	 {
		// printf("Error bind. Trying to reset bind...\r");
		 bind_test=bind(s, (struct sockaddr *)&addr, sizeof(addr));
	 }
	 if (ioctl_test&&bind_test!=-1)
	 {
		 printf("Fatal Error Opening Socket\n");
		 return 1;
	 }
	 else
	 {

		 printf("Socket Opened Succesfully!\n");
	 }
	//struct can_filter rfilter; example filter for CAN IDs 1 - 30
	//rfilter.can_id   = 0x001;
	//rfilter.can_mask = 0x7E0;
	//setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

	//ROS2 Publishers Initialization
	rclcpp::init(argc, argv);
        auto canbus_node =  rclcpp::Node::make_shared("canbus");
	auto pub_ekf_euler= canbus_node-> create_publisher<turtle_interfaces::msg::EKFEuler>("EKFEuler", 10); //topic names inside ""
	auto pub_ekf_vel= canbus_node->create_publisher<turtle_interfaces::msg::EKFVel>("EKFVel", 10);
	auto pub_ekf_pos= canbus_node->create_publisher<turtle_interfaces::msg::EKFPos>("EKFPos", 10);
	auto pub_accel= canbus_node-> create_publisher<turtle_interfaces::msg::Accelerometer>("Accelerometer", 10);
	auto pub_gyro= canbus_node-> create_publisher<turtle_interfaces::msg::Gyro>("Gyro", 10);
	auto pub_ekf_euler_acc= canbus_node-> create_publisher<turtle_interfaces::msg::EKFeulAcc>("EulerAcc", 10);
	auto pub_ekf_vel_acc= canbus_node-> create_publisher<turtle_interfaces::msg::EKFvelAcc>("VelAcc", 10);
	auto pub_ekf_pos_acc= canbus_node-> create_publisher<turtle_interfaces::msg::EKFposAcc>("PosAcc", 10);
	auto pub_imu_info= canbus_node-> create_publisher<turtle_interfaces::msg::IMUinfo>("IMUInfo", 10);
	auto pub_ekf_info= canbus_node-> create_publisher<turtle_interfaces::msg::EKFinfo>("EKFInfo", 10);

	auto pub_steer_act= canbus_node-> create_publisher<turtle_interfaces::msg::SteeringActual>("SteeringActual", 10);
	auto pub_brake_act= canbus_node-> create_publisher<turtle_interfaces::msg::BrakeActual>("BrakeActual", 10);
	auto pub_throttle_act= canbus_node-> create_publisher<turtle_interfaces::msg::ThrottleActual>("ThrottleActual", 10);
	auto pub_mission= canbus_node-> create_publisher<turtle_interfaces::msg::Mission>("Mission", 10);
	auto pub_ebs_state= canbus_node-> create_publisher<turtle_interfaces::msg::EBSstate>("EBSstate", 10);
	auto pub_res_communication= canbus_node-> create_publisher<turtle_interfaces::msg::REScommunication>("REScommunication", 10);
	auto pub_steering_state= canbus_node-> create_publisher<turtle_interfaces::msg::SteeringState>("SteeringState", 10);
	auto pub_service_brake_state= canbus_node-> create_publisher<turtle_interfaces::msg::ServiceBrakeState>("ServiceBrakeState", 10);

	//READING OF CAN FRAMES
	do{
		//nbytes = recvfrom(s, &frame, sizeof(struct can_frame),MSG_DONTWAIT, (struct sockaddr*)&addr, &len);// Non-blocking
		nbytes = read(s,&frame,sizeof(struct can_frame)); //BLOCKING READING

	    if ((nbytes < 0)||(nbytes < (int)sizeof(struct can_frame)) ) //Error Checking
		{
	 		//	printf("nbytes<0 or nbytes < sizeof(struct can_frame)\r");
      perror("Error: Paranoid Check");
		  // return 1;
	   }
	    else
		{
			switch(frame.can_id)
			{
				case IMU_ACCEL:
					{
						turtle_interfaces::msg::Accelerometer acc_msg;
						acc_msg.x = unpack_16(frame.data,0,0.01);
						acc_msg.y = unpack_16(frame.data,2,0.01);
						acc_msg.z = unpack_16(frame.data,4,0.01);
						acc_msg.header.stamp.nanosec=imu_time;
						pub_accel->publish(acc_msg);
						break;
					}
				case IMU_GYRO:
					{
						turtle_interfaces::msg::Gyro gyro_msg;
						gyro_msg.v_roll = unpack_16(frame.data,0,0.001);
						gyro_msg.v_pitch = unpack_16(frame.data,2,0.001);
						gyro_msg.v_yaw = unpack_16(frame.data,4,0.001);
						gyro_msg.header.stamp.nanosec=imu_time;
						pub_gyro->publish(gyro_msg);
						break;
					}
				case  EKF_POS:
					{
						turtle_interfaces::msg::EKFPos pos_msg;
						pos_msg.x=unpack_32(frame.data,0,1e-007);
						pos_msg.y=unpack_32(frame.data,4,1e-007);
						pos_msg.header.stamp.nanosec=ekf_time;
						pub_ekf_pos->publish(pos_msg);
					        break;
					}
				case  EKF_EULER:
					{
						turtle_interfaces::msg::EKFEuler euler_msg;
						euler_msg.roll = unpack_16(frame.data,0,0.0001);
						euler_msg.pitch= unpack_16(frame.data,2,0.0001);
						euler_msg.yaw=unpack_16(frame.data,4,0.0001);
						euler_msg.header.stamp.nanosec=ekf_time;
						pub_ekf_euler->publish(euler_msg);
					        break;
					}
				case EKF_VEL_BODY:
					{
						turtle_interfaces::msg::EKFVel vel_msg;
						vel_msg.v_x =unpack_16(frame.data,0,0.01);
						vel_msg.v_y= unpack_16(frame.data,2,0.01);
						vel_msg.v_z=unpack_16(frame.data,4,0.01);
						vel_msg.header.stamp.nanosec=ekf_time;
						pub_ekf_vel->publish(vel_msg);
					        break;
					}

				case  IMU_INFO:
					{
						turtle_interfaces::msg::IMUinfo imu_info_msg;
						imu_info_msg.time_stamp=unpack_time_stamp(frame.data,0);
						imu_info_msg.temperature=unpack_16(frame.data,6,0.01);
						transformer4.canMes4[0] =frame.data[4];
						transformer4.canMes4[1] =frame.data[5];
						imu_info_msg.status = transformer4.val4;
						imu_time =(imu_info_msg.time_stamp)*1000; //micro to nanosec
						//printf("IMU (STATUS,TS,TEMP): %hu %u %f\n",STATUS,TIME_STAMP,TEMPERATURE);
						pub_imu_info->publish(imu_info_msg);
					        break;
					}
				case EKF_INFO:
					{
						turtle_interfaces::msg::EKFinfo ekf_info_msg;
						ekf_info_msg.time_stamp=unpack_time_stamp(frame.data,0);
						ekf_time = (ekf_info_msg.time_stamp)*1000;//micro to nanosec
						pub_ekf_info->publish(ekf_info_msg);
						break;
					}
				case EKF_ORIENTATION_ACC:
					{
						turtle_interfaces::msg::EKFeulAcc euler_acc_msg;
						euler_acc_msg.ekf_eul_acc_roll=unpack_16(frame.data,0,0.0001);
						euler_acc_msg.ekf_eul_acc_pitch=unpack_16(frame.data,2,0.0001);
						euler_acc_msg.ekf_eul_acc_yaw=unpack_16(frame.data,4,0.0001);
						euler_acc_msg.header.stamp.nanosec=ekf_time;
						pub_ekf_euler_acc->publish(euler_acc_msg);
						break;
					}
				case EKF_VEL_NED_ACC:
					{
						turtle_interfaces::msg::EKFvelAcc vel_acc_msg;
						vel_acc_msg.ekf_vel_acc_x=unpack_16(frame.data,0,0.01);
						vel_acc_msg.ekf_vel_acc_y=unpack_16(frame.data,2,0.01);
						vel_acc_msg.ekf_vel_acc_z=unpack_16(frame.data,4,0.01);
						vel_acc_msg.header.stamp.nanosec=ekf_time;
						pub_ekf_vel_acc->publish(vel_acc_msg);
						break;
					}
				case EKF_POS_ACC:
					{
						turtle_interfaces::msg::EKFposAcc pos_acc_msg;
						pos_acc_msg.ekf_pos_acc_x=unpack_16(frame.data,0,0.01);
						pos_acc_msg.ekf_pos_acc_y=unpack_16(frame.data,2,0.01);
						pos_acc_msg.ekf_pos_acc_z=unpack_16(frame.data,4,0.01);
						pos_acc_msg.header.stamp.nanosec=ekf_time;
						pub_ekf_pos_acc->publish(pos_acc_msg);
						break;
					}
				case STEERING_ACTUAL:
					{
						turtle_interfaces::msg::SteeringActual steer_act_msg;
						steer_act_msg.position=unpack_16(frame.data,0,0.01);
						pub_steer_act->publish(steer_act_msg);
						break;
					}
				case BRAKE_ACTUAL:
					{
						turtle_interfaces::msg::BrakeActual brake_act_msg;
						brake_act_msg.position=unpack_16(frame.data,0,0.01);
						pub_brake_act->publish(brake_act_msg);
						break;
					}
				case THROTTLE_ACTUAL:
					{
						turtle_interfaces::msg::ThrottleActual throttle_act_msg;
						throttle_act_msg.position=unpack_16(frame.data,0,0.01);
						pub_throttle_act->publish(throttle_act_msg);
						break;
					}
				case MISSION_ID:
					{
						turtle_interfaces::msg::Mission mission_msg;
						mission_msg.mission=unpack_16(frame.data,0,1);
						pub_mission->publish(mission_msg);
						break;
					}
				case EBS_STATE:
					{
						turtle_interfaces::msg::EBSstate ebs_state_msg;
						ebs_state_msg.state=unpack_16(frame.data,0,1);
						pub_ebs_state->publish(ebs_state_msg);
						break;
					}
				case RES_COMMUNICATION:
					{
						turtle_interfaces::msg::REScommunication res_msg;
						res_msg.go=unpack_bool(frame.data,0);
						//res_msg.go=frame.data[0];
						res_msg.red_button=frame.data[1];
						res_msg.estop1=frame.data[2];
						res_msg.estop2=frame.data[3];
						res_msg.radio_quality=unpack_16(frame.data,4,1);
						res_msg.pre_alarm=frame.data[6];
						pub_res_communication->publish(res_msg);
						break;
					}
				case STEERING_STATE:
					{
						turtle_interfaces::msg::SteeringState steering_state_msg;
						steering_state_msg.state=unpack_16(frame.data,0,1);
						pub_steering_state->publish(steering_state_msg);
						break;
					}
				case SERVICE_BRAKE_STATE:
					{
						turtle_interfaces::msg::ServiceBrakeState service_brake_state_msg;
						service_brake_state_msg.state=unpack_16(frame.data,0,1);
						pub_service_brake_state->publish(service_brake_state_msg);
						break;
					}
				default:
						//printf("CAN-ID %u not recognized \n",f->can_id);
						{break;}
			}
		}
	 //rclcpp::spin(canbus_node);
	 }while(rclcpp::ok());
	close(s);

return 0;
}




void print_frame(struct can_frame* f) //Printing function for debugging
{
		printf("0x%03X [%d] ",f->can_id, f->can_dlc);
		for (int i = 0; i < f->can_dlc; i++)
 			printf("%02X ",f->data[i]);
		printf("\n");
}
