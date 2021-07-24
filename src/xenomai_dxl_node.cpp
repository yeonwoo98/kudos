#include "ros/ros.h"
#include "std_msgs/String.h"


// **********Basic libraries*********//
#include <sys/mman.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h>
#include <inttypes.h>

// **********ROS libraries*********//

// **********Xenomai libraries*********//
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>

#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
unsigned int cycle_ns = 1000000;

//Xenomai time variablse
RT_TASK RT_task1;
RT_TASK RT_task2;
RT_TASK RT_task3;
RTIME now1, previous1; // Ethercat time
RTIME now2, previous2; // Thread 1 cycle time
RTIME now3, previous3; // Thread 2 cycle time
double thread_time0 = 0.0; // Ethercat time
double thread_time1 = 0.0; // Thread 1 cycle time
double thread_time2 = 0.0; // Thread 2 cycle time
double max_time = 0.0;


//////////////////////////////////dxl sdk///////////////////////


#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk/dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b


// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

int index1 = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
int dxl_goal_position[2] = {1024, 3072};         // Goal position

double a ;
double b ;

uint8_t dxl_error = 0;                          // Dynamixel error
int32_t dxl_present_position = 0;               // Present position


void ros_task(void* arg);


///////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
  ros::init(argc, argv, "xenomai_test_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  ////////////////////////////////////////////////////


    // Open port
    if (portHandler->openPort())
    {
      printf("Succeeded to open the port!\n");
    }
    else
    {
      printf("Failed to open the port!\n");
      printf("Press any key to terminate...\n");
    //  getch();
      return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate!\n");
    }
    else
    {
      printf("Failed to change the baudrate!\n");
      printf("Press any key to terminate...\n");
     // getch();
      return 0;
    }

    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    else
    {
      printf("Dynamixel has been successfully connected \n");

    }

    rt_task_create(&RT_task1, "ros_task", 0, 90, 0);

    rt_task_start(&RT_task1, &ros_task, NULL);


  while (ros::ok())
  {
    /*

    std_msgs::String msg;
    msg.data = "i said hello";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ROS_INFO("hello");
    */
  }

}


void ros_task(void* arg)
{
  rt_task_set_periodic(NULL, TM_NOW, cycle_ns * 10);



  while (1) {
    rt_task_wait_period(NULL);


    /////////////////////////////////////////////////
/*
    std_msgs::String msg;
    msg.data = "send_data";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
*/
    ///////////////////////////////////////////////////
/*
    printf("Press any key to continue! (or press ESC to quit!)\n");
       if (getch() == ESC_ASCII_VALUE)
         break;
*/

       // Write goal position
       dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index1], &dxl_error);
       if (dxl_comm_result != COMM_SUCCESS)
       {
         packetHandler->getTxRxResult(dxl_comm_result);
       }
       else if (dxl_error != 0)
       {
         packetHandler->getRxPacketError(dxl_error);
       }

        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
       {
         packetHandler->getTxRxResult(dxl_comm_result);
       }
       else if (dxl_error != 0)
       {
         packetHandler->getRxPacketError(dxl_error);
       }

       do
       {
         // Read present position
         dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
         if (dxl_comm_result != COMM_SUCCESS)
         {
           packetHandler->getTxRxResult(dxl_comm_result);
         }
         else if (dxl_error != 0)
         {
           packetHandler->getRxPacketError(dxl_error);
         }

         printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index1], dxl_present_position);

       }while((abs(dxl_goal_position[index1] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));


        // Change goal position

       if ((abs(dxl_goal_position[index1] - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD))
       {
          if (index1 == 0)
          {
             index1 = 1;
          }
          else
          {
            index1 = 0;
          }
       }

   // ROSMsgPublish();
  }

  ////////////////////////////////////////

  // Disable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }


    // Close port
    portHandler->closePort();

    return ;

  ///////////////////////////////////////

  return ;
}

