#include <ros/ros.h>
#include <std_msgs/String.h>
#include "diablo_sdk/OSDK_ACCL.h"
#include "diablo_sdk/OSDK_GYRO.h"
#include "diablo_sdk/OSDK_LEGMOTORS.h"
#include "diablo_sdk/OSDK_POWER.h"
#include "diablo_sdk/OSDK_QUATERNION.h"
#include "diablo_sdk/OSDK_STATUS.h"
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;
DIABLO::OSDK::Telemetry* pTelemetry;
ros::Publisher ACCLPublisher;
ros::Publisher GYROPublisher;
ros::Publisher LEGMOTORSPublisher;
ros::Publisher POWERPublisher;
ros::Publisher QUATERNIONPublisher;
ros::Publisher STATUSPublisher;

void status_callback(const ros::TimerEvent& e)
{
  if(pTelemetry->newcome & 0x40)
    {
      diablo_sdk::OSDK_STATUS msg;
      msg.ctrl_mode = pTelemetry->status.ctrl_mode;
      msg.robot_mode = pTelemetry->status.robot_mode;
      msg.error = pTelemetry->status.error;
      msg.warning = pTelemetry->status.warning;
      STATUSPublisher.publish(msg);
      pTelemetry->eraseNewcomeFlag(0xBF);
    }
  if(pTelemetry->newcome & 0x20)
    {
      diablo_sdk::OSDK_QUATERNION msg;
      msg.w = pTelemetry->quaternion.w;
      msg.x = pTelemetry->quaternion.x;
      msg.y = pTelemetry->quaternion.y;
      msg.z = pTelemetry->quaternion.z;
      QUATERNIONPublisher.publish(msg);
      pTelemetry->eraseNewcomeFlag(0xDF);
    }
  if(pTelemetry->newcome & 0x10)
    {
      diablo_sdk::OSDK_ACCL msg;
      msg.x = pTelemetry-> accl.x;
      msg.y = pTelemetry-> accl.y;
      msg.z = pTelemetry-> accl.z;
      ACCLPublisher.publish(msg);
      pTelemetry->eraseNewcomeFlag(0xEF);
    }
  if(pTelemetry->newcome & 0x08)
    {
      diablo_sdk::OSDK_GYRO msg;
      msg.x = pTelemetry->gyro.x;
      msg.y = pTelemetry->gyro.y;
      msg.z = pTelemetry->gyro.z;
      GYROPublisher.publish(msg);
      pTelemetry->eraseNewcomeFlag(0xF7);
    }
  if(pTelemetry->newcome & 0x02)
    {
      diablo_sdk::OSDK_POWER msg;
      msg.battery_voltage = pTelemetry->power.voltage;
      msg.battery_current = pTelemetry->power.current;
      msg.battery_capacitor_energy = pTelemetry->power.capacitor_energy;
      msg.battery_power_percent = pTelemetry->power.power_percent;
      POWERPublisher.publish(msg);
      pTelemetry->eraseNewcomeFlag(0xFD);
    }
  if(pTelemetry->newcome & 0x01)
    {
      diablo_sdk::OSDK_LEGMOTORS msg;
      msg.left_hip_enc_rev = pTelemetry->motors.left_hip.rev;
      msg.left_hip_pos = pTelemetry->motors.left_hip.pos;
      msg.left_hip_vel = pTelemetry->motors.left_hip.vel;
      msg.left_hip_iq = pTelemetry->motors.left_hip.iq;

      msg.left_knee_enc_rev = pTelemetry->motors.left_knee.rev;
      msg.left_knee_pos = pTelemetry->motors.left_knee.pos;
      msg.left_knee_vel = pTelemetry->motors.left_knee.vel;
      msg.left_knee_iq = pTelemetry->motors.left_knee.iq;

      msg.left_wheel_enc_rev = pTelemetry->motors.left_wheel.rev;
      msg.left_wheel_pos = pTelemetry->motors.left_wheel.pos;
      msg.left_wheel_vel = pTelemetry->motors.left_wheel.vel;
      msg.left_wheel_iq = pTelemetry->motors.left_wheel.iq;

      msg.right_hip_enc_rev = pTelemetry->motors.right_hip.rev;
      msg.right_hip_pos = pTelemetry->motors.right_hip.pos;
      msg.right_hip_vel = pTelemetry->motors.right_hip.vel;
      msg.right_hip_iq = pTelemetry->motors.right_hip.iq;

      msg.right_knee_enc_rev = pTelemetry->motors.right_knee.rev;
      msg.right_knee_pos = pTelemetry->motors.right_knee.pos;
      msg.right_knee_vel = pTelemetry->motors.right_knee.vel;
      msg.right_knee_iq =  pTelemetry->motors.right_knee.iq;

      msg.right_wheel_enc_rev = pTelemetry->motors.right_wheel.rev;
      msg.right_wheel_pos = pTelemetry->motors.right_wheel.pos;
      msg.right_wheel_vel = pTelemetry->motors.right_wheel.vel;
      msg.right_wheel_iq = pTelemetry->motors.right_wheel.iq;

      LEGMOTORSPublisher.publish(msg);
      pTelemetry->eraseNewcomeFlag(0xFE);
    }
}

void ctrl_callback(const ros::TimerEvent& e)
{
  if(!pMovementCtrl->in_control())
    {
      pMovementCtrl->obtain_control();
      return;
    }

  if(pMovementCtrl->ctrl_mode_cmd)
    {
      uint8_t result = pMovementCtrl->SendMovementModeCtrlCmd();
    }
  else
    {
      uint8_t result = pMovementCtrl->SendMovementCtrlCmd();
    }
}

void teleop_ctrl(const std_msgs::String::ConstPtr& msg)
{
    if(!pMovementCtrl->in_control())
    {
        printf("to get movement ctrl.\n");
        uint8_t result = pMovementCtrl->obtain_control();
        return;
    }
    if(pMovementCtrl->ctrl_mode_data.height_ctrl_mode == 1)
        pMovementCtrl->ctrl_data.up=0.0f;
    pMovementCtrl->ctrl_data.forward = 0.0f;
    pMovementCtrl->ctrl_data.left = 0.0f;

    for(const char& c : msg->data)
    {
        switch(c)
        {
            case 'w':
                pMovementCtrl->ctrl_data.forward = 1.0f;                // vel ctrl
                break;
            case 'a':
                pMovementCtrl->ctrl_data.left = 1.0f;                   // angular_vel ctrl
                break;
            case 's':
                pMovementCtrl->ctrl_data.forward = -1.0f;               // vel ctrl
                break;
            case 'd':
                pMovementCtrl->ctrl_data.left = -1.0f;                  // angular_vel ctrl
                break;
            case 'q':
                pMovementCtrl->ctrl_data.roll = -0.1f;                  // pos ctrl
                break;
            case 'e':
                pMovementCtrl->ctrl_data.roll = 0.1f;                   // pos ctrl
                break;
            case 'r':
                pMovementCtrl->ctrl_data.roll = 0.0f;                   // pos ctrl
                break;
            case 'z':
                pMovementCtrl->SendTransformDownCmd();
                return;
                break;
            case 'v':
                pMovementCtrl->SendTransformUpCmd();
                return;
                break;
            case 'n':
                pMovementCtrl->ctrl_mode_data.height_ctrl_mode = 0;      // vel ctrl mode 
                pMovementCtrl->ctrl_mode_cmd = true;
                break;
            case 'm':
                pMovementCtrl->ctrl_mode_data.height_ctrl_mode = 1;      // pos ctrl mode
                pMovementCtrl->ctrl_mode_cmd = true;
                break;
            case 'f':
                pMovementCtrl->ctrl_data.up = 0.0f;                     //pos & angular_vel ctrl
                break;
            case 'g':
                pMovementCtrl->ctrl_data.up = 0.5f;                     //pos ctrl
                break;
            case 'h':
                pMovementCtrl->ctrl_data.up = 1.0f;                     //pos ctrl
                break;
            case 'x':
                pMovementCtrl->ctrl_data.up = -0.1f;                    // vel ctrl
                break;
            case 'c':
                pMovementCtrl->ctrl_data.up = 0.1f;                     // vel ctrl
                break;
            case 'y':
                pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = 0;      // angular_vel ctrl mode
                pMovementCtrl->ctrl_mode_cmd = true;
                break;
            case 'u':
                pMovementCtrl->ctrl_mode_data.pitch_ctrl_mode = 1;      // pos ctrl mode
                pMovementCtrl->ctrl_mode_cmd = true;
                break;
            case 'i':
                pMovementCtrl->ctrl_data.pitch = -0.5f;                 // pos & angular_vel ctrl
                break;
            case 'o':
                pMovementCtrl->ctrl_data.pitch = 0.0f;                  // pos & angular_vel ctrl
                break;
            case 'p':
                pMovementCtrl->ctrl_data.pitch = 0.5f;                  // pos & angular_vel ctrl
                break;
            default:
                break;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "movement_ctrl_example");
    ros::NodeHandle nh("~");

    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init("/dev/ttyAMA0")) return -1;

    DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
    if(vehicle.init()) return -1;

    pTelemetry = vehicle.telemetry;
    vehicle.telemetry->activate();
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, OSDK_PUSH_DATA_1Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_50Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_ACCL, OSDK_PUSH_DATA_50Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_GYRO, OSDK_PUSH_DATA_50Hz);
    vehicle.telemetry->configTopic(DIABLO::OSDK::TOPIC_MOTOR, OSDK_PUSH_DATA_10Hz);
    vehicle.telemetry->configUpdate();

    pMovementCtrl = vehicle.movement_ctrl;
    ros::Subscriber sub = nh.subscribe("/DJ_teleop", 1, teleop_ctrl); //subscribe to ROS topic
    ACCLPublisher = nh.advertise<diablo_sdk::OSDK_ACCL>("diablo_ros_ACCL_b", 10);
    GYROPublisher = nh.advertise<diablo_sdk::OSDK_GYRO>("diablo_ros_GYRO_b", 10);
    LEGMOTORSPublisher = nh.advertise<diablo_sdk::OSDK_LEGMOTORS>("diablo_ros_LEGMOTORS_b", 10);
    POWERPublisher = nh.advertise<diablo_sdk::OSDK_POWER>("diablo_ros_POWER_b", 10);
    QUATERNIONPublisher = nh.advertise<diablo_sdk::OSDK_QUATERNION>("diablo_ros_QUATERNION_b", 10);
    STATUSPublisher = nh.advertise<diablo_sdk::OSDK_STATUS>("diablo_ros_STATUS_b", 10);


    ros::Timer ctrl_timer = nh.createTimer(ros::Duration(0.1), ctrl_callback);
    ros::Timer status_timer = nh.createTimer(ros::Duration(0.01), status_callback);

    ros::spin();

    return 0;
}
