#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <Arduino_LSM6DS3.h>
#include <std_msgs/Header.h>

float g = -9.80665;
char imu_topic[] = "imu/raw";
float wx, wy, wz;
float ax, ay, az;
float orientation_covariance[9] = {-1,-1,-1,-1,-1,-1,-1,-1,-1};
float angular_veloity_covariance[9] = {1,0,0,0,1,0,0,0,1};
float linear_acceleration_covariance[9] = {1,0,0,0,1,0,0,0,1};
  
ros::NodeHandle nh;          

sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu(imu_topic, &imu_msg);

void setup_timer()
{
  noInterrupts();
  // for UNO WIFI Rev 2
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CCMP = 25000;
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
  interrupts();
}

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub_imu);
  IMU.begin();
  setup_timer();
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}

ISR(TCB0_INT_vect)
{
  if (IMU.accelerationAvailable()){
    IMU.readGyroscope(wx, wy, wz);
    IMU.readAcceleration(ax, ay, az);
    }
  else{
    wx = 0;
    wy = 0;
    wz = 0;
    ax = 0;
    ay = 0;
    az = 0;
    }

  std_msgs::Header header_msg; 
  header_msg.stamp = nh.now();
  header_msg.frame_id = "imu";
  imu_msg.header = header_msg;

  memcpy(imu_msg.orientation_covariance,orientation_covariance,sizeof(orientation_covariance));
  
  imu_msg.angular_velocity.x = wx/180.0*PI;
  imu_msg.angular_velocity.y = wy/180.0*PI;
  imu_msg.angular_velocity.z = wz/180.0*PI;
  memcpy(imu_msg.angular_velocity_covariance,angular_veloity_covariance,sizeof(angular_veloity_covariance));
  
  imu_msg.linear_acceleration.x = ax*g;
  imu_msg.linear_acceleration.y = ay*g;
  imu_msg.linear_acceleration.z = az*g;
  memcpy(imu_msg.linear_acceleration_covariance,linear_acceleration_covariance,sizeof(linear_acceleration_covariance));
  
  pub_imu.publish(&imu_msg);
  TCB0.INTFLAGS = TCB_CAPT_bm;
}
