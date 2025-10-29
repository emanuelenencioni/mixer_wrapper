#include <cstdio>
#include <ext_dep.h>
#include <internalTime.h>
#include <thread_functions.h>
#include <Com.h>
#include <typeDef.h>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include "std_msgs/msg/bool.hpp"
#include "mixer_msgs/msg/barometer.hpp"
#include "mixer_msgs/msg/magnetometer.hpp"
#include "mixer_msgs/msg/gps.hpp"
#include "mixer_msgs/msg/rtk.hpp"
#include "mixer_msgs/msg/imu.hpp"

#define RANGE 180
#define RESIZE 90

using namespace std::chrono_literals;

class Mixer_comm : public rclcpp::Node{
public:
  Mixer_comm() : Node("MIXER_WRAPPER"){
  //member msgs, must be created before timers and callbacks	  
  m_pBaro_msg = new mixer_msgs::msg::Barometer();
  m_pMagn_msg = new mixer_msgs::msg::Magnetometer();
  m_pGps_msg = new mixer_msgs::msg::Gps();
  m_pRtk_msg = new mixer_msgs::msg::Rtk();
  m_pImu_msg = new mixer_msgs::msg::Imu();
  
  // Declare parameters for calibration
  this->declare_parameter<bool>("enable_calibration", false);
  this->declare_parameter<double>("calibration_duration_sec", 1.0);
  this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
  
  // Set parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Mixer_comm::parametersCallback, this, std::placeholders::_1));
  
  // Get initial parameter values
  portName = this->get_parameter("serial_port").as_string();
  g_dCalibrationDurationSec = this->get_parameter("calibration_duration_sec").as_double();
  	  
  timer_ = this->create_wall_timer(10ms, std::bind(&Mixer_comm::timer_callback, this));
  baro_pub = this->create_publisher<mixer_msgs::msg::Barometer>("mixer/telemetry/barometer", 10);
  magn_pub = this->create_publisher<mixer_msgs::msg::Magnetometer>("mixer/telemetry/magnetometer", 10);
  gps_pub = this->create_publisher<mixer_msgs::msg::Gps>("mixer/telemetry/gps", 10);
  rtk_pub = this->create_publisher<mixer_msgs::msg::Rtk>("mixer/telemetry/rtk", 10);
  imu_pub = this->create_publisher<mixer_msgs::msg::Imu>("mixer/telemetry/imu", 10);
  
  // Subscriber (manteniamo per retrocompatibilità)
  calibration_sub = this->create_subscription<std_msgs::msg::Bool>("/mixer/enable_calibration", 10, std::bind(&Mixer_comm::calibrationCallback_, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Mixer wrapper node started OK!");
  RCLCPP_INFO(this->get_logger(), "Serial port: %s", portName.c_str());
  }

private:

  // Parameter callback for dynamic reconfigure
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    
    for (const auto &param : parameters) {
      if (param.get_name() == "enable_calibration") {
        if (param.as_bool()) {
          g_bStartCalibration = true;
          RCLCPP_INFO(this->get_logger(), "Calibration enabled via parameter");
        }
      }
      else if (param.get_name() == "calibration_duration_sec") {
        g_dCalibrationDurationSec = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Calibration duration set to: %.2f sec", g_dCalibrationDurationSec);
      }
      else if (param.get_name() == "serial_port") {
        // Serial port change requires node restart
        RCLCPP_WARN(this->get_logger(), "Serial port change requires node restart to take effect");
        result.successful = false;
        result.reason = "Serial port change requires node restart";
      }
    }
    
    return result;
  }

  void calibrationCallback_(const std_msgs::msg::Bool::SharedPtr /*msg*/)
  {

      g_bStartCalibration = true;    
      RCLCPP_INFO(this->get_logger(), "START MIXER CALIBRATION ROUTINE ...");
  }

  rclcpp::Publisher<mixer_msgs::msg::Barometer>::SharedPtr baro_pub;
  rclcpp::Publisher<mixer_msgs::msg::Magnetometer>::SharedPtr magn_pub;
  rclcpp::Publisher<mixer_msgs::msg::Gps>::SharedPtr gps_pub;
  rclcpp::Publisher<mixer_msgs::msg::Rtk>::SharedPtr rtk_pub;
  rclcpp::Publisher<mixer_msgs::msg::Imu>::SharedPtr imu_pub;
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibration_sub;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  mixer_msgs::msg::Barometer *m_pBaro_msg;
  mixer_msgs::msg::Magnetometer *m_pMagn_msg;
  mixer_msgs::msg::Gps *m_pGps_msg;
  mixer_msgs::msg::Rtk *m_pRtk_msg;
  mixer_msgs::msg::Imu *m_pImu_msg;

  std::string portName = "/dev/ttyTHS1";

  bool bCommunicationThread = false;
  float fCommunicationHz = 100; //Hz
  double dSampleTime;
  RX_Frame RxFrameShared;
  double l_dSampleTime = 0;
  commandsLl g_sDataToSend;
  internalTime *sampleTime;
  bool bTimer_init = false;
  Com *c;
  double dSteeringSetPoint = 0;
  double  dThrottleSetPoint = 0;

  //std::chrono::steady_clock::time_point g_dLinearActuatorStartTime;
  std::chrono::steady_clock::time_point g_dCalibrationStartTime;
  //bool g_bStartLinearActuator = false;  
  bool g_bStartCalibration = false; 
  //double g_dLinearActuatorDurationSec = 1.0;
  double g_dCalibrationDurationSec = 1.0;   

void timer_callback(){
      
  auto time = now();
    
  if(!bTimer_init){
    sampleTime = new internalTime();
    c = new Com(portName);  
    bTimer_init = true;
  }
    
  //!******** delta time *********!//
  l_dSampleTime = sampleTime->getSampleTime()/1000;
              
  //!*********Read State***********// 
  RXData l_RxFrame = c->uartComRx();
  
  ///   ROUTINE DI CALIBRAZIONE ****************
  //!****************************!//
  /*if(g_bStartLinearActuator){ 
    g_dLinearActuatorStartTime = std::chrono::steady_clock::now();
    ROS_INFO("Send Command /LINEAR ACTUATOR/: START UART TX for %.1f s", g_dLinearActuatorDurationSec);		
    g_sDataToSend.COM_2 = 100*RESIZE;
    g_bStartLinearActuator = false;	
  }*/
    
  if(g_bStartCalibration){
    g_dCalibrationStartTime = std::chrono::steady_clock::now();     
    RCLCPP_INFO(this->get_logger(), "Send Command /CALIBRATION/: START UART TX for %.1f s", g_dCalibrationDurationSec);
    g_sDataToSend.COM_0 = 100*RESIZE;
    g_bStartCalibration = false;	
  }        
    
  auto now = std::chrono::steady_clock::now();
  //double l_dLA_Elapsed = std::chrono::duration<double>(now - g_dLinearActuatorStartTime).count();
  double l_dCalElapsed = std::chrono::duration<double>(now - g_dCalibrationStartTime).count();
  /*
  if(l_dLA_Elapsed < g_dLinearActuatorDurationSec){
    //!********Send Commands To MIXER********!//
    int *l_piPpmSend = c->uartComTx(0, 170, 1810, RANGE, false, true, g_sDataToSend); //aggiunto il controllo di consistenza sui dati
    //!**************************************!//
  }
  else{
    g_sDataToSend.COM_2 = 0;
    //!********Send Commands To MIXER********!//
    int *l_piPpmSend = c->uartComTx(0, 170, 1810, RANGE, false, true, g_sDataToSend); //aggiunto il controllo di consistenza sui dati
    //!**************************************!//
    //ROS_INFO("Command /LINEAR ACTUATOR/ STOP UART TX");
  }*/
    
  if(l_dCalElapsed < g_dCalibrationDurationSec){
    //!********Send Commands To MIXER********!//
    c->uartComTx(0, 170, 1810, RANGE, false, true, g_sDataToSend); //aggiunto il controllo di consistenza sui dati
    //!**************************************!//
  }
  else{
    g_sDataToSend.COM_0 = 0;			
    //!********Send Commands To MIXER********!//
    c->uartComTx(0, 170, 1810, RANGE, false, true, g_sDataToSend); //aggiunto il controllo di consistenza sui dati
    //!**************************************!//
    //ROS_INFO("Command /CALIBRATION/ STOP UART TX");
  }
  ///   ROUTINE DI CALIBRAZIONE ****************

  m_pBaro_msg->header.stamp  = time;
  m_pGps_msg->header.stamp  = time;
  m_pImu_msg->header.stamp  = time;
  
  if(l_RxFrame.GPS_SYSTEM_ERROR == 1)
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "GPS SYSTEM ERROR " << l_RxFrame.GPS_SYSTEM_ERROR);
  if((l_RxFrame.GPS_1_FIXMODE != 4) || (((double)(l_RxFrame.ACCN))/100 > 5.0f) || (((double)(l_RxFrame.ACCE))/100 > 5.0f))
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS1] FIX MODE: " << l_RxFrame.GPS_1_FIXMODE);
    if(((double)(l_RxFrame.ACCN))/100 == 0)
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS1] ACCN: NA");
    else
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS1] ACCN: " << ((double)(l_RxFrame.ACCN))/100 << " cm");
    if(((double)(l_RxFrame.ACCE))/100 == 0)
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS1] ACCE: NA");
    else
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS1] ACCE: " << ((double)(l_RxFrame.ACCE))/100 << " cm");
  }
  else
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS1] FIX 4");

  if((l_RxFrame.GPS_2_FIXMODE) != 4 || (((double)(l_RxFrame.ACCN))/100 > 5.0f) || (((double)(l_RxFrame.ACCE))/100 > 5.0f))
  {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS2] FIX MODE: " << l_RxFrame.GPS_2_FIXMODE);
    if(((double)(l_RxFrame.ACCN2))/100 == 0)
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS2] ACCN: NA");
    else
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS2] ACCN: " << ((double)(l_RxFrame.ACCN2))/100 << " cm");
    if(((double)(l_RxFrame.ACCE))/100 == 0)
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS2] ACCE: NA");
    else
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS2] ACCE: " << ((double)(l_RxFrame.ACCE2))/100 << " cm");      
  }
  else
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[GPS2] FIX 4");

  m_pImu_msg->quat.w = (double)(l_RxFrame.Q0_TMP);
  m_pImu_msg->quat.x = (double)(l_RxFrame.Q1_TMP);
  m_pImu_msg->quat.y = (double)(l_RxFrame.Q2_TMP);
  m_pImu_msg->quat.z = (double)(l_RxFrame.Q3_TMP);
  attitude l_Euler = quaternionToEuler(l_RxFrame.Q0, l_RxFrame.Q1, l_RxFrame.Q2, l_RxFrame.Q3);
  m_pImu_msg->pitch.data = (double)((180/M_PI)*l_Euler.PITCH);
  m_pImu_msg->roll.data = (double)((180/M_PI)*l_Euler.ROLL);
  m_pImu_msg->yaw.data = (double)((180/M_PI)*l_Euler.YAW);	 
  m_pImu_msg->acc.x = (double)(l_RxFrame.AX);
  m_pImu_msg->acc.y = (double)(l_RxFrame.AY);
  m_pImu_msg->acc.z = (double)(l_RxFrame.AZ);
  m_pImu_msg->angularvelocity.x = (double)(l_RxFrame.GX);
  m_pImu_msg->angularvelocity.y = (double)(l_RxFrame.GY);
  m_pImu_msg->angularvelocity.z = (double)(l_RxFrame.GZ);
  m_pImu_msg->delta_time.data = l_RxFrame.DELTA_TIME_IMU_MS;	 
  
  m_pGps_msg->tow.data = l_RxFrame.TIME_OF_WEEK; 
  m_pGps_msg->gpslatitude.data = ((double)(l_RxFrame.GPS_LATITUDE))/10000000 + ((double)(l_RxFrame.GPS_LATITUDE_HP))/1000000000;
  m_pGps_msg->gpslongitude.data = ((double)(l_RxFrame.GPS_LONGITUDE))/10000000 + ((double)(l_RxFrame.GPS_LONGITUDE_HP))/1000000000;
  m_pGps_msg->gpsaltitude.data = l_RxFrame.GPS_ALTITUDE; 
  m_pGps_msg->eph.data = ((float)(l_RxFrame.GPS_EPH))/10; //cm; 
  m_pGps_msg->epv.data = ((float)(l_RxFrame.GPS_EPV))/10; //cm  
  m_pGps_msg->system_error.data = l_RxFrame.GPS_SYSTEM_ERROR;
  m_pGps_msg->gps_fix_1.data = l_RxFrame.GPS_1_FIXMODE;
  m_pGps_msg->gps_fix_2.data = l_RxFrame.GPS_2_FIXMODE;

  m_pRtk_msg->tow.data = l_RxFrame.TIME_OF_WEEK; 
  m_pRtk_msg->relposn.data = ((double)(l_RxFrame.RELPOSN))/100;//cm
  m_pRtk_msg->relpose.data = ((double)(l_RxFrame.RELPOSE))/100;//cm
  m_pRtk_msg->relposd.data = ((double)(l_RxFrame.RELPOSD))/100;//cm
  m_pRtk_msg->relposn2.data = ((double)(l_RxFrame.RELPOSN2))/100;//cm
  m_pRtk_msg->relpose2.data = ((double)(l_RxFrame.RELPOSE2))/100;//cm
  m_pRtk_msg->relposd2.data = ((double)(l_RxFrame.RELPOSD2))/100;//cm
  m_pRtk_msg->accn.data = ((double)(l_RxFrame.ACCN))/100;//cm
  m_pRtk_msg->acce.data = ((double)(l_RxFrame.ACCE))/100;//cm
  m_pRtk_msg->accd.data = ((double)(l_RxFrame.ACCD))/100;//cm
  m_pRtk_msg->accn2.data = ((double)(l_RxFrame.ACCN2))/100;//cm
  m_pRtk_msg->acce2.data = ((double)(l_RxFrame.ACCE2))/100;//cm
  m_pRtk_msg->accd2.data = ((double)(l_RxFrame.ACCD2))/100;//cm
  m_pRtk_msg->veln.data = l_RxFrame.VELN;
  m_pRtk_msg->vele.data = l_RxFrame.VELE;
  m_pRtk_msg->veld.data = l_RxFrame.VELD;
  m_pRtk_msg->velacc.data = l_RxFrame.VELACC;
  m_pRtk_msg->veln2.data = l_RxFrame.VELN2;
  m_pRtk_msg->vele2.data = l_RxFrame.VELE2;
  m_pRtk_msg->veld2.data = l_RxFrame.VELD2;
  m_pRtk_msg->velacc2.data = l_RxFrame.VELACC2;
  m_pRtk_msg->system_error.data = l_RxFrame.GPS_SYSTEM_ERROR;
  m_pRtk_msg->gps_fix_1.data = l_RxFrame.GPS_1_FIXMODE;
  m_pRtk_msg->gps_fix_2.data = l_RxFrame.GPS_2_FIXMODE;
  
  m_pBaro_msg->baroaltitude.data = l_RxFrame.BARO_ALTITUDE;
  m_pBaro_msg->baropressure.data = l_RxFrame.BARO_PRESSURE;
  m_pBaro_msg->barotemp.data = l_RxFrame.BARO_TEMP;
  m_pBaro_msg->delta_time.data = l_RxFrame.DELTA_TIME_BARO_MS;	 
  
  m_pMagn_msg->mx.data = l_RxFrame.MX;
  m_pMagn_msg->my.data = l_RxFrame.MY;
  m_pMagn_msg->mz.data = l_RxFrame.MZ;

  baro_pub->publish(*m_pBaro_msg);
  magn_pub->publish(*m_pMagn_msg);
  gps_pub->publish(*m_pGps_msg);
  rtk_pub->publish(*m_pRtk_msg);
  imu_pub->publish(*m_pImu_msg);

}

attitude quaternionToEuler(float l_fQ0, float l_fQ1, float l_fQ2, float l_fQ3){
  attitude l_Euler;
  l_Euler.PITCH = (double)(asin(2*(l_fQ0*l_fQ2 - l_fQ1*l_fQ3)));
  l_Euler.ROLL = (double)(-atan(2*(l_fQ0*l_fQ1 + l_fQ2*l_fQ3)/(l_fQ0*l_fQ0 - l_fQ1*l_fQ1 - l_fQ2*l_fQ2 + l_fQ3*l_fQ3)));
  l_Euler.YAW = (double)(atan(2*(l_fQ0*l_fQ3 + l_fQ1*l_fQ2)/(l_fQ0*l_fQ0 + l_fQ1*l_fQ1 - l_fQ2*l_fQ2 - l_fQ3*l_fQ3)));
  return l_Euler;
}  
  
};

int main(int argc, const char **argv){	
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mixer_comm>());
  rclcpp::shutdown();
  return 0;

} 


