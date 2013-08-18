#include "handheld_mapper_driver/handheld_mapper_driver.h"
#include "handheld_mapper_driver/string_common.h"

#include "sensor_msgs/Imu.h"

#define TIMEOUT 1000
#define DEFAULT_BAUDRATE 115200
//#define _DEBUG_


// This example opens the serial port and sends a request 'R' at 1Hz and waits for a reply.
EBIMU9DOF imu_device;
ros::Publisher * imu_pub_ptr;
sensor_msgs::Imu imu;
std::string frameid_;

double linear_acceleration_stdev_ = 0.098; 
double orientation_stdev_= 0.035; 
double angular_velocity_stdev_= 0.012; 


//Receive encoder ticks and send 'odom' and 'tf'
void robotDataCallback(std::string * data){ 
#ifdef _DEBUG_  
   ROS_INFO("Get:\"%s\"",data->c_str());
#endif

    std::vector<std::string> v;

    //Tokenize
    std::string text = trim_left(trim_right(*data));
    split( v, text, IMU_RESP_SEP );
    //

    double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    double orientation_covariance = orientation_stdev_ * orientation_stdev_;
    double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    
    imu.header.frame_id = frameid_;

    imu.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    imu.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    imu.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    imu.angular_velocity_covariance[0] = angular_velocity_covariance;
    imu.angular_velocity_covariance[4] = angular_velocity_covariance;
    imu.angular_velocity_covariance[8] = angular_velocity_covariance;
    
    imu.orientation_covariance[0] = orientation_covariance;
    imu.orientation_covariance[4] = orientation_covariance;
    imu.orientation_covariance[8] = orientation_covariance;

if(v.size()==7){
    imu.orientation.x = atof(v[0].c_str());
    imu.orientation.y = atof(v[1].c_str());
    imu.orientation.z = atof(v[2].c_str());
    imu.orientation.w = atof(v[3].c_str());

    imu.linear_acceleration.x = atof(v[4].c_str());
    imu.linear_acceleration.y = atof(v[5].c_str());
    imu.linear_acceleration.z = atof(v[6].c_str());
}else{
    ROS_INFO("vSize Error [%d]",(int)v.size());
    imu_device.accelero.setVisible(ON);    
    imu_device.setOutputForm(OUTPUT_FORM_QUARER);
}


imu_pub_ptr->publish(imu);
#ifdef _DEBUG_
    for (unsigned int i = 0;  i < v.size();   i++)
        ROS_INFO("Tokenize data[%d]:\"%s\"",i,v[i].c_str());
#endif
}



int main(int argc, char** argv)
{


    int baudrate = DEFAULT_BAUDRATE;
    std::string port_;

    //ROS INITIALIZE
    ros::init(argc, argv, "handheld_mapper_node");
    ros::NodeHandle nh;


        
    // ROS Parameter
    nh.param("frame_id", frameid_, std::string("imu"));
    nh.param("port", port_, std::string("/dev/ttyUSB0"));
    nh.param("linear_acceleration_stdev", linear_acceleration_stdev_, 0.098);
    nh.param("orientation_stdev", orientation_stdev_, 0.035);
    nh.param("angular_velocity_stdev", angular_velocity_stdev_, 0.012);

    // ROS publishers and subscribers
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(frameid_, 10);
    

    imu_pub_ptr = &imu_pub;

    //SERIAL INITIALIZE
    if (argc==2){
       baudrate = atoi(argv[1]);     
       ROS_INFO("Serial Baudrate defined to \"%d\" ", baudrate);
    }else if (argc==1){
       baudrate = DEFAULT_BAUDRATE;
       ROS_WARN("No Serial baudrate defined, defaulting to \"%s\" \"%d\" ",port_.c_str(),baudrate);
       ROS_WARN("Usage: \"rosrun [pkg] robot_node baudrate\"");
    }else{
       ROS_WARN("No Serial Baudrate defined, defaulting to \"%d\" ", baudrate);
       ROS_WARN("Usage: \"rosrun [pkg] robot_node baudrate\"");
    } 


    std::string reply;
    // Change the next line according to your port name and baud rate
    try{ device.open((char*)port_.c_str(), baudrate); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    ros::Duration(2.5).sleep();     
    ROS_INFO("The IMU serial port is opened.");
    

    ROS_INFO("The IMU is initializing"); 
    imu_device.initializeIMU();
    ROS_INFO("The IMU is initialized");

    try{ device.readLine(&reply, TIMEOUT); } 
    catch(cereal::TimeoutException& e){
      ROS_ERROR("Initial Read Timeout!");
    }    

    // Start receiving streaming data
    if( !device.startReadBetweenStream(boost::bind(&robotDataCallback, _1),IMU_RESP_BEGIN_CHAR,IMU_RESP_END_CHAR)){
          ROS_FATAL("Robot -- Failed to start streaming data!");
          ROS_BREAK();
    }

    ros::spin(); //trigger callbacks and prevents exiting
    return(0);
}