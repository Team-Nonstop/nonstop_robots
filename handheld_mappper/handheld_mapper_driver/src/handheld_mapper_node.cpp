#include "handheld_mapper_driver/handheld_mapper_driver.h"
#include "handheld_mapper_driver/string_common.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>          // odom
#include "sensor_msgs/Imu.h"

#define TIMEOUT 1000
#define DEFAULT_BAUDRATE 115200
//#define _DEBUG_


// This example opens the serial port and sends a request 'R' at 1Hz and waits for a reply.
EBIMU9DOF imu_device;
ros::Publisher * imu_pub_ptr;
ros::Publisher * odom_pub_ptr;
tf::TransformBroadcaster * odom_broadcaster_ptr;

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
    geometry_msgs::Quaternion odom_quat;
    
    //Tokenize
    std::string text = trim_left(trim_right(*data));
    split( v, text, IMU_RESP_SEP );
    //

    double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    double orientation_covariance = orientation_stdev_ * orientation_stdev_;
    double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;

    sensor_msgs::Imu imu;    
    imu.header.frame_id = frameid_;
    imu.header.stamp = ros::Time::now();


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
    odom_quat.x = atof(v[0].c_str());
    odom_quat.y = atof(v[1].c_str());
    odom_quat.z = atof(v[2].c_str());
    odom_quat.w = atof(v[3].c_str());

    imu.orientation = odom_quat;
    imu.linear_acceleration.x = atof(v[4].c_str());
    imu.linear_acceleration.y = atof(v[5].c_str());
    imu.linear_acceleration.z = atof(v[6].c_str());
}else{
    ROS_INFO("vSize Error [%d]",(int)v.size());
    imu_device.accelero.setVisible(ON);    
    imu_device.setOutputForm(OUTPUT_FORM_QUARER);
}


imu_pub_ptr->publish(imu);









    // First, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
        
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
        
    // Send the transform
    odom_broadcaster_ptr->sendTransform(odom_trans);        // odom->base_link transform
        
        
    // Next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
        
    // Set the position
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;
        
    // Set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
        
odom.pose.covariance[0]  = 0.01;
odom.pose.covariance[7]  = 0.01;
odom.pose.covariance[14] = 99999;
odom.pose.covariance[21] = 99999;
odom.pose.covariance[28] = 99999;
odom.pose.covariance[35] = 0.01;

odom.twist.covariance = odom.pose.covariance;
    // Publish the message
    odom_pub_ptr->publish(odom);        // odometry publisher 






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
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(frameid_, 100);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    tf::TransformBroadcaster odom_broadcaster;
    imu_pub_ptr = &imu_pub;
    odom_pub_ptr = &odom_pub;
    odom_broadcaster_ptr = &odom_broadcaster;

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