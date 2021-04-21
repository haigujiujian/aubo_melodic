#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    std::cout << "test motion" << std::endl;

    ros::init(argc, argv, "arm_motion");
    ros::NodeHandle nh;

    ros::Publisher arm_motion_pub = nh.advertise<std_msgs::String>("/aubo_ros_driver/movej", 1);

    bool start = false;
    ros::param::get("global_arm_motion", start);

    tf::TransformListener tflistener_;
    tf::StampedTransform Livox_to_Odom_;

    ros::Rate loop_sleep(0.1);

    bool step1 = false;
    bool step2 = false;
    bool step3 = false;

    while (ros::ok() && start)
    {
        try{
            tflistener_.lookupTransform("/odom", "/livox_link",
                                        ros::Time(0), Livox_to_Odom_);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            // ros::Duration(1.0).sleep();
            continue;
        }
        
        Eigen::Quaternionf quat_eigen(Livox_to_Odom_.getRotation().w(),
                                        Livox_to_Odom_.getRotation().x(),
                                        Livox_to_Odom_.getRotation().y(),
                                        Livox_to_Odom_.getRotation().z());

        Eigen::Vector3f eulerAngle = quat_eigen.matrix().eulerAngles(2,1,0);
        
        float angle = eulerAngle[2]* 180.0 / 3.1415926;

        std_msgs::String msg;
        std::stringstream ss;

        if(!step1 && (angle == 174.5))
        {
            ROS_INFO("step1");
            ss << "movej{joint=[(174.5, 0.0, 60.0, 0.0, 0.0, 0.0, 0.0)],paramlist=(0.3,0.3,1.0)}";
            msg.data = ss.str();
            arm_motion_pub.publish(msg);
            step1 = true;
        }

        if(step1 && !step2 && (angle == -174.5))
        {
            ROS_INFO("step2");
            ss << "movej{joint=[(-174.5, 0.0, 60.0, 0.0, 0.0, 0.0, 0.0)],paramlist=(0.3,0.3,1.0)}";
            msg.data = ss.str();
            arm_motion_pub.publish(msg);
            step2 = true;
        }

        if(step1 && step2 && !step3 && (angle == 0))
        {
            ROS_INFO("step3");
            ss << "movej{joint=[(0, 0.0, 60.0, 0.0, 0.0, 0.0, 0.0)],paramlist=(0.3,0.3,1.0)}";
            msg.data = ss.str();
            arm_motion_pub.publish(msg);
            step3 = true;
        }

        ros::param::set("global_arm_motion_done", true);


        loop_sleep.sleep();
        ros::spinOnce();
    }
    

}