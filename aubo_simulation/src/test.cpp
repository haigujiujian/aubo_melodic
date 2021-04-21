#include <iostream>
#include<string>
#include <ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<robot_state_publisher/robot_state_publisher.h>
#include <eigen3/Eigen/Dense>
#include"kinematics.h"
#include <visualization_msgs/Marker.h>

using   namespace std;

sensor_msgs::JointState joint_state;
ros::Publisher  joint_pub;

int  moveJ(double  j1,double j2,double j3,double j4,double j5 ,double j6);
int DisplayPoint(double x,double y,double z);

int main(int argc ,char** argv)
{
    Eigen::MatrixXd TcpCenter(4,4);
    Eigen::VectorXd  arcAng(6);
    visualization_msgs::Marker points;
     geometry_msgs::Point p;
    double pointX,pointY,pointZ;
    ros::init(argc,argv,"aubo_joint_publisher");
    ros::NodeHandle n;
    joint_pub=n.advertise<sensor_msgs::JointState>("joint_states",1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate loop_rate(10);
     int command;
     
    char Menu[256]="\n0. Exit. \n1. Control joints to move. \n Please input your command";
    double  j1,j2,j3,j4,j5,j6;

    while(ros::ok())
    {
        printf("%s",Menu);
        scanf("%d",&command);
        switch(command)
        {
                case 0:
                                exit(1);
                                break;
                case 1:
                                printf("please input 6 joint angle: \n");
                                scanf("%lf %lf %lf %lf  %lf %lf",&j1,&j2,&j3,&j4,&j5,&j6);
                                arcAng(0)=j1*M_PI/180.0;
                                arcAng(1)=j2*M_PI/180.0;
                                arcAng(2)=j3*M_PI/180.0;
                                arcAng(3)=j4*M_PI/180.0;
                                arcAng(4)=j5*M_PI/180.0;
                                arcAng(5)=j6*M_PI/180.0;
                           
                                aubo_forward(TcpCenter,arcAng);
                                moveJ(arcAng(0),arcAng(1),arcAng(2),arcAng(3),arcAng(4),arcAng(5));
                                cout<<TcpCenter<<endl;
                                pointX=TcpCenter(0,3);
                                pointY=TcpCenter(1,3);
                                pointZ=TcpCenter(2,3);
                                printf("%lf %lf  %lf  \n",pointX,pointY,pointZ);
                                points.header.frame_id="myframe";
                                points.id=0;
                                points.header.stamp = ros::Time::now();
                                points.ns="point";
                                points.type = visualization_msgs::Marker::POINTS;
                                //points.action = visualization_msgs::Marker::DELETE;//delete not work
                                points.points.clear();//work to clear
                                //marker_pub.publish(points);
                                points.action = visualization_msgs::Marker::ADD;
                                points.lifetime=ros::Duration();
                                points.scale.x = 0.2;
                                points.scale.y = 0.2;
                                points.scale.z = 0.2;
                                points.color.r = 1.0f;
                                points.color.g = 1.0f;
                                points.color.b = 0.0f;
                                points.color.a = 1.0;

                                p.x=pointX;
                                p.y=pointY;
                                p.z=pointZ;
                                points.header.stamp=ros::Time::now();
                                points.points.push_back(p);
                                //发布 point marker
                                marker_pub.publish(points);
                                cout<<" points have been sent"<<endl;
                    default:
                                printf("no this command");
                                break;
         }
         ros::spinOnce();
         loop_rate.sleep();


    }
    return 0;
}

int moveJ(double j1,double j2,double j3,double j4,double j5,double j6)
{
    joint_state.header.stamp=ros::Time::now();
    joint_state.header.frame_id="";
    joint_state.name.resize(6);
    joint_state.position.resize(6);

    joint_state.name[0]="shoulder_joint";
    joint_state.position[0]=j1;
    
    joint_state.name[1]="upperArm_joint";
    joint_state.position[1]=j2;

    joint_state.name[2]="foreArm_joint";
    joint_state.position[2]=j3;

    joint_state.name[3]="wrist1_joint";
    joint_state.position[3]=j4;

    joint_state.name[4]="wrist2_joint";
    joint_state.position[4]=j5;

    joint_state.name[5]="wrist3_joint";
    joint_state.position[5]=j6;

    joint_state.velocity.resize(6);
    joint_state.effort.resize(6);

    joint_pub.publish(joint_state);
    return 0;

}
