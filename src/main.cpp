// PATH FOLLOWING Tried On Lift
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h> 
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <random>
#include <stdio.h>
#include <time.h>
#include <fstream>

using namespace std;

// controller parameters
double dist_error = 0;
double angle_error = 0;
double dist_limit = 0.35;
double dist_stop = 0;
double max_v_local = 0;
int k_v = 0;

// velocities paramters
double v_cmd = 0;
double w_cmd = 0;
double v_robot = 0;
double w_robot = 0;

// path parameters
uint32_t size_path = 0 , size = 0;
double final_orientation = 0;

// odometry parametrs
double odom_x;
double odom_y;
double odom_theta;

int direction = 0;
double compensate_angle = 0;

// rate time parameter
double dt = 0.1;

// boolean paramters
bool final_theta_corrected = false;
bool path_stored = false;
bool path_changed = false;
bool robot_reached = false;
bool activate_cmd = false; 

// writing file paramter

// declare message parameters
tf::Point odom_pos;
visualization_msgs::Marker  robot_mk;
geometry_msgs::Point        robot_pt;
geometry_msgs::Twist        tw_msg;
geometry_msgs::Pose2D       qp[100] , new_qp[100], err;
std_msgs::String            status_msg;

// declare Publishers and Subscribers
ros::Publisher   cmd_pub;
ros::Publisher   robot_mk_pub;
ros::Publisher   status_control_pub;
ros::Subscriber  path_sub_1;
ros::Subscriber  path_sub_2;
ros::Subscriber  path_sub_3;
ros::Subscriber  gloab_local_sub;
ros::Subscriber  odom_sub;
ros::Subscriber  direction_sub;
ros::Subscriber  orient_sub , cmd_vel_mpc;

// declare topics
std::string path_topic1;
std::string path_topic2;
std::string path_topic3;
std::string globla_local_topic;
std::string direction_topic;
std::string orient_topic;
std::string cmd_topic;
std::string global_frame;
std::string base_frame;
std::string odometry_topic;
std::string status_topic;

// declare config parameters
double max_v;
double min_v;
double acc_v;
double max_w;
double acc_w;

// declare functions
// to check stop condition 
// will stop the robot when it is close to the goal
void stop_mode();
// calculate the errors between the robot and the path
void get_path_errors();
// to reset the paramters when there is no path or path completed
void reset_parameters();
// to rotate the robot at the last point to the desired orientation
void correction_final_theta();
// to initialize the markers
void initialize_markers();
// to publish three markers: robot position, desired point and
// the distance from robot to path
void publish_marker_odom();
// the path following algorithm
void path_following_controller();
// to define subscribers and publishers
void define_sub_pub(ros::NodeHandle n);
// to initialize config paramters
void initialize_parameters(ros::NodeHandle n);
// to get the orientation of yaw for the local frame and the last segment
double get_yaw(geometry_msgs::Quaternion q);
// to set saturation on the linear velocity of the robot
double sat_linear_velocity(double max , double min ,double accel, double v_ref , double old_velocity);

int ind_control = 0;

bool  flag_direction = true , flag_orientation = true , flag_global_map;

void path_back(const geometry_msgs::PoseArray msg) 
{
   activate_cmd = true;
   size = msg.poses.size();
   if(size!=0)
   {         
       if (size != size_path)
       {
           path_changed = true;
       }
       else
       {
           for (int j=0; j<size ; j++)
       {   
           path_changed = false;           
           new_qp[j].x=msg.poses[j].position.x;
           new_qp[j].y=msg.poses[j].position.y;
           if (abs(new_qp[j].x - qp[j].x)>0.1 || abs(new_qp[j].y - qp[j].y)>0.1)
           {
                path_changed = true;
                break;
           }
       }
       }
        if (path_changed)
       {
           ROS_INFO("New Path, it contains: %i points" , size);
           for (int j=0; j<size ; j++)
            { 
                qp[j].x=msg.poses[j].position.x;
                qp[j].y=msg.poses[j].position.y;
                ROS_INFO("x , y %f , %f" , qp[j].x , qp[j].y );
                final_orientation = get_yaw(msg.poses[size-1].orientation);
            }
       
        size_path = size;
        path_stored = true;
        robot_reached = false;
        k_v=0;
        get_path_errors();
        stop_mode();
        final_theta_corrected = false;
       }
 
   }
   else
   {
       path_stored = false;
       v_cmd = 0;
       w_cmd = 0;
   }
}

void path_back2(const nav_msgs::Path msg) 
{
        if(size!=0)
        {         
            //  ROS_INFO("Odometry Robot %f , %f" , odom_x , odom_y);
                ROS_INFO("New Path, it contains: %i points" , size);
            //  ROS_INFO("time solving: %f" , msg.poses[0].pose.position.z);
                for (int j=0; j<size ; j++)
                { 
                    qp[j].x=msg.poses[j].pose.position.x;
                    qp[j].y=msg.poses[j].pose.position.y;
                    ROS_INFO("x , y %f , %f" , qp[j].x , qp[j].y );
                    
                    final_orientation = get_yaw(msg.poses[size-1].pose.orientation);
                }
            
                size_path = size;
                path_stored = true;
                robot_reached = false;
                k_v=0;
                get_path_errors();
                stop_mode();
                final_theta_corrected = false;
        }
        else
        {
            path_stored = false;
            v_cmd = 0;
            w_cmd = 0;
        }

}

void path_mpc(const nav_msgs::Path msg) 
{
        activate_cmd = true;
        size = msg.poses.size();
        

        if(size!=0)
        {         
            //  ROS_INFO("Odometry Robot %f , %f" , odom_x , odom_y);
                ROS_INFO("New Path, it contains: %i points" , size);
            //  ROS_INFO("time solving: %f" , msg.poses[0].pose.position.z);
                for (int j=0; j<size ; j++)
                { 
                    qp[j].x=msg.poses[j].pose.position.x;
                    qp[j].y=msg.poses[j].pose.position.y;
                //    ROS_INFO("x , y %f , %f" , qp[j].x , qp[j].y );
                    
                }
            
                size_path = size;
                path_stored = true;
                robot_reached = false;
                k_v=0;
                get_path_errors();
                stop_mode();
                final_theta_corrected = false;
        }
        else
        {
            path_stored = false;
            v_cmd = 0;
            w_cmd = 0;
        }
}

void orient_back(const std_msgs::Bool msg) 
{
    flag_orientation = msg.data;

}

void direction_back(const std_msgs::Bool msg) 
{
    if(flag_direction)
    {
    compensate_angle = 0;
        direction = 1;
        min_v = 0;
        max_v = 0.6;
    }
    else
    {
        compensate_angle = 3.14;
        direction = -1;
        min_v = -0.6;
        max_v = 0;
    }
}

void global_local_back(const std_msgs::Bool msg) 
{
    flag_global_map = msg.data;
    if(flag_global_map)
    {
        max_v = 0.4;
        acc_v = 0.04;
        dist_limit = 0.4;  //0.8
    }
    else
    {
        max_v = 0.2;
        acc_v = 0.02;
        dist_limit = 0.4;
    }
}

void cmd_vel_mpc_back(const geometry_msgs::Twist msg)
{
  //  w_cmd = msg.angular.z;
    w_cmd = 1*angle_error;
    ROS_INFO("wcmd %f" , w_cmd);

}

void odom_back(const nav_msgs::Odometry msg) {
    v_robot = abs(msg.twist.twist.linear.x);
         
}

void path_following_controller()
{
    max_v = 0.4;
    acc_v = 0.04;
    //  DECREASING
    if (k_v == size_path-1 && dist_error<1.5 && v_cmd>0.3)// && abs(angle_error)<=0.25 && v_cmd>0.3)  // decreasing
    {
        max_v_local=v_cmd;
        if (max_v_local>(max_v/2))
        {
            max_v_local=max_v_local-0.04;
        }
        v_cmd = sat_linear_velocity(max_v_local,min_v,acc_v ,dist_error, v_cmd);
        ind_control = 1;
    }
    else
    {
        // Motion 
        if(abs(angle_error)<0.1)  // case error angle very small
        {
            v_cmd = sat_linear_velocity(max_v,min_v,acc_v,dist_error , v_cmd);
            w_cmd = angle_error;
            ind_control = 2;
        }
        else if(abs(angle_error)>=0.1 && abs(angle_error)<0.38)  // case error angle small
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/6),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.9*angle_error;
            ind_control = 3;
        }
        else if(abs(angle_error)>=0.38 && abs(angle_error)<0.7)  // case error angle medium
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/3),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.85*angle_error;
            ind_control = 4;
        }
        else if(abs(angle_error)>=0.7 && abs(angle_error)<1)  // case error angle big
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/2),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.8*angle_error;
            ind_control = 5;
        }
        else if(abs(angle_error)>=1 && abs(angle_error)<1.25)  // case angle error very big
        {
            v_cmd = sat_linear_velocity(max_v-(max_v/1.5),min_v,acc_v,dist_error , v_cmd);
            w_cmd = 0.75*angle_error;
            ind_control = 6;   
        }
        else if(abs(angle_error)>=1.25)
        {
            v_cmd = 0;
            w_cmd = 1*angle_error;
            ind_control = 7;
        }
        
    }

 //   ROS_INFO("vel 1 %f" , v_cmd);
    
    if(abs(angle_error)>=0.1 && v_robot<=0.1 && k_v<=5 && size_path > 1) // v_cmd instead of v_robot for simulation
    {
        v_cmd = 0;
    //    ROS_INFO("STOP vel 2 %f " , v_cmd);
        w_cmd = 1*angle_error;
        ind_control = 8;
    }

    if(abs(angle_error)>=0.3 && v_robot<=0.1 && k_v<=5 && size_path == 1)
    {
        v_cmd = 0;
    //    ROS_INFO("STOP vel 2 %f " , v_cmd);
        w_cmd = 1*angle_error;
        ind_control = 10;
    }
    if(w_cmd>max_w)
    {w_cmd = max_w;}
    if(w_cmd<-max_w)
    {w_cmd = -max_w;}
    if(v_cmd>max_v)
    {v_cmd = max_v;}


    tw_msg.linear.x=v_cmd;
    tw_msg.angular.z=w_cmd;
    
  
}



void get_path_errors()
{

    double angle_path1 = 0;
    if(path_stored)
    {
        //Calculate the erros between the robot and the current segement
        err.x = (qp[k_v].x-odom_x) ; 
        err.y =  (qp[k_v].y-odom_y) ;
        dist_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
        angle_path1=atan2(err.y,err.x);
        err.theta =  angle_path1 - odom_theta - compensate_angle;
        angle_error = atan2(sin(err.theta ),cos(err.theta ));

        //Calculate the erros between the robot and the next segement
        if(k_v<(size_path-1))
        {
            err.x = (qp[k_v+1].x-qp[k_v].x);
            err.y =  (qp[k_v+1].y-qp[k_v].y);
            double next_angle_path=atan2(err.y,err.x);
            err.theta =  next_angle_path - odom_theta;
        }
        // accordingly to these errors move to the next segment      
        while (dist_error<dist_limit && k_v <size_path-1)
        {
            err.x = (qp[k_v].x-odom_x) ; 
            err.y =  (qp[k_v].y-odom_y) ;
            dist_error = sqrt(pow(err.x, 2) + pow(err.y, 2));
            angle_path1=atan2(err.y,err.x);
            err.theta =  angle_path1 - odom_theta - compensate_angle;
            angle_error = atan2(sin(err.theta ),cos(err.theta ));
            k_v++;
        }
    }  
}

void stop_mode()
{
    dist_stop = sqrt(pow((qp[size_path-1].x-odom_x), 2) + pow((qp[size_path-1].y-odom_y), 2));
    if (dist_stop<=0.25)
    {
        path_stored=false;
        if(!flag_orientation)
        {
            robot_reached=true;
            ROS_INFO("Robot Reached to the goal");
            status_msg.data="reached";
        }
    }
    else
    {
        status_msg.data="not reached";
        robot_reached = false;
    }
    
}

void reset_parameters()
{
    v_cmd = 0;
    w_cmd = 0;
}

void publish_marker_odom()
{
    robot_pt.x=odom_x;
    robot_pt.y=odom_y;
    robot_mk.points.push_back(robot_pt);
    robot_mk_pub.publish(robot_mk);
}

void correction_final_theta()
{
    err.theta =  final_orientation - odom_theta;
    angle_error = atan2(sin(err.theta ),cos(err.theta ));
    if (abs(err.theta)>=0.05 && !final_theta_corrected)
    {
        v_cmd = 0.0;
        w_cmd = 1.2*angle_error;
        if (angle_error > 0)
        {
            w_cmd = 0.25;
        }
        else if(angle_error<0)
        {
            w_cmd = -0.25;
        }
    }
    if (abs(err.theta)<0.06 && !final_theta_corrected)
    {
        robot_reached=true;
        v_cmd = 0;
        w_cmd = 0;
        final_theta_corrected = true;
        status_msg.data="reached";
        ROS_INFO("Robot Reached to the goal");
        ROS_INFO("The Final ORIENTATION IS CORRECTED");
    }
   /// ROS_INFO("Final orient error %f" , abs(err.theta));
}

double sat_linear_velocity(double max , double min ,double accel, double v_ref , double v_real)
{
    double v_diff = v_ref - v_real;
    // saturations on accelerations
    if(v_diff>=0)
    {
        v_diff = v_real + direction * accel*dt;
    }
    else if(v_diff<0)
    {
        v_diff = v_real - direction * accel*dt;
    }

    if (v_diff<min)
    {
        return min;
    }
    else if (v_diff>max)
    {
        return max;
    }
    else
    {
        return v_diff;
    }
}

void initialize_parameters(ros::NodeHandle n)
{
    n.getParam("path_topic1",   path_topic1);
    n.getParam("path_topic2",   path_topic2);
    n.getParam("path_topic3",   path_topic3);
    n.getParam("flag_global_map",         flag_global_map);
    n.getParam("globla_local_topic",   globla_local_topic);   
    n.getParam("direction_topic",   direction_topic); 
    n.getParam("orient_topic",   orient_topic);
    n.getParam("cmd_topic" ,    cmd_topic);
    n.getParam("global_frame" , global_frame);
    n.getParam("base_frame" ,   base_frame);
    n.getParam("odometry_topic" ,  odometry_topic);
    n.getParam("status_topic" , status_topic);
    n.getParam("max_v", max_v);
    n.getParam("min_v", min_v);
    n.getParam("acc_v", acc_v);
    n.getParam("max_w", max_w);
    n.getParam("acc_w", acc_w);
}

void define_sub_pub(ros::NodeHandle n)
{
    // Define the publishers and sunscribers
    path_sub_1          = n.subscribe                              (path_topic1,10 , path_back);
    path_sub_2          = n.subscribe                              (path_topic2,10 , path_back2);
    path_sub_3          = n.subscribe                              (path_topic3,10 , path_mpc);
    cmd_vel_mpc         = n.subscribe                              ("cmd_vel_mpc_",10 , cmd_vel_mpc_back);
    gloab_local_sub     = n.subscribe                              (globla_local_topic,10 , global_local_back);
    direction_sub       = n.subscribe                              (direction_topic,10 , direction_back);
    orient_sub          = n.subscribe                              (orient_topic,10 , orient_back);
    odom_sub            = n.subscribe                              ("/husky_velocity_controller/odom", 5, odom_back);
    cmd_pub             = n.advertise<geometry_msgs::Twist>        (cmd_topic, 1);
    robot_mk_pub        = n.advertise<visualization_msgs::Marker>  ("marker_real", 1);
    status_control_pub  = n.advertise<std_msgs::String>            (status_topic, 10);
}

void initialize_markers()
{

    robot_mk.header.frame_id = global_frame;
    robot_mk.header.stamp = ros::Time();
    robot_mk.ns = "points_and_lines";
    robot_mk.action = visualization_msgs::Marker::ADD;
    robot_mk.pose.orientation.w = 1.0;
    robot_mk.type = 4;
    robot_mk.scale.x = 0.2;
    robot_mk.scale.y = 0.2;
    robot_mk.color.a = 1.0;
    robot_mk.color.r = 0.0;
    robot_mk.color.g = 0.0;
    robot_mk.color.b = 1.0;
    robot_mk.points.clear();
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle n;
    initialize_parameters(n);

    initialize_markers();

    define_sub_pub(n);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    if(flag_global_map)
    {
        ROS_WARN("High speed motion");
    }
    else
    {
        ROS_WARN("Low speed motion");
    }


    ros::Rate loop_rate(10); // ros spins 20 frames per second

    while (ros::ok()) 
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {    
            transformStamped = tfBuffer.lookupTransform(global_frame, base_frame,
                               ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        odom_x = transformStamped.transform.translation.x;
        odom_y = transformStamped.transform.translation.y;
        odom_theta = get_yaw(transformStamped.transform.rotation);

    //    ROS_INFO("Odometry robot %f %f" , odom_x , odom_y);


        if (path_stored) 
        {
            get_path_errors();
            path_following_controller();
            publish_marker_odom();
            stop_mode(); 
        }
        else
        {
            tw_msg.linear.x=0;
            tw_msg.angular.z=0;
            reset_parameters();
            if (flag_orientation)
            {
                correction_final_theta();
                tw_msg.angular.z=w_cmd;
            }
            
        }

        // to start smoothly in case emergency stop
        if (abs(v_robot)<0.06 && abs(v_cmd-v_robot)>0.4)
        {
            v_cmd = 0;
            tw_msg.linear.x = v_cmd;
            ind_control = 9;
        }

        if (activate_cmd)
        {
            cmd_pub.publish(tw_msg);
            status_control_pub.publish(status_msg);
        }

        if (robot_reached)
        {
            activate_cmd = false;
            status_msg.data="";
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

double get_yaw(geometry_msgs::Quaternion q)
{
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
