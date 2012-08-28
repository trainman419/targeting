/*
 * rocket_aim: a node for aiming a USB nerf turret based on targets
 * detected by a vision algorithm and a kinect
 *
 * Author: Austin Hendrix
 */

#include <math.h>

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <target_detector/PointArray.h>
#include <rocket_msgs/RocketCommand.h>
#include <rocket_msgs/RocketPosition.h>
#include <std_srvs/Empty.h>

double dx, dy, dz;
ros::Publisher cmd_pub;

double last_a, last_b;
double delta;
int stable_cnt;

ros::ServiceClient fire;

double dist(geometry_msgs::Point p) {
   return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

void positionCb( const rocket_msgs::RocketPositionConstPtr & msg ) {
   // TODO: look at position and occasionally re-home end stops
   double da = msg->alpha - last_a;
   double db = msg->beta - last_b;
   if( fabs(da) > 0.15 || fabs(db) > 0.15 ) {
      ROS_INFO("Positioning error: (%lf, %lf)", da, db);
      stable_cnt = 0;
   }
}

void pointsCb( const target_detector::PointArrayConstPtr & msg ) {
   // coordinates are in optical frame, and I'm too lazy to translate them
   // right now.
   // z: distance to target
   // x: right
   // y: down
   if( msg->points.size() < 1 ) return;
   geometry_msgs::Point target = msg->points.front();
   double d = dist(target);

   // choose the closest target (to the camera)
   BOOST_FOREACH( geometry_msgs::Point p, msg->points ) {
      if( dist(p) < d ) {
         d = dist(p);
         target = p;
      }
   }

   //ROS_INFO("Chosen target: (%lf, %lf, %lf)", target.x, target.y, target.z);

   // in ROS-standard axes
   double x = target.z - dx;
   double y = -target.x - dy;
   double z = -target.y - dz;
   double yaw = atan2( y, x );

   double pitch = -atan2( z, sqrt( x*x + y*y ) );

   delta = (delta + fabs(yaw - last_a) + fabs(pitch - last_b))/2.0;

   d = delta;
   ROS_INFO("Target drift: %lf", d);
   if( d < 0.005 ) {
      ++stable_cnt;
      if( stable_cnt > 30 ) {
         ROS_INFO("Target stable for %d messages, firing", stable_cnt);
         std_srvs::Empty e;
         if( ! fire.call(e) ) {
            ROS_ERROR("Failed to fire rocket!");
         }
         stable_cnt = 0;
      }
   } else {
      stable_cnt = 0;
   }

   rocket_msgs::RocketCommand cmd;
   cmd.alpha = yaw;
   cmd.beta = pitch;
   //ROS_INFO("Target yaw, pitch: %lf, %lf", yaw*180.0/M_PI, pitch*180.0/M_PI);
   cmd_pub.publish(cmd);

   last_a = yaw;
   last_b = pitch;
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "rocket_aim");

   ros::NodeHandle nh;

   nh.param<double>("x_offset", dx, 0.0);
   nh.param<double>("y_offset", dy, 0.0);
   nh.param<double>("z_offset", dz, 0.0);

   delta = 0.0;
   stable_cnt = 0;
   last_a = 0.0;
   last_b = 0.0;

   fire = nh.serviceClient<std_srvs::Empty>("rocket_fire");
   cmd_pub = nh.advertise<rocket_msgs::RocketCommand>("rocket_command", 2);
   ros::Subscriber s = nh.subscribe<target_detector::PointArray>("targets", 1,
         &pointsCb);
   ros::Subscriber s2 = nh.subscribe<rocket_msgs::RocketPosition>(
         "rocket_position", 1, &positionCb);

   ROS_INFO("Rocket aiming ready");

   ros::spin();
   return 0;
}
