/* Target Detector
 *
 * Detect a bull's-eye or a set of bulls-eyes in an input image
 *
 * Note: this is my first piece of opencv code. it will probably suck
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include <target_detector/TargetDetectorConfig.h>

namespace enc = sensor_msgs::image_encodings;

class TargetsDetector {
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;

   double t1;
   double t2;

   public:
      TargetsDetector() : it_(nh_), t1(300.0), t2(150.0) {
         image_sub_ = it_.subscribe("in", 1, &TargetsDetector::imageCb, this);
         image_pub_ = it_.advertise("out", 1);
      }

      void dyn_callback(target_detector::TargetDetectorConfig & config, 
            uint32_t level) {
         // this probably isn't thread-safe and I DON'T CARE
         // (it should only screw up one image in the worse case)
         t1 = config.t1;
         t2 = config.t2;
         ROS_INFO("Reconfigured to %lf %lf", t1, t2);
      }

      void imageCb(const sensor_msgs::ImageConstPtr & msg) {
         cv_bridge::CvImagePtr cv_ptr;
         try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);

            // detect edges in input image
            cv::Mat edges;
            // TODO: use dynamic-reconfigure to set parameters for this
            cv::Canny(cv_ptr->image, edges, t1, t2);

            //ROS_INFO("Edge detector complete; size %d x %d", edges.cols, edges.rows);
            // loop on pixels in edge image
            int i, j;
            cv::Mat groups(edges.rows, edges.cols, CV_32S);
            std::map<int, int> group_num;
            int group = 0;
            group_num[0] = 0; // no group

            ROS_ASSERT(edges.type() == CV_8U);
            uchar * row_b = 0;
            uchar * row = 0;
            for( i=0; i<edges.rows; i++) {
               row_b = row;
               row = edges.ptr<uchar>(i);
               int * group_row = groups.ptr<int>(i);
               for( j=0; j<edges.cols; j++) {
                  group_row[j] = 0;
                  if( row[j] ) {
                     // if the previous pixel in this row is set, add us to 
                     //  that group
                     if( j>0 && row[j-1] ) {
                        group_row[j] = group_row[j-1];
                     } 

                     // if the pixel above us is set, add us to that group
                     if( i>0 ) {
                        int f = -1;
                        // above left
                        if( j>0 && row_b[j-1] ) f = j-1;
                        // above
                        else if( row_b[j] ) f = j;
                        // above right. BUG here?
                        else if( j<(edges.cols-1) && row_b[j+1] ) {
                           f = j+1;
                        }

                        if( f > -1 ) {
                           int g = groups.at<int>(i-1, f);
                           // if this point is already in a group; rename its 
                           // group to match the other group
                           //  this effectively merges two groups
                           if( group_row[j] && group_row[j] != g ) {
                              int a = group_num[group_row[j]];
                              int n = a < group_num[g] ? a : group_num[g];
                              group_num[group_row[j]] = n;
                              group_num[g] = n;
                           } else {
                              group_row[j] = group_num[g];
                           }
                        }

                     } 

                     // if we didn't find a group, create a new one
                     if( ! group_row[j] ) {
                        group++;
                        group_num[group] = group;
                        group_row[j] = group;
                     }
                  }
               }
            }

            // count the number of resulting groups
            //printf("% 6zd initial groups\n", group_num.size());
            std::set<int> final_groups;
            for( std::map<int, int>::iterator itr = group_num.begin(); 
                  itr != group_num.end(); itr++ ) {

               int g = itr->second;
               while( g != group_num[g] ) g = group_num[g];
               itr->second = g;

               if( itr->first && !itr->second ) {
                  printf("zero-id group detected");
               }
               final_groups.insert(itr->second);
            }
            //printf("% 6zd condensed groups\n", final_groups.size());

            // generate colors
            std::map<int, cv::Vec3b> colors;
            srand(0);
            for( std::set<int>::iterator itr = final_groups.begin();
                  itr != final_groups.end(); itr++ ) {
               int a = rand();
               uchar r = a & 0xFF;
               uchar g = (a >> 8) & 0xFF;
               uchar b = (a >> 16) & 0xFF;
               if( r < 20 ) r = 255 - r;
               if( g < 20 ) g = 255 - g;
               if( b < 20 ) b = 255 - b;
               colors[*itr] = cv::Vec3b(r, g, b);
            }
            // for color 0 to black (no edge group)
            colors[0] = cv::Vec3b(0, 0, 0);


            // publish edge image for viewing
            cv_bridge::CvImage out;
            /*
            // output edges
            out.encoding = enc::MONO8;
            out.header = cv_ptr->header;
            out.image = edges;
            */

            // output grouped and colored edges
            out.encoding = enc::BGR8;
            out.header = cv_ptr->header;
            out.image.create(groups.rows, groups.cols, CV_8UC3);
            for( int i=0; i<groups.rows; i++ ) {
               int * group_row = groups.ptr<int>(i);
               cv::Vec3b * out_row = out.image.ptr<cv::Vec3b>(i);
               for( int j=0; j<groups.cols; j++ ) {
                  int g = group_num[group_row[j]];
                  out_row[j] = colors[g];
               }
            }

            image_pub_.publish(out.toImageMsg());

         } catch(cv_bridge::Exception &e) {
            ROS_ERROR("cv_Bridge exception: %s", e.what());
         }
      }
};

int main(int argc, char ** argv) {
   ros::init(argc, argv, "targets");
   TargetsDetector td;

   dynamic_reconfigure::Server<target_detector::TargetDetectorConfig> server;
   dynamic_reconfigure::Server<target_detector::TargetDetectorConfig>::CallbackType f;
   f = boost::bind(&TargetsDetector::dyn_callback, &td, _1, _2);

   server.setCallback(f);

   ros::spin();
   return 0;
}
