/* Target Detector
 *
 * Detect a bull's-eye or a set of bulls-eyes in an input image
 *
 * Note: this is my first piece of opencv code. it will probably suck
 *
 * Author: Austin Hendrix
 */

#include <map>
#include <set>
#include <list>
#include <queue>

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

      void fitEllipse(std::vector<cv::Point> &points) {
         // thoughts on ellipse-fitting
         // if it's really an ellipse, the center will be an average of all
         // points
         //
         // once we have the center, we ought to be able to derive the angles
         // of the major and minor axes, do some sort of compromise, and
         // then derive the length of the major and minor axes
         // 
         // it might be worth considering transforming the input points
         // into an r-theta space and fitting a sinusoid

         std::vector<cv::Point>::iterator itr;

         // derive center of point cluster
         float x=0, y=0;
         for( itr = points.begin(); itr != points.end(); ++itr ) {
            x += itr->x;
            y += itr->y;
         }
         x /= points.size();
         y /= points.size();

         // equation for ellipse in polar coordinates:
         // r(t) = (R)^2 + 2dR * cos(2t) + d^2
         //
         // equation for ellipse in cartesian coordinates:
         // (x/a)^2 + (y/b)^2 = 1
      }

      void imageCb(const sensor_msgs::ImageConstPtr & msg) {
         cv_bridge::CvImagePtr cv_ptr;
         try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);

            // detect edges in input image
            cv::Mat edges;
            cv::Canny(cv_ptr->image, edges, t1, t2);

            // Grouping algorithm by Michael LeKander <michaelll@michaelll.com>
            int groups = 0;
            int i, o, endBound = edges.rows * edges.cols;
            int * contours = (int*)malloc((sizeof(int)*endBound) + 2);
            int *stack = contours, *queue = contours;
            uchar * isWhite = edges.data;
            int width = edges.cols;

            for (i = 0; i < endBound; i++) {
               if (isWhite[i]) {
                  *stack++ = i;
                  isWhite[i] = 0;

                  for (o = *queue; queue != stack; o = *++queue) { 
                     // all surrounding points
                     for( int j=-1; j<=1; j++ ) {
                        for( int k=-1; k<=1; k++ ) {
                           int p = o + j + k*width;
                           if( p%width == o%width + j ) {
                              if( p >= 0 && p < endBound ) {
                                 if( isWhite[p] ) {
                                    *stack++ = p;
                                    isWhite[p] = 0;
                                 }
                              }
                           }
                        }
                     }
                  }

                  *stack++ = -1;
                  queue++;
                  groups++;
               }
            }
            *stack++ = -1;

            // fit ellipses
            std::vector<cv::Point> points;
            std::list<cv::RotatedRect> ellipses;
            for( int i=0; 
                  i < endBound && (contours[i] >= 0 || contours[i+1] >= 0);
                  i++ ) {
               if( contours[i] >= 0 ) {
                  // another point. add it
                  int j = contours[i] / width;
                  int k = contours[i] % width;
                  points.push_back(cv::Point(k, j));
               } else {
                  // fit ellipse; need at least 5 points
                  if( points.size() >= 5 ) {
                     cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(points));
                     cv::Point2f center = ellipse.center;
                     /*
                     printf("Fitted ellipse at (%03.5f, %03.5f)\n", 
                           center.x, center.y);
                           */
                     // TODO: determine quality of fit; either write custom
                     // fitting algorithm or measure fit.
                     // TODO: only consider ellipses that fit "well"
                     ellipses.push_back(ellipse);
                  }

                  points.clear();
               }
            }
            //printf("\n");

            //printf("% 6d groups\n", groups);

            // generate colors
            std::map<int, cv::Vec3b> colors;
            srand(0);
            for( int i=0; i<groups+1; i++ ) {
               int a = rand();
               uchar r = a & 0xFF;
               uchar g = (a >> 8) & 0xFF;
               uchar b = (a >> 16) & 0xFF;
               if( r < 20 ) r = 255 - r;
               if( g < 20 ) g = 255 - g;
               if( b < 20 ) b = 255 - b;
               colors[i] = cv::Vec3b(r, g, b);
            }


            // publish edge image for viewing
            cv_bridge::CvImage out;
            // output edges
            /*
            out.encoding = enc::MONO8;
            out.header = cv_ptr->header;
            out.image = edges;
            */

            // output grouped and colored edges
            out.encoding = enc::BGR8;
            out.header = cv_ptr->header;
            out.image.create(edges.rows, edges.cols, CV_8UC3);
            out.image.setTo(0);

            int color = 0;
            for( int i = 0; 
                  i < endBound && (contours[i] >= 0 || contours[i+1] >= 0); 
                  i++ ) {
               if( contours[i] >= 0 ) { 
                  int j = contours[i] / width;
                  int k = contours[i] % width;
                  out.image.at<cv::Vec3b>(j, k) = colors[color];
               } else {
                  color++;
               }
            }

            // overlay ellipses
            for( std::list<cv::RotatedRect>::iterator itr = ellipses.begin();
                  itr != ellipses.end(); itr++ ) {
               cv::Size size;
               size.width = round(itr->size.width * 0.5);
               size.height = round(itr->size.height * 0.5);
               cv::ellipse(out.image, itr->center, size, 
                     itr->angle, 0, 360, CV_RGB(255, 255, 255), 1, CV_AA, 0);
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
