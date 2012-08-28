/* Target Detector
 *
 * Detect a bull's-eye or a set of bulls-eyes in an input image
 *
 * Note: this is my first piece of opencv code. it will probably suck
 *
 * Author: Austin Hendrix
 */

#include <math.h>

#include <map>
#include <set>
#include <list>
#include <queue>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <dynamic_reconfigure/server.h>
#include <target_detector/TargetDetectorConfig.h>
#include <target_detector/PointArray.h>

namespace enc = sensor_msgs::image_encodings;

class TargetsDetector {
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;
   image_transport::Publisher edges_pub_;

   ros::Subscriber points_sub_;
   ros::Publisher targets_pub_;

   double t1;
   double t2;
   double allowed_error;
   double center_threshold;

   public:
      TargetsDetector() : it_(nh_), t1(300.0), t2(150.0), allowed_error(10) {
         image_sub_ = it_.subscribe("in", 1, &TargetsDetector::imageCb, this);
         points_sub_ = nh_.subscribe("points2", 1, &TargetsDetector::cloudCb, 
               this);
         image_pub_ = it_.advertise("out", 1);
         edges_pub_ = it_.advertise("edges", 1);
         targets_pub_ = nh_.advertise<target_detector::PointArray>("targets", 1);
      }

      // dynamic-reconfigure callback
      void dyn_callback(target_detector::TargetDetectorConfig & config, 
            uint32_t level) {
         // this probably isn't thread-safe and I DON'T CARE
         // (it should only screw up one image in the worse case)
         t1 = config.t1;
         t2 = config.t2;
         allowed_error = config.error;
         center_threshold = config.center;
      }

      inline float dist(cv::Point a, cv::Point b) {
         return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
      }

      class Ellipse {
         public:
            cv::Point center;
            float error;
            Ellipse(cv::Point c, float e) : center(c), error(e) {}
      };

      // average RMS error
      float ellipseError(cv::RotatedRect rect, std::vector<cv::Point> &points){
         cv::Point2f center = rect.center;
         float a = rect.size.width / 2.0f;
         float b = rect.size.height / 2.0f;
         float angle = (90.0 + rect.angle) * M_PI / 180.0;
         //ROS_ASSERT(a >= b);

         float err = 0;
         float theta_max = 0;
         float d_max = 0;
         for( std::vector<cv::Point>::iterator itr = points.begin();
               itr != points.end(); ++itr ) {
            float d = dist(center, *itr);
            float t = atan2f(itr->y - center.y, itr->x - center.x);
            if( d > d_max ) {
               d_max = d;
               theta_max = t;
            }
            float t2 = t - angle;
            float r = a * b / sqrt( (a*cosf(t2)) * (a*cosf(t2)) + 
                                    (b*sinf(t2)) * (b*sinf(t2)) );
            err += ((r-d)/r) * ((r-d)/r);
         }
         err /= points.size();
         err = sqrt(err);

         return err;
      }

      struct groups {
         int * contours;
         int end;
         int count;
      };

      // Grouping algorithm by Michael LeKander <michaelll@michaelll.com>
      groups extractContours(cv::Mat edges) {
         int count = 0;
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
               count++;
            }
         }
         *stack++ = -1;

         groups g;
         g.contours = contours;
         g.end = endBound;
         g.count = count;
         return g;
      }

      void falseColorGroups(groups g, std_msgs::Header header, int h, int w) {
         if( edges_pub_.getNumSubscribers() > 0 ) {
            // generate colors
            std::map<int, cv::Vec3b> colors;
            srand(0);
            for( int i=0; i<g.count+1; i++ ) {
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

            // output grouped and colored edges
            out.encoding = enc::BGR8;
            out.header = header;
            out.image.create(h, w, CV_8UC3);
            out.image.setTo(0);

            int color = 0;
            for( int i = 0; 
                  i < g.end && (g.contours[i] >= 0 || g.contours[i+1] >= 0); 
                  i++ ) {
               if( g.contours[i] >= 0 ) { 
                  int j = g.contours[i] / w;
                  int k = g.contours[i] % w;
                  out.image.at<cv::Vec3b>(j, k) = colors[color];
               } else {
                  color++;
               }
            }

            edges_pub_.publish(out.toImageMsg());
         }
      }

      // image callback
      void imageCb(const sensor_msgs::ImageConstPtr & msg) {
         try {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);

            std::list<cv::Point2f> centers = extractCenters(cv_ptr);

            publishCrosshairs(msg, centers);
         } catch(cv_bridge::Exception &e) {
            ROS_ERROR("cv_Bridge exception: %s", e.what());
         }
      }

      // point cloud callback
      void cloudCb(const sensor_msgs::PointCloud2ConstPtr & cloud) {
         if( (cloud->width * cloud->height) == 0 ) {
            // not a dense cloud; return
            return;
         }
         try {
            sensor_msgs::ImagePtr image = boost::make_shared<sensor_msgs::Image>();
            pcl::toROSMsg( *cloud, *image );

            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(image, enc::MONO8);

            std::list<cv::Point2f> centers = extractCenters(cv_ptr);

            // TODO: extract depth to points
            pcl::PointCloud<pcl::PointXYZ> depth;
            pcl::fromROSMsg( *cloud, depth);

            // TODO: find appropriate output data type
            // many PointStamped on one topic?
            target_detector::PointArray targets;
            for( std::list<cv::Point2f>::iterator itr = centers.begin();
                  itr != centers.end(); ++itr ) {
               unsigned int x = itr->x;
               unsigned int y = itr->y;
               int dx[] = { 0, 1, 0, -1 };
               int dy[] = { 1, 0, -1, 0 };
               if( x >= depth.width ) x = depth.width-1;
               if( x == depth.width - 1 ) dx[1] = 0;
               if( y >= depth.height ) y = depth.height-1;
               if( y == depth.height - 1) dy[0] = 0;
               if( 0 == x ) dx[3] = 0;
               if( 0 == y ) dy[2] = 0;
               pcl::PointXYZ p1 = depth.at(x, y);
               for( int i=0; isnan(p1.x) && i < 4; ++i ) {
                  p1 = depth.at(x + dx[i], y + dy[i]);
               }
               geometry_msgs::Point p2;
               p2.x = p1.x; p2.y = p1.y; p2.z = p1.z;
               ROS_INFO("Target point at (%lf, %lf, %lf)", p2.x, p2.y, p2.z);
               if( ! isnan(p2.x) ) {
                  targets.points.push_back(p2);
               }
            }
            targets_pub_.publish(targets);

            publishCrosshairs(image, centers);
         } catch( std::runtime_error e ) {
            ROS_ERROR_STREAM("Error in converting point cloud to image: " << e.what());
         }
      }

      std::list<cv::Point2f> extractCenters(cv_bridge::CvImagePtr cv_ptr) {
         // detect edges in input image
         cv::Mat edges;
         cv::Canny(cv_ptr->image, edges, t1, t2);

         // group points
         groups g = extractContours(edges);

         // fit ellipses
         int width = edges.cols;
         std::vector<cv::Point> points;
         std::list<cv::RotatedRect> ellipses;
         std::list<cv::Point2f> centers;
         for( int i=0; 
               i < g.end && (g.contours[i] >= 0 || g.contours[i+1] >= 0);
               i++ ) {
            if( g.contours[i] >= 0 ) {
               // another point. add it
               int j = g.contours[i] / width;
               int k = g.contours[i] % width;
               points.push_back(cv::Point(k, j));
            } else {
               // fit ellipse; need at least 5 points
               if( points.size() >= 5 ) {
                  cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(points));

                  float error = ellipseError(ellipse, points);
                  // TODO: determine quality of fit; either write custom
                  // fitting algorithm or measure fit.
                  // TODO: only consider ellipses that fit "well"
                  if( error < allowed_error ) {
                     ellipses.push_back(ellipse);
                     centers.push_back(ellipse.center);
                  }
               }

               points.clear();
            }
         }
         printf("Fit %zd ellipses\n", ellipses.size());

         // gather center points
         typedef std::list<std::pair<cv::Point2f, std::list<cv::Point2f> > > cgType;
         cgType center_groups;
         for( std::list<cv::Point2f>::iterator itr = centers.begin(); 
               itr != centers.end(); itr++ ) {
            bool f = false;
            for( cgType::iterator itr2 = center_groups.begin(); 
                  itr2 != center_groups.end() && !f; ++itr2 ) {
               if( dist(*itr, itr2->first) < center_threshold ) {
                  itr2->second.push_back(*itr);
                  cv::Point2f avg;
                  for( std::list<cv::Point2f>::iterator itr3 = itr2->second.begin(); itr3 != itr2->second.end(); ++itr3 ) {
                     avg += *itr3;
                  }
                  avg.x /= itr2->second.size();
                  avg.y /= itr2->second.size();
                  itr2->first = avg;
                  f = true;
               }
            }
            // if we didn't find a group for this point, make a new one
            if( !f ) {
               cv::Point2f avg = *itr;
               std::list<cv::Point2f> l;
               l.push_back(*itr);
               center_groups.push_back(std::pair<cv::Point2f, std::list<cv::Point2f> >(avg, l));
            }
         }

         centers.clear();
         // threshold on number of circles required for target
         for( cgType::iterator itr = center_groups.begin(); 
               itr != center_groups.end(); ++itr ) {
            if( itr->second.size() > 2 ) { // TODO: remove magic
               printf("Found target at %f %f with %zd ellipses\n", 
                     itr->first.x, itr->first.y, itr->second.size());
               centers.push_back(itr->first);
            }
         }

         falseColorGroups(g, cv_ptr->header, edges.rows, edges.cols);

         free(g.contours);

         return centers;
      }

      void publishCrosshairs( const sensor_msgs::ImageConstPtr & msg, 
            std::list<cv::Point2f> & centers) {
         if( image_pub_.getNumSubscribers() > 0 ) {
            // republish original image
            cv_bridge::CvImagePtr out_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

            // draw crosshairs at target centers
            static int ll = 5; // half line-length of center cross
            for( std::list<cv::Point2f>::iterator itr = centers.begin();
                  itr != centers.end(); ++itr ) {
               cv::Point a1 = *itr;
               cv::Point a2 = *itr;
               cv::Point b1 = *itr;
               cv::Point b2 = *itr;
               a1.x -= ll;
               a2.x += ll;
               b1.y -= ll;
               b2.y += ll;
               cv::line(out_ptr->image, a1, a2, CV_RGB(0, 255, 0));
               cv::line(out_ptr->image, b1, b2, CV_RGB(0, 255, 0));
            }


            image_pub_.publish(out_ptr->toImageMsg());
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
