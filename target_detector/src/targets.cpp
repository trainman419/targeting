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

      void imageCb(const sensor_msgs::ImageConstPtr & msg) {
         cv_bridge::CvImagePtr cv_ptr;
         try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);

            // detect edges in input image
            cv::Mat edges;
            cv::Canny(cv_ptr->image, edges, t1, t2);

            // loop on pixels in edge image


            // Grouping algorithm by Michael LeKander <michaelll@michaelll.com>
            int groups = 0;
            int i, o, endBound = edges.rows * edges.cols;
            int * contours = (int*)malloc(sizeof(int)*endBound + 1);
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

                  *stack++ = INT_MAX;
                  queue++;
                  groups++;
               }
            }

            printf("% 6d groups\n", groups);

            /*
            int i, j;
            uchar * row = 0;
            int count = 0;
            for( i=0; i<edges.rows; i++) {
               row = edges.ptr<uchar>(i);
               for( j=0; j<edges.cols; j++) {
                  if( row[j] ) count++;
               }
            }


            int id = 0;
            // position of each node
            std::vector<std::pair<int, int> > position(count);
            // neighbor ids for each node
            std::vector<std::list<int> > neighbors(count);
            // set of all node ids (for later)
            std::set<int> nodes;
            // image-coordinate to ID map
            cv::Mat ids(edges.rows, edges.cols, CV_32S);

            ROS_ASSERT(edges.type() == CV_8U);
            uchar * row_b = 0;
            int * ids_row;
            // for each pixel
            for( i=0; i<edges.rows; i++) {
               row_b = row;
               row = edges.ptr<uchar>(i);
               ids_row = ids.ptr<int>(i);
               for( j=0; j<edges.cols; j++) {
                  ids_row[j] = -1;
                  if( row[j] ) {
                     // set ID into image
                     ids_row[j] = id;
                     // add to node set
                     nodes.insert(id);
                     // set position
                     position[id] = std::pair<int, int>(i, j);


                     // if the pixel to the left is set, add to neighbors
                     if( j>0 && row[j-1] ) {
                        neighbors[id].push_back(ids_row[j-1]);
                     } 

                     // if there is a line above us...
                     if( i>0 ) {
                        // above left
                        if( j>0 && row_b[j-1] ) {
                           neighbors[id].push_back(ids.at<int>(i-1, j-1));
                        }

                        // above
                        if( row_b[j] ) {
                           neighbors[id].push_back(ids.at<int>(i-1, j));
                        }

                        // above right. BUG here?
                        if( j<(edges.cols-1) && row_b[j+1] ) {
                           neighbors[id].push_back(ids.at<int>(i-1, j+1));
                        }
                     } 

                     // next id
                     id++;
                  }
               }
            }

            printf("% 6d edge points\n", count);
            ROS_ASSERT(id == count);

            std::list<std::set<int> *> groups;
            std::queue<int> todo;
            std::set<int> * group;

            int group_sum = 0;

            // while we have unprocessed nodes
            while(nodes.size() > 0) {
               // get first node
               int n = *(nodes.begin());
               group = new std::set<int>();
               todo.push(n);
               nodes.erase(n);
               group->insert(n);
               group_sum++;
               while( todo.size() > 0 ) {
                  n = todo.front();
                  for( std::list<int>::iterator itr = neighbors[n].begin();
                        itr != neighbors[n].end(); itr++ ) {
                     ROS_ASSERT(*itr != -1);
                     if( ! group->count(*itr) && nodes.count(*itr) ) {
                        todo.push(*itr);
                        nodes.erase(*itr);

                        group->insert(*itr);
                        group_sum++;
                     }
                  }
                  todo.pop();
               }

               groups.push_back(group);
            }

            printf("% 6zd groups\n", groups.size());
            printf("% 6d group sum\n", group_sum);

            // delete groups so we don't leak memory
            for( std::list<std::set<int> *>::iterator itr = groups.begin();
                  itr != groups.end(); itr++ ) {
               delete *itr;
            }
            */

            // count the number of resulting groups
            /*
            printf("% 6zd initial groups\n", group_num.size());
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
            printf("% 6zd condensed groups\n", final_groups.size());

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
            */


            // publish edge image for viewing
            cv_bridge::CvImage out;
            // output edges
            out.encoding = enc::MONO8;
            out.header = cv_ptr->header;
            out.image = edges;

            /*
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
            */

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
