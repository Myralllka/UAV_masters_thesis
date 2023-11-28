#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>
#include <random>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>

/* other important includes */
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/circular_buffer.hpp>



/* user includes */

//}

namespace masters {
    using vec3 = Eigen::Vector3d;

/* class Masters //{ */
    class Masters : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_is_initialized = false;
        double m_mean;
        double m_stddev;
        double m_upd_th;

        int m_history_bufsize;

        /* ros parameters */
//        std::string m_uav_name;
        std::string m_name_eagle;
        std::string m_name_target;
        std::string m_name_world_origin;
        std::string m_nodename = "Masters";
        std::string m_name_front_camera;

        /* other parameters */
        image_geometry::PinholeCameraModel m_camera_front;
        boost::circular_buffer<std::pair<vec3, vec3>> m_history;


        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        [[maybe_unused]] void m_cbk_front_camera_detection(const geometry_msgs::PoseArray &msg);
        [[maybe_unused]] void m_cbk_front_camera(const sensor_msgs::ImageConstPtr &msg);

        // | --------------------- timer callbacks -------------------- |

        // | ----------------------- publishers ----------------------- |
        ros::Publisher m_pub_image_changed;
        ros::Publisher m_pub_front_camera_detection;
        ros::Publisher m_pub_history1;
        ros::Publisher m_pub_history2;
        ros::Publisher m_pub_viz;

        // | ----------------------- subscribers ---------------------- |
        ros::Subscriber m_sub_front_camera_detection;
        ros::Subscriber m_sub_front_camera;


        // | --------------------- other functions -------------------- |
        static vec3 m_find_intersection_svd(const boost::circular_buffer<std::pair<vec3, vec3>> &data);

        std::optional<cv::Point2d> m_detect_uav(const sensor_msgs::Image::ConstPtr &msg);
    };

}  // namespace masters
