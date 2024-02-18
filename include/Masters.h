#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <masters/DynrecConfig.h>


/* some STL includes */
#include <cstdlib>
#include <cstdio>
#include <random>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/geometry/misc.h>


/* other important includes */
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/circular_buffer.hpp>



/* user includes */

//}

namespace masters {
    /* class DKF //{ */
    template<int n_states, int n_inputs>
    class DKF : public mrs_lib::LKF<n_states, n_inputs, -1> {
    public:
        /* DKF definitions (typedefs, constants etc) //{ */
        using Base_class = mrs_lib::LKF<n_states, n_inputs, -1>;
        static constexpr int n = Base_class::n;
        static constexpr int m = Base_class::m;
        static constexpr int p = Base_class::p;
        using x_t = typename Base_class::x_t;
        using u_t = typename Base_class::u_t;
        using z_t = typename Base_class::z_t;
        using P_t = typename Base_class::P_t;
        using R_t = typename Base_class::R_t;
        using Q_t = typename Base_class::Q_t;
        using statecov_t = typename Base_class::statecov_t;
        typedef Eigen::Matrix<double, n, n> A_t;
        typedef Eigen::Matrix<double, n, m> B_t;
        typedef Eigen::Matrix<double, p, n> H_t;
        typedef Eigen::Matrix<double, n, p> K_t;
        using mat2_t = Eigen::Matrix<double, 2, 2>;
        using mat3_t = Eigen::Matrix<double, 3, 3>;
        using pt3_t = mrs_lib::geometry::vec3_t;
        using pt2_t = mrs_lib::geometry::vec2_t;
        using vec3_t = mrs_lib::geometry::vec3_t;
        //}

    public:
        DKF() {};

        DKF(const A_t &A, const B_t &B) : Base_class(A, B, {}) {};

        /* correctLine() method //{ */
        virtual std::enable_if_t<(n > 3), statecov_t>
        correctLine(const statecov_t &sc, const pt3_t &line_origin, const vec3_t &line_direction,
                    const double line_variance) const {
            assert(line_direction.norm() > 0.0);
            const vec3_t zunit{0.0, 0.0, 1.0};
            // rot is a rotation matrix, transforming from F to F'
            const mat3_t rot = mrs_lib::geometry::rotationBetween(line_direction, zunit);
            /* const mat3_t rotT = rot.transpose(); */

            H_t H = Eigen::Matrix<double, 2, n>::Zero();
            H.block(2, 2, 0, 0) = rot.block<2, 2>(0, 0);

            const pt3_t oprime = rot * line_origin;
            const pt2_t z = oprime.head<2>();

            const mat2_t R = line_variance * mat2_t::Identity();
            return this->correction_impl(sc, z, R, H);
        };
        //}
    };
    //}

    using vec3 = Eigen::Vector3d;

    using t_hist_vv = boost::circular_buffer<std::pair<vec3, vec3>>;
    using t_hist_vvt = boost::circular_buffer<std::tuple<vec3, vec3, ros::Time>>;

/* class Masters //{ */
    class Masters : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_is_initialized = false;
        bool m_is_kalman_initialized = false;
        double m_mean;
        double m_stddev;
        double m_upd_th;
        double m_dt;
        ros::Time m_t0;
        int m_history_bufsize;

        /* ros parameters */
//        std::string m_uav_name;
        std::string m_name_eagle;
        std::string m_name_front_camera;
        std::string m_name_front_camera_tf;
        std::string m_name_target;
        std::string m_name_world_origin;
        std::string m_name_eagle_odom_msg;
        const std::string m_nodename = "Masters";

        /* other parameters */
        image_geometry::PinholeCameraModel m_camera_front;
        t_hist_vv m_history_linear;
        t_hist_vvt m_history_velocity;

        /* Kalman filter */
        Eigen::Matrix<double, 6, 1> m_state_interceptor;
//        Eigen::Matrix<double, 6, 1> m_x_k;
//        Eigen::Matrix<double, 6, 6> m_P_k;
        Eigen::Matrix<double, 6, 6> m_P0;
//        Eigen::Matrix<double, 3, 3> m_Q;
        Eigen::Matrix<double, 6, 6> m_Q;
        Eigen::Matrix3d m_R;
        ros::Time m_last_kalman_time;
        ros::Time m_time_prev_real;

        // Dynamic reconfigure
        masters::DynrecConfig m_dynrecconf;
        dynamic_reconfigure::Server<masters::DynrecConfig> server;

        void m_cbk_dynrec(masters::DynrecConfig &config, uint32_t level);

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        [[maybe_unused]] void m_cbk_detection(const geometry_msgs::PointStamped &msg);

        [[maybe_unused]] void m_cbk_front_camera(const sensor_msgs::ImageConstPtr &msg);

        // | --------------------- timer callbacks -------------------- |
        void update_kalman(Eigen::Vector3d m_detection_vec, ros::Time m_detection_time);
        // void m_tim_cbk_kalman(const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |
        ros::Publisher m_pub_image_changed;
//        ros::Publisher m_pub_front_camera_detection;
        ros::Publisher m_pub_history1;
        ros::Publisher m_pub_history2;
        ros::Publisher m_pub_viz;
        ros::Publisher m_pub_detection;
        ros::Publisher m_pub_target_odom;

        // | ----------------------- subscribers ---------------------- |
        mrs_lib::SubscribeHandler<nav_msgs::Odometry> m_subh_eagle_odom;

        ros::Subscriber m_sub_detection;
        ros::Subscriber m_sub_front_camera;

        ros::Timer m_tim_kalman;

        // | --------------------- other functions -------------------- |

        std::optional<cv::Point2d> m_detect_uav(const sensor_msgs::Image::ConstPtr &msg);

        DKF<6, 0> m_dkf = DKF<6, 0>();
        DKF<6, 0>::statecov_t m_state_vec;

        std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>
        plkf_predict(const Eigen::Matrix<double, 6, 1> &xk_1,
                     const Eigen::Matrix<double, 6, 6> &Pk_1,
                     const Eigen::Matrix<double, 6, 1> &x_i,
                     const Eigen::Matrix<double, 6, 1> &x_i_,
                     const double &dt);

        std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>
        plkf_correct(const Eigen::Matrix<double, 6, 1> &xk_,
                     const Eigen::Matrix<double, 6, 6> &Pk_,
                     const Eigen::Vector3d &lmb);

        // https://gist.github.com/javidcf/25066cf85e71105d57b6
        template<class MatT>
        Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
        pseudoinverse(const MatT &mat,
                      typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
        {
            typedef typename MatT::Scalar Scalar;
            auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
            const auto &singularValues = svd.singularValues();
            Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(),
                                                                                                      mat.rows());
            singularValuesInv.setZero();
            for (unsigned int i = 0; i < singularValues.size(); ++i) {
                if (singularValues(i) > tolerance) {
                    singularValuesInv(i, i) = Scalar{1} / singularValues(i);
                } else {
                    singularValuesInv(i, i) = Scalar{0};
                }
            }
            return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
        }
    };

}  // namespace masters
