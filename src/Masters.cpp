#include <Masters.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>


namespace masters {

/* onInit() method //{ */
    void Masters::onInit() {

        // | ---------------- set my booleans to false ---------------- |
        // but remember, always set them to their default value in the header file
        // because, when you add new one later, you might forget to come back here

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */
        mrs_lib::ParamLoader pl(nh, m_nodename);

//        pl.loadParam("UAV_NAME", m_uav_name);
        pl.loadParam("eagle_name", m_name_eagle);
        pl.loadParam("target_name", m_name_target);
        pl.loadParam("world_origin", m_name_world_origin);
        pl.loadParam("triang_update_th", m_upd_th);
        pl.loadParam("history_buffer_size", m_history_bufsize);

        pl.loadParam("front_camera", m_name_front_camera);
        pl.loadParam("mean", m_mean);
        pl.loadParam("dt_kalman", m_dt);
        pl.loadParam("deviation", m_stddev);
        pl.loadParam("eagle_odometry", m_name_eagle_odom_msg);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", m_nodename.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", m_nodename.c_str());
        }

        m_Q = Eigen::Matrix3d::Identity() * 0.1;
        m_R = Eigen::Matrix3d::Identity() * 1;

        // Dynamic reconfigure
        server.setCallback(boost::bind(&Masters::m_cbk_dynrec, this, _1, _2));

        // | ---------------- some data post-processing --------------- |
        m_history_velocity = t_hist_vvt(m_history_bufsize);

        // | ----------------- publishers initialize ------------------ |
        m_pub_image_changed = nh.advertise<sensor_msgs::Image>("changed", 1);
        m_pub_detection = nh.advertise<geometry_msgs::PointStamped>("detection", 1);
        m_pub_history1 = nh.advertise<geometry_msgs::PoseArray>(m_name_eagle + "/history_1", 1);
        m_pub_history2 = nh.advertise<geometry_msgs::PoseArray>(m_name_eagle + "/history_2", 1);
        m_pub_viz = nh.advertise<visualization_msgs::Marker>(m_name_eagle + "/detection", 1);
        m_pub_target_odom = nh.advertise<nav_msgs::Odometry>(m_nodename + "/detected_target", 1);

        // | ---------------- subscribers initialize ------------------ |
        mrs_lib::SubscribeHandlerOptions shopt{nh};
        shopt.node_name = m_nodename;
        shopt.threadsafe = true;
        shopt.no_message_timeout = ros::Duration(1.0);
        sensor_msgs::CameraInfo camfront_info;
        mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> handler_camfrontinfo;
        mrs_lib::construct_object(handler_camfrontinfo,
                                  shopt,
                                  m_name_front_camera + "/camera_info");
        mrs_lib::construct_object(m_subh_eagle_odom,
                                  shopt,
                                  m_name_eagle_odom_msg);
        while (ros::ok()) {
            if (handler_camfrontinfo.hasMsg()) {
                camfront_info = *handler_camfrontinfo.getMsg().get();
                break;
            }
        }
        handler_camfrontinfo.stop();


        m_t0 = ros::Time::now();
        m_camera_front.fromCameraInfo(camfront_info);
        m_sub_detection = nh.subscribe("detection", 1,
                                       &Masters::m_cbk_detection, this);
        m_sub_front_camera = nh.subscribe(m_name_front_camera + "/image_raw", 1,
                                          &Masters::m_cbk_front_camera, this);
        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer(nh, m_nodename, ros::Duration(1));
        m_transformer.setLookupTimeout(ros::Duration(0.1));

        // | -------------------- initialize timers ------------------- |
        //m_tim_kalman = nh.createTimer(ros::Duration(m_dt),
         //                             &Masters::m_tim_cbk_kalman,
          //                            this);
        // Some additional inits

        m_Q = Eigen::Matrix<double, 3, 3>::Identity();
        m_R = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 6> G;
//        https://www.mdpi.com/2072-4292/13/15/2915

        ROS_INFO_ONCE("[%s]: initialized", m_nodename.c_str());
        m_is_initialized = true;
    }
//}

// | ----------------- dynamic reconf callback ---------------- |
    void Masters::m_cbk_dynrec(masters::DynrecConfig &config, [[maybe_unused]] uint32_t level) {
        Eigen::Matrix<double, 6, 6> G;
//        https://www.mdpi.com/2072-4292/13/15/2915

        ROS_INFO("New dynamic reconfigure values received.");
        m_Q = Eigen::Matrix3d::Identity() * config.s_Q;
        m_R = Eigen::Matrix3d::Identity() * config.s_R;
        m_P0 = Eigen::Matrix<double, 6, 6>::Zero();
        m_P0.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config.s_P0_position;
        m_P0.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * config.s_P0_velocity;
    }

// | ---------------------- msg callbacks --------------------- |

    [[maybe_unused]] void Masters::m_cbk_detection(const geometry_msgs::PointStamped &msg) {

        // find the eagle pose
        auto pt_rect = m_camera_front.rectifyPoint({msg.point.x, msg.point.y});
        cv::Point3d td_ray = m_camera_front.projectPixelTo3dRay(pt_rect);
        auto T_msg_frame2world_opt = m_transformer.transformAsVector(msg.header.frame_id,
                                                                     Eigen::Vector3d{td_ray.x, td_ray.y, td_ray.z},
                                                                     m_name_world_origin,
                                                                     msg.header.stamp);
        if (T_msg_frame2world_opt.has_value()) {

            if (msg.header.stamp - m_last_kalman_time > ros::Duration(m_dt)) {
                const Eigen::Vector3d detection_vec = Eigen::Vector3d{T_msg_frame2world_opt->x(),
                                                  T_msg_frame2world_opt->y(),
                                                  T_msg_frame2world_opt->z()}.normalized();
                update_kalman(detection_vec, msg.header.stamp);
            }
        } else {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR no transformation from " << msg.header.frame_id << " to "
                                 << m_name_world_origin);
        }
    }

    [[maybe_unused]] void Masters::m_cbk_front_camera(const sensor_msgs::ImageConstPtr &msg) {
        if (not m_is_initialized) return;
        const auto detection_opt = m_detect_uav(msg);
        m_name_front_camera_tf = msg->header.frame_id;

        if (detection_opt.has_value()) {
            cv::Point2d detection = detection_opt.value();
            geometry_msgs::PointStamped det_ros_msg;
            det_ros_msg.header = msg->header;
            det_ros_msg.point.x = detection.x;
            det_ros_msg.point.y = detection.y;
            det_ros_msg.point.z = 1;
            m_pub_detection.publish(det_ros_msg);
        } else {
            ROS_ERROR_THROTTLE(1.0, "[%s]: No detection present;", m_nodename.c_str());
            return;
        }
    }

// | --------------------- timer callbacks -------------------- |

    void Masters::update_kalman(
            Eigen::Vector3d detection_vec,
            ros::Time detection_time) {
        if (not m_is_initialized) return;
        ROS_INFO_THROTTLE(5.0, "[%s]: tim kalman start", m_nodename.c_str());
        const double dt = (detection_time - m_last_kalman_time).toSec();
        Eigen::Matrix<double, 6, 6> A;
        A << 1, 0, 0, dt, 0, 0,
             0, 1, 0, 0, dt, 0,
             0, 0, 1, 0, 0, dt,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

        Eigen::Matrix<double, 6, 1> state_interceptor_new;
        if (m_subh_eagle_odom.hasMsg()) {
            const auto msg_eagle_odom = m_subh_eagle_odom.getMsg();
            const auto new_pose_st_opt = m_transformer.transformAsPoint(msg_eagle_odom->header.frame_id,
                                                                  {msg_eagle_odom->pose.pose.position.x,
                                                                   msg_eagle_odom->pose.pose.position.y,
                                                                   msg_eagle_odom->pose.pose.position.z},
                                                                  m_name_world_origin,
                                                                  msg_eagle_odom->header.stamp);
            const auto new_twist_st_opt = m_transformer.transformAsVector(msg_eagle_odom->child_frame_id,
                                                                   {msg_eagle_odom->twist.twist.linear.x,
                                                                    msg_eagle_odom->twist.twist.linear.y,
                                                                    msg_eagle_odom->twist.twist.linear.z},
                                                                   m_name_world_origin,
                                                                   msg_eagle_odom->header.stamp);
            if (new_pose_st_opt.has_value() and new_twist_st_opt.has_value()) {
                state_interceptor_new.segment<3>(0) = new_pose_st_opt.value();
                state_interceptor_new.segment<3>(3) = new_twist_st_opt.value();
            } else {
                ROS_ERROR("[%s]: pose or twist transformation has no value", m_nodename.c_str());
                return;
            }
        } else {
            ROS_ERROR("[%s]: No odometry msg from %s", m_nodename.c_str(), m_name_eagle_odom_msg.c_str());
            return;
        }

        if (m_is_kalman_initialized) {
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman is initialised", m_nodename.c_str());

            // Predict always
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman predict", m_nodename.c_str());
            std::tie(m_x_k, m_P_k) = plkf_predict(m_x_k,
                                                  m_P_k,
                                                  m_state_interceptor,
                                                  state_interceptor_new,
                                                  dt);

            // Correct always
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman correct", m_nodename.c_str());
            std::tie(m_x_k, m_P_k) = plkf_correct(m_x_k, m_P_k, detection_vec);
            m_last_kalman_time = detection_time;

        } else {
            // initialise
            ROS_INFO("[%s]: kalman initialise", m_nodename.c_str());
            const auto opt_est_init = m_transformer.transformAsPoint(m_name_front_camera_tf,
                                                               Eigen::Vector3d{0, 0, 10},
                                                               m_name_world_origin,
                                                               detection_time);
            if (opt_est_init.has_value()) {
                //TODO: initialize only when detection exists
                m_x_k.setZero();
                m_x_k.x() = opt_est_init->x() - state_interceptor_new.x();
                m_x_k.y() = opt_est_init->y() - state_interceptor_new.y();
                m_x_k.z() = opt_est_init->z() - state_interceptor_new.z();
                m_P_k = m_P0;
            } else {
                ROS_ERROR("[%s]: No transformation from %s to %s ", m_nodename.c_str(), m_name_world_origin.c_str(),
                          (m_name_front_camera_tf).c_str());
                return;
            }
            ROS_INFO("[%s]: kalman is initialised", m_nodename.c_str());
            m_is_kalman_initialized = true;
        }

        nav_msgs::Odometry msg;
        msg.header.stamp = detection_time;
        msg.child_frame_id = m_name_world_origin;
        msg.header.frame_id = m_name_world_origin;
        msg.pose.pose.position.x = state_interceptor_new.x() + m_x_k.x();
        msg.pose.pose.position.y = state_interceptor_new.y() + m_x_k.y();
        msg.pose.pose.position.z = state_interceptor_new.z() + m_x_k.z();
        msg.twist.twist.linear.x = state_interceptor_new(3) + m_x_k(3);
        msg.twist.twist.linear.y = state_interceptor_new(4) + m_x_k(4);
        msg.twist.twist.linear.z = state_interceptor_new(5) + m_x_k(5);
        // Set the covariance matrix
        const int n = static_cast<int>(m_x_k.rows());
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.pose.covariance.at(n * i + j) = m_P_k(i, j);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.twist.covariance.at(n * i + j) = m_P_k(i+3, j+3);

        m_pub_target_odom.publish(msg);

        visualization_msgs::Marker marker_predict;
        marker_predict.header.frame_id = m_name_world_origin;
        marker_predict.header.stamp = detection_time;
        marker_predict.ns = "my_namespace";
        marker_predict.id = 0;
        marker_predict.type = visualization_msgs::Marker::SPHERE;
        marker_predict.action = visualization_msgs::Marker::MODIFY;
        marker_predict.pose.position.x = state_interceptor_new.x() + m_x_k.x();
        marker_predict.pose.position.y = state_interceptor_new.y() + m_x_k.y();
        marker_predict.pose.position.z = state_interceptor_new.z() + m_x_k.z();
        marker_predict.scale.x = 0.2;
        marker_predict.scale.y = 0.2;
        marker_predict.scale.z = 0.2;
        marker_predict.color.a = 1.0; // Don't forget to set the alpha!
        marker_predict.color.r = 0.0;
        marker_predict.color.g = 1.0;
        marker_predict.color.b = 0.0;
        m_pub_viz.publish(marker_predict);

        visualization_msgs::Marker marker_detect;
        marker_detect.header.frame_id = m_name_world_origin;
        marker_detect.header.stamp = detection_time;
        marker_detect.ns = "my_namespace_detect";
        marker_detect.id = 1;
        marker_detect.type = visualization_msgs::Marker::ARROW;
        marker_detect.action = visualization_msgs::Marker::MODIFY;
        marker_detect.pose.position.x = 0;
        marker_detect.pose.position.y = 0;
        marker_detect.pose.position.z = 0;
        std::vector<geometry_msgs::Point> points;
        geometry_msgs::Point gpt1, gpt2;
        int cst = 5;
        gpt1.x = state_interceptor_new.x();
        gpt1.y = state_interceptor_new.y();
        gpt1.z = state_interceptor_new.z();
        gpt2.x = state_interceptor_new.x() + detection_vec.x() * cst;
        gpt2.y = state_interceptor_new.y() + detection_vec.y() * cst;
        gpt2.z = state_interceptor_new.z() + detection_vec.z() * cst;
        points.push_back(gpt1);
        points.push_back(gpt2);
        marker_detect.points = points;
        marker_detect.scale.x = 0.1;
        marker_detect.scale.y = 0.2;
        marker_detect.scale.z = 1;

        marker_detect.color.a = 1.0; // Don't forget to set the alpha!
        marker_detect.color.r = 0.0;
        marker_detect.color.g = 1.0;
        marker_detect.color.b = 0.0;
        m_pub_viz.publish(marker_detect);

        m_state_interceptor = state_interceptor_new;
    }

// | -------------------- other functions ------------------- |
    std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>
    Masters::plkf_predict(const Eigen::Matrix<double, 6, 1> &xk_1,
                          const Eigen::Matrix<double, 6, 6> &Pk_1,
                          const Eigen::Matrix<double, 6, 1> &x_i,
                          const Eigen::Matrix<double, 6, 1> &x_i_,
                          const double &dt) {
        Eigen::Matrix<double, 6, 6> A;
        Eigen::Matrix<double, 6, 3> B;

        A << 1, 0, 0, dt, 0, 0,
                0, 1, 0, 0, dt, 0,
                0, 0, 1, 0, 0, dt,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;

        const auto dts = dt * dt / 2;
        B << dts, 0, 0,
                0, dts, 0,
                0, 0, dts,
                dt, 0, 0,
                0, dt, 0,
                0, 0, dt;


        Eigen::Matrix<double, 6, 1> xk_ = A * (xk_1 + x_i_) - x_i;
        Eigen::Matrix<double, 6, 6> Pk_ = A * Pk_1 * A.transpose() + B * m_Q * B.transpose();
        return {xk_, Pk_};
    }

    std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>
    Masters::plkf_correct(const Eigen::Matrix<double, 6, 1> &xk_,
                          const Eigen::Matrix<double, 6, 6> &Pk_,
                          const Eigen::Vector3d &lmb) {
        const Eigen::Matrix3d Plk = Eigen::Matrix3d::Identity() - lmb * lmb.transpose() / lmb.squaredNorm();
        Eigen::Matrix<double, 3, 6> Hk;

        double r_ = xk_.segment<0>(3).norm();
        const Eigen::Matrix3d Vk = r_ * Plk;
        Hk.setZero();
        Hk.block<3, 3>(0, 0) = Plk;

        Eigen::Matrix3d ps = Hk * Pk_ * Hk.transpose() + Vk * m_R * Vk.transpose() * m_dt;
//        Eigen::Matrix3d pinv = Masters::pseudoinverse(ps);
        Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> cod(ps);
        Eigen::Matrix3d pinv = cod.pseudoInverse();

        Eigen::Matrix<double, 6, 3> K = Pk_ * Hk.transpose() * pinv;

        Eigen::Matrix<double, 6, 1> xk1 = xk_ - K * Hk * xk_;
        Eigen::Matrix<double, 6, 6> Pk1 = (Eigen::Matrix<double, 6, 6>::Identity() - K * Hk) * Pk_;
//        std::cout << "===========================" << std::endl;
//        std::cout << Pk_ << std::endl;
//        std::cout << "---------------------------" << std::endl;
//        std::cout << Pk1 << std::endl;
        return {xk1, Pk1};
    }


    std::optional<cv::Point2d> Masters::m_detect_uav(const sensor_msgs::Image::ConstPtr &msg) {
        auto T_eagle2drone_opt = m_transformer.getTransform("uav2/fcu",
                                                            msg->header.frame_id,
                                                            msg->header.stamp);
        if (!T_eagle2drone_opt.has_value()) {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR wrong detection");
            return std::nullopt;
        }
        const geometry_msgs::TransformStamped T_eagle2drone = T_eagle2drone_opt.value();;
        const Eigen::Vector3d dir_vec(tf2::transformToEigen(T_eagle2drone.transform).translation().data());
        if (dir_vec.z() < 0) {
            return std::nullopt;
        }
        const auto pt_ideal = m_camera_front.project3dToPixel(cv::Point3d(dir_vec.x(), dir_vec.y(), dir_vec.z()));
        std::random_device rseed;
        std::mt19937 rng(rseed());
        std::normal_distribution<double> dist(m_mean, m_stddev);
        double e_x = dist(rng), e_y = dist(rng);

        const cv::Point2d pt_noisy{pt_ideal.x + e_x, pt_ideal.y + e_y};
        if (pt_noisy.x < 0 or pt_noisy.x > m_camera_front.cameraInfo().width or
            pt_noisy.y < 0 or pt_noisy.y > m_camera_front.cameraInfo().height) {
            return std::nullopt;
        }
        return pt_noisy;
    }
}  // namespace masters

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(masters::Masters, nodelet::Nodelet)
