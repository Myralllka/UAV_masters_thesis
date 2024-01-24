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

        m_name_front_camera_tf = "uav91/bluefox_front";

        pl.loadParam("front_camera", m_name_front_camera);
        pl.loadParam("mean", m_mean);
        pl.loadParam("deviation", m_stddev);
        pl.loadParam("eagle_odometry", m_name_eagle_odom_msg);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", m_nodename.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", m_nodename.c_str());
        }
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

        // | -------------------- initialize timers ------------------- |
        m_tim_kalman = nh.createTimer(ros::Duration(m_dt),
                                      &Masters::m_tim_cbk_kalman,
                                      this);
        // Some additional inits

        m_Q = Eigen::Matrix3d::Identity() * 0.1;
        m_R = Eigen::Matrix3d::Identity() * 10;

        ROS_INFO_ONCE("[%s]: initialized", m_nodename.c_str());
        m_is_initialized = true;
    }
//}

// | ----------------- dynamic reconf callback ---------------- |
    void Masters::m_cbk_dynrec(masters::DynrecConfig &config, [[maybe_unused]] uint32_t level) {
        m_Q = Eigen::Matrix3d::Identity() * config.s_Q;
        m_R = Eigen::Matrix3d::Identity() * config.s_R;
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

            std::lock_guard<std::mutex> lt(m_detection_mut);
            m_detection_vec = Eigen::Vector3d{T_msg_frame2world_opt->x(),
                                              T_msg_frame2world_opt->y(),
                                              T_msg_frame2world_opt->z()}.normalized();
            m_detecton_time = msg.header.stamp;
        } else {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR no transformation from " << msg.header.frame_id << " to "
                                 << m_name_world_origin);
        }
    }

    [[maybe_unused]] void Masters::m_cbk_front_camera(const sensor_msgs::ImageConstPtr &msg) {
        if (not m_is_initialized) return;
        const auto detection_opt = m_detect_uav(msg);

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

    void Masters::m_tim_cbk_kalman(const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;
        ROS_INFO("[%s]: tim kalman start", m_nodename.c_str());

        Eigen::Matrix<double, 6, 6> A;
        A << 1, 0, 0, m_dt, 0, 0,
                0, 1, 0, 0, m_dt, 0,
                0, 0, 1, 0, 0, m_dt,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
        ROS_INFO("[%s]: A mat", m_nodename.c_str());

        // Find the drone's position in worlds coordinate frame
        auto T_eagle2world_opt = m_transformer.getTransform(m_name_front_camera_tf,
                                                            m_name_world_origin,
                                                            ros::Time(0)); // TODO: use real time
//        std::cout << "header stamp: " << T_eagle2world_opt->header.stamp << "; " << ev.current_real << "; "
//                  << T_eagle2world_opt->header.stamp - ev.current_real;
        Eigen::Vector3d p1;
        if (T_eagle2world_opt.has_value()) {
            ROS_INFO("[%s]: T_eagle has value", m_nodename.c_str());
            auto T_eagle2world = T_eagle2world_opt.value().transform.translation;
            p1.x() = T_eagle2world.x;
            p1.y() = T_eagle2world.y;
            p1.z() = T_eagle2world.z;
        } else {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR no transformation from world to eagle");
            return;
        }
        Eigen::Matrix<double, 6, 1> state_interceptor_new;

        if (m_is_kalman_initialized) {
            ROS_INFO("[%s]: kalman is initialised", m_nodename.c_str());
            // New position and new velocity

            if (m_subh_eagle_odom.hasMsg()) {
                const auto msg_ = m_subh_eagle_odom.getMsg().get();
                const auto position = msg_->pose.pose.position;
                const auto velocity = msg_->twist.twist.linear;
                state_interceptor_new.segment<3>(0) = Eigen::Vector3d{position.x, position.y, position.z};
                state_interceptor_new.segment<3>(3) = Eigen::Vector3d{velocity.x, velocity.y, velocity.z};
            } else {
                ROS_ERROR("[%s]: No odometry msg from %s", m_nodename.c_str(), m_name_eagle_odom_msg.c_str());
                state_interceptor_new.segment<3>(0) = p1;
                state_interceptor_new.segment<3>(3) = (p1 - m_state_interceptor.segment<3>(0)) / m_dt;

            }
            // Predict always
            ROS_INFO("[%s]: kalman predict", m_nodename.c_str());
            std::tie(m_x_k, m_P_k) = plkf_predict(m_x_k,
                                                  m_P_k,
                                                  m_state_interceptor,
                                                  state_interceptor_new,
                                                  m_dt);

            // Correct if possible
            if (m_last_kalman_time < m_detecton_time) {
                ROS_INFO("[%s]: kalman correct", m_nodename.c_str());
                std::lock_guard<std::mutex> lt(m_detection_mut);
                std::tie(m_x_k, m_P_k) = plkf_correct(m_x_k, m_P_k, m_detection_vec);
                m_last_kalman_time = ev.current_real;
            }

        } else {
            // initialise
            ROS_INFO("[%s]: kalman initialise", m_nodename.c_str());
            // TODO: use odometry
            state_interceptor_new.segment<3>(0) = p1;
            state_interceptor_new.segment<3>(3) = Eigen::Vector3d::Ones();
            //TODO: initialize only when detection exists
            m_x_k = state_interceptor_new;
            m_P_k = Eigen::Matrix<double, 6, 6>::Identity() * 100; // TODO: parametrize

            ROS_INFO("[%s]: kalman is initialised", m_nodename.c_str());
            m_is_kalman_initialized = true;
        }

        nav_msgs::Odometry msg;
        msg.header.stamp = ev.current_expected;
        msg.header.frame_id = m_name_world_origin;
        msg.pose.pose.position.x = m_state_interceptor.x() + m_x_k.x();
        msg.pose.pose.position.y = m_state_interceptor.y() + m_x_k.y();
        msg.pose.pose.position.z = m_state_interceptor.z() + m_x_k.z();
        msg.twist.twist.linear.x = m_x_k(3);
        msg.twist.twist.linear.x = m_x_k(4);
        msg.twist.twist.linear.x = m_x_k(5);
        // Set the covariance matrix
        for (int i = 0; i < 9; ++i) {
            msg.pose.covariance[i] = m_P_k(i / 3, i % 3);
            msg.twist.covariance[i] = m_P_k((i / 3) + 3, (i % 3) + 3);
        }
        m_pub_target_odom.publish(msg);

        visualization_msgs::Marker marker_predict;
        marker_predict.header.frame_id = m_name_world_origin;
        marker_predict.header.stamp = ev.current_expected;
        marker_predict.ns = "my_namespace";
        marker_predict.id = 0;
        marker_predict.type = visualization_msgs::Marker::SPHERE;
        marker_predict.action = visualization_msgs::Marker::MODIFY;
        marker_predict.pose.position.x = m_state_interceptor.x() + m_x_k.x();
        marker_predict.pose.position.y = m_state_interceptor.y() + m_x_k.y();
        marker_predict.pose.position.z = m_state_interceptor.z() + m_x_k.z();
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
        marker_detect.header.stamp = ev.current_expected;
        marker_detect.ns = "my_namespace_detect";
        marker_detect.id = 1;
        marker_detect.type = visualization_msgs::Marker::ARROW;
        marker_detect.action = visualization_msgs::Marker::MODIFY;
        marker_detect.pose.position.x = m_state_interceptor.x();
        marker_detect.pose.position.y = m_state_interceptor.y();
        marker_detect.pose.position.z = m_state_interceptor.z();
        std::vector<geometry_msgs::Point> points;
        geometry_msgs::Point gpt1, gpt2;
        int cst = 5;
        gpt1.x = 0;
        gpt1.y = 0;
        gpt1.z = 0;
        gpt2.x = m_detection_vec.x() * cst;
        gpt2.y = m_detection_vec.y() * cst;
        gpt2.z = m_detection_vec.z() * cst;
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
        const Eigen::Matrix3d Plk = Eigen::Matrix3d::Identity() - lmb * lmb.transpose() / (lmb.norm() * lmb.norm());
        Eigen::Matrix<double, 3, 6> Hk;

        double r_ = xk_.segment<0>(3).norm();
        const Eigen::Matrix3d Vk = r_ * Plk;
        Hk.setZero();
        Hk.block<3, 3>(0, 0) = Plk;

        Eigen::Matrix3d ps = Hk * Pk_ * Hk.transpose() + Vk * m_R * Vk.transpose();
        Eigen::Matrix3d pinv = Masters::pseudoinverse(ps);

        Eigen::Matrix<double, 6, 3> K = Pk_ * Hk.transpose() * pinv;

        Eigen::Matrix<double, 6, 1> xk1 = xk_ - K * Hk * xk_;
        Eigen::Matrix<double, 6, 6> Pk1 = (Eigen::Matrix<double, 6, 6>::Identity() - K * Hk) * Pk_;
        return {xk1, Pk1};
    }


    std::optional<cv::Point2d> Masters::m_detect_uav(const sensor_msgs::Image::ConstPtr &msg) {
        geometry_msgs::TransformStamped T_eagle2drone;
        auto T_eagle2drone_opt = m_transformer.getTransform("uav2/fcu",
                                                            "uav91/bluefox_front_optical",
                                                            msg->header.stamp);
        if (T_eagle2drone_opt.has_value()) {
            T_eagle2drone = T_eagle2drone_opt.value();
        } else {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR wrong detection");
            return {};
        }
        Eigen::Vector3d vec = Eigen::Vector3d(tf2::transformToEigen(T_eagle2drone.transform).translation().data());
        if (vec.z() < 0) {
            return {};
        }
        auto pt_ideal = m_camera_front.project3dToPixel(cv::Point3d(vec.x(), vec.y(), vec.z()));
        std::random_device rseed;
        std::mt19937 rng(rseed());
        std::normal_distribution<double> dist(m_mean, m_stddev);
        double e_x = dist(rng), e_y = dist(rng);

        cv::Point2d pt_noisy{pt_ideal.x + e_x, pt_ideal.y + e_y};
        if (pt_noisy.x < 0 or pt_noisy.x > m_camera_front.cameraInfo().width or
            pt_noisy.y < 0 or pt_noisy.y > m_camera_front.cameraInfo().height) {
            return {};
        }
        return pt_noisy;
    }
}  // namespace masters

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(masters::Masters, nodelet::Nodelet)
