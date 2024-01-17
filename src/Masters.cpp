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

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", m_nodename.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", m_nodename.c_str());
        }
        // | ---------------- some data post-processing --------------- |
        m_history_velocity = t_hist_vvt(m_history_bufsize);

        // | ----------------- publishers initialize ------------------ |
        m_pub_image_changed = nh.advertise<sensor_msgs::Image>("changed", 1);
        m_pub_detection = nh.advertise<geometry_msgs::PointStamped>("detection", 1);
        m_pub_history1 = nh.advertise<geometry_msgs::PoseArray>(m_name_eagle + "/history_1", 1);
        m_pub_history2 = nh.advertise<geometry_msgs::PoseArray>(m_name_eagle + "/history_2", 1);
        m_pub_viz = nh.advertise<visualization_msgs::Marker>(m_name_eagle + "/detection", 1);

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
        while (ros::ok()) {
            if (handler_camfrontinfo.hasMsg()) {
                camfront_info = *handler_camfrontinfo.getMsg().get();
                break;
            }
        }
        m_t0 = ros::Time::now();

        m_camera_front.fromCameraInfo(camfront_info);

        m_sub_detection = nh.subscribe("detection", 1,
                                       &Masters::m_cbk_detection, this);
        m_sub_front_camera = nh.subscribe(m_name_front_camera + "/image_raw", 1,
                                          &Masters::m_cbk_front_camera, this);
        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer(nh, m_nodename);

        // | -------------------- initialize timers ------------------- |
        m_tim_kalman = nh.createTimer(ros::Duration(m_dt),
                                      &Masters::m_tim_cbk_kalman,
                                      this);

        ROS_INFO_ONCE("[%s]: initialized", m_nodename.c_str());
        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |

    [[maybe_unused]] void Masters::m_cbk_detection(const geometry_msgs::PointStamped &msg) {

        // find the eagle pose
        auto T_eagle2world_opt = m_transformer.getTransform(m_name_front_camera_tf,
                                                            m_name_world_origin,
                                                            msg.header.stamp);
        Eigen::Vector3d p_eagle;
        if (T_eagle2world_opt.has_value()) {
            auto T_eagle2world = T_eagle2world_opt.value().transform.translation;
            p_eagle.x() = T_eagle2world.x;
            p_eagle.y() = T_eagle2world.y;
            p_eagle.z() = T_eagle2world.z;
        } else {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR no transformation from eagle to world");
        }
        cv::Point3d td_ray = m_camera_front.projectPixelTo3dRay({msg.point.x, msg.point.y});

        auto T_optical2normal_opt = m_transformer.transformAsVector(m_name_front_camera_tf,
                                                                    Eigen::Vector3d{td_ray.x, td_ray.y, td_ray.z},
                                                                    m_name_front_camera_tf + "_optical",
                                                                    msg.header.stamp);
        if (T_optical2normal_opt.has_value()) {
            auto t_normal = T_optical2normal_opt.value();
            {
                std::lock_guard<std::mutex> lt(m_detection_mut);
                m_detection_vec = Eigen::Vector3d{t_normal.x(),
                                                  t_normal.y(),
                                                  t_normal.z()};
                std::cout << m_detection_vec << std::endl;
                m_detecton_time = msg.header.stamp;
            }
        } else {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR no transformation from eagle to world");
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
            state_interceptor_new.segment<3>(0) = p1;
            state_interceptor_new.segment<3>(3) = (p1 - m_state_interceptor.segment<3>(0)) / m_dt;

            // Correct if possible
            if (m_last_kalman_time < m_detecton_time) {
                ROS_INFO("[%s]: kalman correct", m_nodename.c_str());
                std::lock_guard<std::mutex> lt(m_detection_mut);
                std::tie(m_x_k, m_P_k) = plkf_correct(m_x_k, m_P_k, m_detection_vec);
                m_last_kalman_time = ev.current_real;
            }

            // Predict always
            ROS_INFO("[%s]: kalman predict", m_nodename.c_str());
            std::tie(m_x_k, m_P_k) = plkf_predict(m_x_k,
                                                  m_P_k,
                                                  m_state_interceptor,
                                                  state_interceptor_new,
                                                  m_dt);
        } else {
            // initialise
            ROS_INFO("[%s]: kalman initialise", m_nodename.c_str());
            state_interceptor_new.segment<3>(0) = p1;
            state_interceptor_new.segment<3>(3) = Eigen::Vector3d::Ones();
            m_x_k = state_interceptor_new;
            m_P_k = Eigen::Matrix<double, 6, 6>::Identity();
            ROS_INFO("[%s]: kalman is initialised", m_nodename.c_str());
            m_is_kalman_initialized = true;
        }


        visualization_msgs::Marker marker_predict, marker_detect;
        marker_predict.header.frame_id = m_name_world_origin;
        marker_predict.header.stamp = ev.current_real;
        marker_predict.ns = "my_namespace";
        marker_predict.id = 0;
        marker_predict.type = visualization_msgs::Marker::SPHERE;
        marker_predict.action = visualization_msgs::Marker::MODIFY;
        marker_predict.pose.position.x = m_state_interceptor.x() + m_x_k.x();
        marker_predict.pose.position.y = m_state_interceptor.y() + m_x_k.y();
        marker_predict.pose.position.z = m_state_interceptor.z() + m_x_k.z();
        marker_predict.scale.x = 1;
        marker_predict.scale.y = 1;
        marker_predict.scale.z = 1;
        marker_predict.color.a = 1.0; // Don't forget to set the alpha!
        marker_predict.color.r = 0.0;
        marker_predict.color.g = 1.0;
        marker_predict.color.b = 0.0;
        m_pub_viz.publish(marker_predict);

        marker_detect.header.frame_id = m_name_world_origin;
        marker_detect.header.stamp = ev.current_real;
        marker_detect.ns = "my_namespace_detect";
        marker_detect.id = 1;
        marker_detect.type = visualization_msgs::Marker::LINE_STRIP;
        marker_detect.action = visualization_msgs::Marker::MODIFY;
        marker_detect.pose.position.x = m_state_interceptor.x();
        marker_detect.pose.position.y = m_state_interceptor.y();
        marker_detect.pose.position.z = m_state_interceptor.z();
        std::vector<geometry_msgs::Point> points;
        geometry_msgs::Point gpt1, gpt2;
        gpt1.x = m_state_interceptor.x();
        gpt1.y = m_state_interceptor.y();
        gpt1.z = m_state_interceptor.z();
        gpt2.x = m_state_interceptor.x() + m_detection_vec.x();
        gpt2.y = m_state_interceptor.y() + m_detection_vec.y();
        gpt2.z = m_state_interceptor.z() + m_detection_vec.z();
        points.push_back(gpt1);
        points.push_back(gpt2);
        marker_detect.points = points;
        marker_detect.scale.x = 1;
        marker_detect.scale.y = 1;
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
        double sigma_squared = 100; // TODO: parametrise
        Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * sigma_squared;
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
        Eigen::Matrix<double, 6, 6> Pk_ = A * Pk_1 * A.transpose() + B * Q * B.transpose();
        return {xk_, Pk_};
    }

    std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>
    Masters::plkf_correct(const Eigen::Matrix<double, 6, 1> &xk_,
                          const Eigen::Matrix<double, 6, 6> &Pk_,
                          const Eigen::Vector3d &lmb) {
        double sigma_squared = 1; // TODO: parametrise
        Eigen::Matrix3d Plk = Eigen::Matrix3d::Identity() - lmb * lmb.transpose() / (lmb.norm() * lmb.norm()), Vk;
        Eigen::Matrix<double, 3, 6> Hk;
        double r_ = xk_.segment<0>(3).norm();
        Vk = r_ * Plk;
        Hk.setZero();
        Hk.block<3, 3>(0, 0) = Plk;
        Eigen::Matrix3d S = Eigen::Matrix3d::Identity() * sigma_squared;
        Eigen::Matrix3d ps = Hk * Pk_ * Hk.transpose() + Vk * S * Vk.transpose();
        Eigen::Matrix3d pinv = Masters::pseudoinverse(ps);

        Eigen::Matrix<double, 6, 3> K = Pk_ * Hk.transpose() * pinv;

        Eigen::Matrix<double, 6, 1> xk1 = xk_ - K * Hk * xk_;
        Eigen::Matrix<double, 6, 6> Pk1 = (Eigen::Matrix<double, 6, 6>::Identity() - K * Hk) * Pk_;
        return {xk1, Pk1};
    }


//    std::pair<vec3, vec3> Masters::m_find_intersection_svd_static_velocity_obj(const t_hist_vvt &data) {
//        const int n_pts = data.size();
//        const int n_r = 3 * n_pts;
//        const int n_c = 6 + n_pts;
//
//        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_r, n_c);
//        Eigen::VectorXd b(n_r);
//
//        int r_idx = 0;
//        for (const auto &pair: data) {
//            const vec3 &Os = std::get<0>(pair);
//            const vec3 &Ds = std::get<1>(pair);
//            const ros::Time t = std::get<2>(pair);
//
//            const vec3 ks = Ds - Os;
//
//            A.block<3, 3>(r_idx, 0) = Eigen::Matrix3d::Identity();
//            A.block<3, 3>(r_idx, 3) = Eigen::Matrix3d::Identity() * (t - m_t0).toSec();
//            A.block<3, 1>(r_idx, 6 + r_idx / 3) = ks;
//            b.segment<3>(r_idx) = Os;
//
//            r_idx += 3;
//        }
////        Eigen::BDCSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
//        Eigen::JacobiSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
//
//        const Eigen::VectorXd x = SVD.solve(b);
//        // Extract the first 3 elements of x to get the intersection point.
//        const vec3 intersection = x.segment<3>(0);
//        const vec3 speed = x.segment<3>(3);
//
//        return {intersection, speed};
//    }

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
        // TODO: check for image boundaries

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
