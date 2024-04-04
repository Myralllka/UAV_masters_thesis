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
        pl.loadParam("approach", m_approach);
        pl.loadParam("target_name", m_name_target);
        pl.loadParam("world_origin", m_name_world_origin);

        pl.loadParam("front_camera", m_name_front_camera);
        pl.loadParam("mean", m_mean);
        pl.loadParam("dt_kalman", m_dt);
        pl.loadParam("deviation", m_stddev);
        pl.loadParam("eagle_odometry", m_name_eagle_odom_msg);
        pl.loadParam("correction_th", m_correction_th);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", m_nodename.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", m_nodename.c_str());
        }

        // Dynamic reconfigure
        server.setCallback(boost::bind(&Masters::m_cbk_dynrec, this, _1, _2));

        // | ---------------- some data post-processing --------------- |

        // | ----------------- publishers initialize ------------------ |
        m_pub_image_changed = nh.advertise<sensor_msgs::Image>("changed", 1);
        m_pub_detection = nh.advertise<geometry_msgs::PointStamped>("detection", 1);
        m_pub_history1 = nh.advertise<geometry_msgs::PoseArray>(m_name_eagle + "/history_1", 1);
        m_pub_history2 = nh.advertise<geometry_msgs::PoseArray>(m_name_eagle + "/history_2", 1);
        m_pub_viz = nh.advertise<visualization_msgs::Marker>("detection_marker", 1);
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

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer(nh, m_nodename, ros::Duration(1));
        m_transformer.setLookupTimeout(ros::Duration(0.1));


        m_t0 = ros::Time::now();
        m_camera_front.fromCameraInfo(camfront_info);
        ROS_INFO("[%s]: approach used: %s", m_nodename.c_str(), m_approach.c_str());
        if (m_approach == "dkf") {
            m_sub_detection = nh.subscribe("detection", 1,
                                           &Masters::m_cbk_detection, this);
            m_sub_front_camera = nh.subscribe(m_name_front_camera + "/image_raw", 1,
                                              &Masters::m_cbk_front_camera, this);
        } else if (m_approach == "plkf") {

        } else if (m_approach == "svd_static") {
            m_sub_front_camera_detection = nh.subscribe(m_name_front_camera + "/detection", 1,
                                                        &Masters::m_cbk_posearray_svd_pos_vel, this);
            m_sub_front_camera = nh.subscribe(m_name_front_camera + "/image_raw", 1,
                                              &Masters::m_cbk_camera_image_to_detection_svd, this);

        } else if (m_approach == "svd_dynamic") {
            m_sub_front_camera_detection = nh.subscribe(m_name_front_camera + "/detection", 1,
                                                        &Masters::m_cbk_posearray_svd_pos_vel, this);
            m_sub_front_camera = nh.subscribe(m_name_front_camera + "/image_raw", 1,
                                              &Masters::m_cbk_camera_image_to_detection_svd, this);

        } else {
            ROS_ERROR("[%s]: unknown m_aproach", m_nodename.c_str());
        }

        // | -------------------- initialize timers ------------------- |
        // Some additional inits
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
        m_Q = Eigen::Matrix<double, 3, 3>::Identity() * config.s_Q_acc;
        m_R = Eigen::Matrix3d::Identity() * config.s_R;
        m_state_vec.P = Eigen::Matrix<double, 6, 6>::Zero();
        m_state_vec.P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config.s_P0_position;
        m_state_vec.P.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * config.s_P0_velocity;
        m_P0 = Eigen::Matrix<double, 6, 6>::Zero();
        m_P0.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config.s_P0_position;
        m_P0.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * config.s_P0_velocity;
        m_line_variance = config.dkf_var;
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


    [[maybe_unused]] void Masters::m_cbk_camera_image_to_detection_svd(const sensor_msgs::ImageConstPtr &msg) {

        if (not m_is_initialized) return;
        ROS_INFO_THROTTLE(1.0, "[%s]: SVD detection start %s", m_nodename.c_str(), m_approach.c_str());
//        TODO: rewrite to align it with  m_cbk_front_camera
        const auto detection_opt = m_detect_uav(msg);
        cv::Point2d detection;
        if (detection_opt.has_value()) {
            detection = detection_opt.value();
        } else {
            ROS_ERROR_THROTTLE(1.0, "[%s]: No detection present;", m_nodename.c_str());
            return;
        }
        // in the Camera optical frame
        cv::Point3d ray_to_detection_original = m_camera_front.projectPixelTo3dRay(detection);
        // check for NaNs
        if (ray_to_detection_original != ray_to_detection_original) {
            return;
        }
        if (ray_to_detection_original.z < 0) {
            ROS_WARN_THROTTLE(1.0, "[%s]: no target detected", m_nodename.c_str());
            return;
        }
        ray_to_detection_original *= 2;
        // transform to the static world frame
        auto ray_opt = m_transformer.transformAsPoint("uav91/bluefox_front_optical",
                                                      vec3{ray_to_detection_original.x,
                                                           ray_to_detection_original.y,
                                                           ray_to_detection_original.z},
                                                      m_name_world_origin,
                                                      msg->header.stamp);
        cv::Point3d ray_to_detection;
        if (ray_opt.has_value()) {
            ray_to_detection = {ray_opt.value().x(),
                                ray_opt.value().y(),
                                ray_opt.value().z()};
        } else {
            return;
        }

        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // visualize detection
        cv::circle(image, detection, 20, cv::Scalar(255, 0, 0), 2);
        m_pub_image_changed.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());


        // find the eagle Pose and direction vector of detection, for the pose reconstruction
        auto T_eagle2world_opt = m_transformer.getTransform("uav91/bluefox_front_optical",
                                                            m_name_world_origin,
                                                            msg->header.stamp);

        geometry_msgs::Pose p1, p2;
        if (T_eagle2world_opt.has_value()) {
            auto T_eagle2world = T_eagle2world_opt.value().transform.translation;
            p1.position.x = T_eagle2world.x;
            p1.position.y = T_eagle2world.y;
            p1.position.z = T_eagle2world.z;
        } else {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR no transformation from eagle to world");
        }
        p2.position.x = ray_to_detection.x;
        p2.position.y = ray_to_detection.y;
        p2.position.z = ray_to_detection.z;

        geometry_msgs::PoseArray res;
        res.header.stamp = msg->header.stamp;
        res.header.frame_id = m_name_world_origin;
        res.poses.push_back(p1);
        res.poses.push_back(p2);
        m_pub_front_camera_detection.publish(res);
    }
// | --------------------- timer callbacks -------------------- |

    void Masters::update_kalman(Eigen::Vector3d detection_vec, ros::Time detection_time) {
        if (not m_is_initialized) return;
        ROS_INFO_THROTTLE(5.0, "[%s]: kalman cbk start", m_nodename.c_str());
        if (m_last_kalman_time == ros::Time(0)) {
            m_last_kalman_time = detection_time;
            return;
        }
        const double dt = (detection_time - m_last_kalman_time).toSec();
        m_last_kalman_time = detection_time;

        dkf_t::A_t A = dkf_t::A_t::Identity();
        Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Identity();
        A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
        B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt * dt / 2;
        B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
        m_dkf.A = A;
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
            dkf_t::u_t u;
            u.setZero();

            Eigen::Matrix<double, 6, 6> Q = B * m_Q * B.transpose();

            try {
                m_state_vec = m_dkf.predict(m_state_vec, u, Q, dt);
            }
            catch (const std::exception &e) {
                ROS_ERROR("DKF predict failed: %s", e.what());
            }

            // Correct based on the distance moved
            if ((state_interceptor_new.segment<3>(0) - m_position_last_correction).norm() >= m_correction_th) {
                m_cnt_update = 0;
                ROS_INFO_THROTTLE(5.0, "[%s]: kalman correct", m_nodename.c_str());

                try {
                    m_state_vec = m_dkf.correctLine(m_state_vec,
                                                    state_interceptor_new.segment<3>(0),
                                                    detection_vec,
                                                    m_line_variance);
                }
                catch (const std::exception &e) {
                    ROS_ERROR("DKF correct failed: %s", e.what());
                }
                m_position_last_correction = state_interceptor_new.segment<3>(0);
            }


        } else {
            // initialise
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman initialise", m_nodename.c_str());
            const auto opt_est_init = m_transformer.transformAsPoint(m_name_front_camera_tf,
                                                                     Eigen::Vector3d{0, 0, 10},
                                                                     m_name_world_origin,
                                                                     detection_time);
            if (opt_est_init.has_value()) {
                m_dkf = dkf_t(A, dkf_t::B_t::Zero());

                Eigen::Matrix<double, 6, 1> x_k;
                x_k.setZero();
                x_k.x() = opt_est_init->x();
                x_k.y() = opt_est_init->y();
                x_k.z() = opt_est_init->z();
                x_k[3] = 0;
                x_k[4] = 0;
                x_k[5] = 0;
                m_state_vec.x = x_k;
                m_state_vec.P = m_P0;
                m_position_last_correction = x_k.head(3);

            } else {
                ROS_ERROR("[%s]: No transformation from %s to %s ",
                          m_nodename.c_str(),
                          m_name_world_origin.c_str(),
                          (m_name_front_camera_tf).c_str());
                return;
            }
            ROS_INFO("[%s]: kalman is initialised", m_nodename.c_str());
            m_is_kalman_initialized = true;
        }

        const Eigen::Matrix<double, 6, 1> m_x_k = m_state_vec.x;
        const Eigen::Matrix<double, 6, 6> m_P_k = m_state_vec.P;

        visualise_odometry(m_x_k, m_P_k, detection_time);
        visualise_arrow(state_interceptor_new.head(3),
                        state_interceptor_new.head(3) + detection_vec,
                        detection_time);
        visualise_sphere(state_interceptor_new.head(3) + detection_vec, detection_time);

        m_state_interceptor = state_interceptor_new;
    }

// | -------------------- other functions ------------------- |
    std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>
    Masters::plkf_predict(const Eigen::Matrix<double, 6, 1> &xk_1,
                          const Eigen::Matrix<double, 6, 6> &Pk_1,
                          const Eigen::Matrix<double, 6, 1> &x_i,
                          const Eigen::Matrix<double, 6, 1> &x_i_,
                          const double &dt) {
        Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Identity();
        A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
        B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt * dt / 2;
        B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;

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

        double r_ = xk_.head(3).norm();
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
        return {xk1, Pk1};
    }

    [[maybe_unused]] void Masters::m_cbk_posearray_svd_pos_vel(const geometry_msgs::PoseArray &msg) {
        ROS_INFO_THROTTLE(1.0, "[%s]: SVD start %s", m_nodename.c_str(), m_approach.c_str());
        vec3 pos_origin = vec3(msg.poses.at(0).position.x,
                               msg.poses.at(0).position.y,
                               msg.poses.at(0).position.z);
        vec3 pos_s = vec3(msg.poses.at(1).position.x,
                          msg.poses.at(1).position.y,
                          msg.poses.at(1).position.z);

        geometry_msgs::PoseArray ps1, ps2;
        ps1.header.stamp = msg.header.stamp;
        ps2.header.stamp = msg.header.stamp;
        ps1.header.frame_id = m_name_world_origin;
        ps2.header.frame_id = m_name_world_origin;
        for (const auto &h: m_history_velocity) {
            geometry_msgs::Pose p;
            const vec3 v1 = std::get<0>(h);
            const vec3 v2 = std::get<1>(h);
            p.position.x = v1.x();
            p.position.y = v1.y();
            p.position.z = v1.z();
            ps1.poses.push_back(p);
            geometry_msgs::Pose p2;
            p2.position.x = v2.x();
            p2.position.y = v2.y();
            p2.position.z = v2.z();
            ps2.poses.push_back(p2);
        }
        m_pub_history1.publish(ps1);
        m_pub_history2.publish(ps2);

        if (m_history_velocity.size() <= 1) {
            m_history_linear.push_back(std::pair{pos_origin, pos_s});
            m_history_velocity.push_back(std::tuple{pos_origin, pos_s, msg.header.stamp});
            return;
        }
        if ((pos_origin - std::get<0>(m_history_velocity.back())).norm() > m_correction_th) {
            m_history_linear.push_back(std::pair{pos_origin, pos_s});
            m_history_velocity.push_back(std::tuple{pos_origin, pos_s, msg.header.stamp});
        }
        if (m_history_velocity.size() >= 4) {
            vec3 position, velocity;
            if (m_approach == "svd_dynamic") {
                std::tie(position, velocity) = m_find_intersection_svd_velocity_obj(m_history_velocity);
                std::cout << position << std::endl;
                std::cout << position << std::endl;
                velocity = {0, 0, 0};
            } else if (m_approach == "svd_static") {
                position = m_find_intersection_svd_static_obj(m_history_linear);
                std::cout << position << std::endl;
                velocity = {0, 0, 0};
            } else {
                ROS_ERROR("[%s] unknown approach; shutting down...", m_nodename.c_str());
                // to avoid Wall unitialized
                position = {0, 0, 0};
                velocity = {0, 0, 0};
                ros::shutdown();
            }

            const auto time_now = msg.header.stamp;
            const double x = position.x() + velocity.x() * (time_now - m_t0).toSec();
            const double y = position.y() + velocity.y() * (time_now - m_t0).toSec();
            const double z = position.z() + velocity.z() * (time_now - m_t0).toSec();
            Eigen::Matrix<double, 6, 1> state;
            state << x, y, z, velocity.x(), velocity.y(), velocity.z();
            visualise_odometry(state,
                               Eigen::Matrix<double, 6, 6>::Identity(),
                               msg.header.stamp);
            visualise_sphere(state.head(3), msg.header.stamp);

//            visualization_msgs::Marker marker;
//            marker.header.frame_id = m_name_world_origin;
//            marker.header.stamp = msg.header.stamp;
//            marker.ns = "my_namespace";
//            marker.id = 0;
//            marker.type = visualization_msgs::Marker::SPHERE;
//            marker.action = visualization_msgs::Marker::MODIFY;
//
//            const auto time_now = msg.header.stamp;
//            marker.pose.position.x = position.x() + velocity.x() * (time_now - m_t0).toSec();
//            marker.pose.position.y = position.y() + velocity.y() * (time_now - m_t0).toSec();
//            marker.pose.position.z = position.z() + velocity.z() * (time_now - m_t0).toSec();
//
//            marker.scale.x = 1;
//            marker.scale.y = 1;
//            marker.scale.z = 1;
//
//            marker.color.a = 1.0; // Don't forget to set the alpha!
//            marker.color.r = 0.0;
//            marker.color.g = 1.0;
//            marker.color.b = 0.0;
//
//            m_pub_viz.publish(marker);
        }

    }


// | --------------------- timer callbacks -------------------- |


// | -------------------- other functions ------------------- |

    [[maybe_unused]] vec3
    Masters::m_find_intersection_svd_static_obj(const t_hist_vv &data) {

        const int n_pts = data.size();
        const int n_r = 3 * n_pts;
        const int n_c = 3 + n_pts;

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_r, n_c);
        // A.fill(0);
        Eigen::VectorXd b(n_r);

        int r_idx = 0;
        for (const auto &pair: data) {
            const Eigen::Vector3d &Os = pair.first;
            const Eigen::Vector3d &Ds = pair.second;

            Eigen::Vector3d ks = Ds - Os;

            A.block<3, 3>(r_idx, 0) = Eigen::Matrix3d::Identity();
            A.block<3, 1>(r_idx, 3 + r_idx / 3) = -ks;
            b.segment<3>(r_idx) = Os;

            r_idx += 3;
        }
        Eigen::BDCSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

        Eigen::VectorXd x = SVD.solve(b);
        // Extract the first 3 elements of x to get the intersection point.
        Eigen::Vector3d intersection = x.head<3>();

        return intersection;
    }

    std::pair<vec3, vec3> Masters::m_find_intersection_svd_velocity_obj(const t_hist_vvt &data) {
        const int n_pts = data.size();
        const int n_r = 3 * n_pts;
        const int n_c = 6 + n_pts;

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_r, n_c);
        Eigen::VectorXd b(n_r);

        int r_idx = 0;
        for (const auto &pair: data) {
            const vec3 &Os = std::get<0>(pair);
            const vec3 &Ds = std::get<1>(pair);
            const ros::Time t = std::get<2>(pair);

            const vec3 ks = Ds - Os;

            A.block<3, 3>(r_idx, 0) = Eigen::Matrix3d::Identity();
            A.block<3, 3>(r_idx, 3) = Eigen::Matrix3d::Identity() * (t - m_t0).toSec();
            A.block<3, 1>(r_idx, 6 + r_idx / 3) = ks;
            b.segment<3>(r_idx) = Os;

            r_idx += 3;
        }
//        Eigen::BDCSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

        const Eigen::VectorXd x = SVD.solve(b);
        // Extract the first 3 elements of x to get the intersection point.
        const vec3 intersection = x.segment<3>(0);
        const vec3 speed = x.segment<3>(3);

        return {intersection, speed};
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

    void Masters::visualise_odometry(const Eigen::Matrix<double, 6, 1> state,
                                     const Eigen::Matrix<double, 6, 6> covariance,
                                     const ros::Time &t) {
        nav_msgs::Odometry msg;
        msg.header.stamp = t;
        msg.child_frame_id = m_name_world_origin;
        msg.header.frame_id = m_name_world_origin;
        msg.pose.pose.position.x = state(0);
        msg.pose.pose.position.y = state(1);
        msg.pose.pose.position.z = state(2);
        msg.twist.twist.linear.x = state(3);
        msg.twist.twist.linear.y = state(4);
        msg.twist.twist.linear.z = state(5);
        // Set the covariance matrix
        const int n = static_cast<int>(state.rows());
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.pose.covariance.at(n * i + j) = covariance(i, j);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.twist.covariance.at(n * i + j) = covariance(i + 3, j + 3);

        m_pub_target_odom.publish(msg);
    }

    void Masters::visualise_arrow(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &end,
                                  const ros::Time &detection_time) {

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
        gpt1.x = start[0];
        gpt1.y = start[1];
        gpt1.z = start[2];
        gpt2.x = end[0];
        gpt2.y = end[1];
        gpt2.z = end[2];
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
    }

    void Masters::visualise_sphere(const Eigen::Vector3d &pos, const ros::Time &t) {

        visualization_msgs::Marker marker_predict;
        marker_predict.header.frame_id = m_name_world_origin;
        marker_predict.header.stamp = t;
        marker_predict.ns = "my_namespace";
        marker_predict.id = 0;
        marker_predict.type = visualization_msgs::Marker::SPHERE;
        marker_predict.action = visualization_msgs::Marker::MODIFY;

        marker_predict.pose.position.x = pos[0];
        marker_predict.pose.position.y = pos[1];
        marker_predict.pose.position.z = pos[2];
        marker_predict.scale.x = 0.2;
        marker_predict.scale.y = 0.2;
        marker_predict.scale.z = 0.2;
        marker_predict.color.a = 1.0; // Don't forget to set the alpha!
        marker_predict.color.r = 0.0;
        marker_predict.color.g = 1.0;
        marker_predict.color.b = 0.0;
        m_pub_viz.publish(marker_predict);

    }
}  // namespace masters

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(masters::Masters, nodelet::Nodelet)
