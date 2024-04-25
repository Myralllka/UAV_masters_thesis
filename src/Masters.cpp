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
        pl.loadParam("history_size", m_history_buf_size);
        pl.loadParam("real_world", m_is_real_world);
        pl.loadParam("main_camera", m_name_main_camera);
        pl.loadParam("approx_drone_size_for_bbox_height", m_drone_h);
        pl.loadParam("approx_drone_size_for_bbox_width", m_drone_w);
        pl.loadParam("mean", m_mean);
        pl.loadParam("imhint", m_image_transport_hint);
        pl.loadParam("dt_kalman", m_dt);
        pl.loadParam("deviation", m_stddev);
        pl.loadParam("eagle_gt_odometry", m_name_eagle_odom_msg);
        pl.loadParam("target_gt_odometry", m_name_target_odom_msg);
        pl.loadParam("time_th", m_time_th);
        pl.loadParam("correction_th", m_correction_th);
        if (m_is_real_world) {
            pl.loadParam("lidar_tracker", m_name_lidar_tracker);
        }

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", m_nodename.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", m_nodename.c_str());
        }

        // Dynamic reconfigure
        server.setCallback(boost::bind(&Masters::m_cbk_dynrec, this, _1, _2));

        // | ---------------- some data post-processing --------------- |
        m_history_linear = boost::circular_buffer<std::pair<vec3, vec3>>(m_history_buf_size);
        m_history_velocity = boost::circular_buffer<std::tuple<vec3, vec3, ros::Time>>(m_history_buf_size);

        // | ----------------- publishers initialize ------------------ |
        m_pub_image_changed = nh.advertise<sensor_msgs::Image>("changed", 1);
        m_pub_detection = nh.advertise<geometry_msgs::PointStamped>("detection", 1);
        m_pub_detection_svd = nh.advertise<geometry_msgs::PoseArray>("detection_svd", 1);
        m_pub_history1 = nh.advertise<geometry_msgs::PoseArray>("history_1", 1);
        m_pub_history2 = nh.advertise<geometry_msgs::PoseArray>("history_2", 1);
        m_pub_viz = nh.advertise<visualization_msgs::Marker>("detection_marker", 1);
        m_pub_target_odom = nh.advertise<nav_msgs::Odometry>(m_nodename + "/detected_target", 1);

        // | ---------------- subscribers initialize ------------------ |
        mrs_lib::SubscribeHandlerOptions shopt{nh};
        shopt.node_name = m_nodename;
        shopt.threadsafe = true;
        shopt.no_message_timeout = ros::Duration(1.0);
        sensor_msgs::CameraInfo cam_info;
        mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> handler_caminfo;
        mrs_lib::construct_object(handler_caminfo,
                                  shopt,
                                  m_name_main_camera + "/camera_info");
        mrs_lib::construct_object(m_subh_eagle_odom,
                                  shopt,
                                  m_name_eagle_odom_msg);
        mrs_lib::construct_object(m_subh_target_odom,
                                  shopt,
                                  m_name_target_odom_msg);
        mrs_lib::construct_object(m_subh_pcl_track,
                                  shopt,
                                  m_name_lidar_tracker);
        while (ros::ok()) {
            if (handler_caminfo.hasMsg()) {
                cam_info = *handler_caminfo.getMsg().get();
                break;
            }
        }
        handler_caminfo.stop();

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer(nh, m_nodename, ros::Duration(1));
        m_transformer.setLookupTimeout(ros::Duration(0.1));


        m_t0 = ros::Time::now();
        m_camera_main.fromCameraInfo(cam_info);
        image_transport::ImageTransport it(nh);
        const image_transport::TransportHints hint(m_image_transport_hint);
        ROS_INFO("[%s]: approach used: %s", m_nodename.c_str(), m_approach.c_str());
        if (m_approach == "dkf") {
            m_sub_detection = nh.subscribe("detection", 1, &Masters::m_cbk_detection, this);
            m_sub_main_camera = it.subscribe(m_name_main_camera + "/image_raw", 1,
                                             &Masters::m_cbk_main_camera, this, hint);
        } else if (m_approach == "plkf") {
            m_sub_detection = nh.subscribe("detection", 1, &Masters::m_cbk_detection, this);
            m_sub_main_camera = it.subscribe(m_name_main_camera + "/image_raw", 1,
                                             &Masters::m_cbk_main_camera, this, hint);

        } else if (m_approach == "svd_static") {
            m_sub_main_camera_detection = nh.subscribe("detection_svd", 1, &Masters::m_cbk_posearray_svd_pos_vel, this);
            m_sub_main_camera = it.subscribe(m_name_main_camera + "/image_raw", 1,
                                             &Masters::m_cbk_camera_image_to_detection_svd, this, hint);

        } else if (m_approach == "svd_dynamic") {
            m_sub_main_camera_detection = nh.subscribe("detection_svd", 1, &Masters::m_cbk_posearray_svd_pos_vel, this);
            m_sub_main_camera = it.subscribe(m_name_main_camera + "/image_raw", 1,
                                             &Masters::m_cbk_camera_image_to_detection_svd, this, hint);

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
        auto pt_rect = m_camera_main.rectifyPoint({msg.point.x, msg.point.y});
        cv::Point3d td_ray = m_camera_main.projectPixelTo3dRay(pt_rect);
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


    [[maybe_unused]] void Masters::m_cbk_main_camera(const sensor_msgs::ImageConstPtr &msg) {
        if (not m_is_initialized) return;
        const auto detection_opt = m_detect_uav(msg);
//        const auto detection_opt_lr = m_detect_uav_with_bbox(msg);

        m_name_main_camera_tf = msg->header.frame_id;

        if (detection_opt.has_value()) {
            cv::Point2d detection = detection_opt.value();
//            double lm, rm;
//            std::tie(detection, lm, rm) = detection_opt_lr.value();
//            std::cout << lm << std::endl;
//            std::cout << rm << std::endl;

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
        const auto detection_opt = m_detect_uav(msg);
        cv::Point2d detection;
        if (detection_opt.has_value()) {
            detection = detection_opt.value();
        } else {
            ROS_ERROR_THROTTLE(1.0, "[%s]: No detection present;", m_nodename.c_str());
            return;
        }
        cv::Point3d ray_to_detection;
        ray_to_detection = {detection.x, detection.y, 1};

        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // visualize detection
        cv::circle(image, detection, 20, cv::Scalar(255, 0, 0), 2);
        m_pub_image_changed.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());


        geometry_msgs::Pose p1, p2;
//        p1 is the interceptor in the world coordinate frame
        if (m_subh_eagle_odom.hasMsg()) {
            const auto msg_eagle_odom = m_subh_eagle_odom.getMsg();
            const auto new_pose_st_opt = m_transformer.transformAsPoint(msg_eagle_odom->header.frame_id,
                                                                        {msg_eagle_odom->pose.pose.position.x,
                                                                         msg_eagle_odom->pose.pose.position.y,
                                                                         msg_eagle_odom->pose.pose.position.z},
                                                                        m_name_world_origin,
                                                                        msg_eagle_odom->header.stamp);
            if (new_pose_st_opt.has_value()) {
                p1.position.x = new_pose_st_opt->x();
                p1.position.y = new_pose_st_opt->y();
                p1.position.z = new_pose_st_opt->z();
            } else {
                ROS_ERROR("[%s]: pose or twist transformation has no value", m_nodename.c_str());
                return;
            }
        } else {
            ROS_ERROR("[%s]: No odometry msg from %s", m_nodename.c_str(), m_name_eagle_odom_msg.c_str());
            return;
        }
        // transform to the static world frame
        cv::Point3d ray_to_detection_original = m_camera_main.projectPixelTo3dRay({ray_to_detection.x,
                                                                                   ray_to_detection.y});

        auto ray_opt = m_transformer.transformAsPoint(m_name_main_camera_tf,
                                                      vec3{ray_to_detection_original.x,
                                                           ray_to_detection_original.y,
                                                           ray_to_detection_original.z},
                                                      m_name_world_origin,
                                                      msg->header.stamp);
        if (ray_opt.has_value()) {
            ray_to_detection = {ray_opt.value().x(),
                                ray_opt.value().y(),
                                ray_opt.value().z()};
        } else {
            return;
        }

//        p2 is the 3d position of the end of a ray from the interceptor to the target
        p2.position.x = ray_to_detection.x;
        p2.position.y = ray_to_detection.y;
        p2.position.z = ray_to_detection.z;

        geometry_msgs::PoseArray res;
        res.header.stamp = msg->header.stamp;
        res.header.frame_id = m_name_world_origin;
        res.poses.push_back(p1);
        res.poses.push_back(p2);
        m_pub_detection_svd.publish(res);
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
            const auto ps = msg_eagle_odom->pose.pose.position;
            const auto tw = msg_eagle_odom->twist.twist.linear;
            const auto new_pose_st_opt = m_transformer.transformAsPoint(msg_eagle_odom->header.frame_id,
                                                                        {ps.x, ps.y, ps.z},
                                                                        m_name_world_origin,
                                                                        msg_eagle_odom->header.stamp);
            const auto new_twist_st_opt = m_transformer.transformAsVector(msg_eagle_odom->child_frame_id,
                                                                          {tw.x, tw.y, tw.z},
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
//        if (m_is_kalman_initialized and dt <= m_time_th) {
        if (m_is_kalman_initialized) {
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman is initialised", m_nodename.c_str());

            // Predict always
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman predict", m_nodename.c_str());
            if (m_approach == "dkf") {
                dkf_t::u_t u;
                u.setZero();
                Eigen::Matrix<double, 6, 6> Q = B * m_Q * B.transpose();
                try {
                    m_state_vec = m_dkf.predict(m_state_vec, u, Q, dt);
                }
                catch (const std::exception &e) {
                    ROS_ERROR("DKF predict failed: %s", e.what());
                }
            } else if (m_approach == "plkf") {
                std::tie(m_state_vec.x, m_state_vec.P) = plkf_predict(m_state_vec.x,
                                                                      m_state_vec.P,
                                                                      m_state_interceptor,
                                                                      state_interceptor_new,
                                                                      dt);
            }

            // Correct based on the distance moved from the previous position
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman correct", m_nodename.c_str());
            if (m_approach == "dkf") {
                if ((state_interceptor_new.segment<3>(0) - m_position_last_correction).norm() >= m_correction_th) {
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
            } else if (m_approach == "plkf") {
                std::tie(m_state_vec.x, m_state_vec.P) = plkf_correct(m_state_vec.x, m_state_vec.P, detection_vec);
            }
        } else {
            // initialise
            m_dkf = dkf_t(A, dkf_t::B_t::Zero());
            Eigen::Matrix<double, 6, 1> x_k;
            x_k.setZero();
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman initialise", m_nodename.c_str());
            if (not m_is_real_world and m_subh_target_odom.hasMsg()) {
                const auto msg_target_odom = m_subh_target_odom.getMsg();
                const auto ps = msg_target_odom->pose.pose.position;
                const auto tw = msg_target_odom->twist.twist.linear;
//                const auto new_pose_st_opt = m_transformer.transformAsPoint(msg_target_odom->header.frame_id,
//                                                                            {ps.x, ps.y, ps.z},
//                                                                            m_name_world_origin,
//                                                                            msg_target_odom->header.stamp);
//                const auto new_twist_st_opt = m_transformer.transformAsVector(msg_target_odom->child_frame_id,
//                                                                              {tw.x, tw.y, tw.z},
//                                                                              m_name_world_origin,
//                                                                              msg_target_odom->header.stamp);
//                if (new_pose_st_opt.has_value() and new_twist_st_opt.has_value()) {
                if (true) {
//                    const auto pos = new_pose_st_opt.value();
//                    const auto vel = new_twist_st_opt.value();
                    const Eigen::Vector3d pos{ps.x, ps.y, ps.z};
                    const Eigen::Vector3d vel{tw.x, tw.y, tw.z};
                    if (m_approach == "dkf") {
// state for the DKF is the absolute position of the target in a common frame
                        x_k[0] = pos.x();
                        x_k[1] = pos.y();
                        x_k[2] = pos.z();
                        x_k[3] = vel.x();
                        x_k[4] = vel.y();
                        x_k[5] = vel.z();
                    } else if (m_approach == "plkf") {
//  state for the PLKF is the relative pos and vel of a target with respect to the interceptor
                        x_k[0] = pos.x() - state_interceptor_new[0];
                        x_k[1] = pos.y() - state_interceptor_new[1];
                        x_k[2] = pos.z() - state_interceptor_new[2];
                        x_k[3] = vel.x() - state_interceptor_new[3];
                        x_k[4] = vel.y() - state_interceptor_new[4];
                        x_k[5] = vel.z() - state_interceptor_new[5];
                    }
                    m_state_vec.x = x_k;
                    m_state_vec.P = m_P0;
                } else {
                    ROS_ERROR("[%s]: pose or twist transformation for dkf init has no value", m_nodename.c_str());
                    return;
                }

            } else if (m_is_real_world and m_subh_pcl_track.hasMsg()) {
                ROS_INFO("[%s]: is_real_world set to use the lidar, or there are no msgs from the %s",
                         m_nodename.c_str(), m_name_target_odom_msg.c_str());
                const auto tracks = m_subh_pcl_track.getMsg();
                bool is_selected = false;
                for (const auto &track: tracks.get()->tracks) {
                    if (track.selected) {
                        is_selected = true;
                        const geometry_msgs::Point pos = track.position;
                        const geometry_msgs::Vector3 vel = track.velocity;
                        if (m_approach == "dkf") {
// state for the DKF is the absolute position of the target in a common frame
                            x_k[0] = pos.x;
                            x_k[1] = pos.y;
                            x_k[2] = pos.z;
                            x_k[3] = vel.x;
                            x_k[4] = vel.y;
                            x_k[5] = vel.z;
                        } else if (m_approach == "plkf") {
//  state for the PLKF is the relative pos and vel of a target with respect to the interceptor
                            x_k[0] = pos.x - state_interceptor_new[0];
                            x_k[1] = pos.y - state_interceptor_new[1];
                            x_k[2] = pos.z - state_interceptor_new[2];
                            x_k[3] = vel.x - state_interceptor_new[3];
                            x_k[4] = vel.y - state_interceptor_new[4];
                            x_k[5] = vel.z - state_interceptor_new[5];
                        }
                        m_state_vec.x = x_k;
                        m_state_vec.P = m_P0;
                    }
                }
                if (not is_selected) {
                    ROS_ERROR("[%s]: No tracks *selected* found ", m_nodename.c_str());
                }
            } else {
                ROS_ERROR("[%s]: No lidar tracks from %s", m_nodename.c_str(),
                          m_subh_pcl_track.subscribedTopicName().c_str());
                return;
            }
            ROS_INFO("[%s]: kalman is initialised", m_nodename.c_str());
            m_is_kalman_initialized = true;
        }

        const Eigen::Matrix<double, 6, 1> m_x_k = m_state_vec.x;
        const Eigen::Matrix<double, 6, 6> m_P_k = m_state_vec.P;
        postproc(m_x_k, m_P_k, detection_time);
        visualise_arrow(state_interceptor_new.head(3),
                        state_interceptor_new.head(3) + detection_vec,
                        detection_time);
        if (m_approach == "dkf") {
            visualise_sphere(state_interceptor_new.head(3) + detection_vec, detection_time);
        } else if (m_approach == "plkf") {
            visualise_sphere(state_interceptor_new.head(3) + m_x_k.head(3), detection_time);
        }
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

        Eigen::Matrix3d ps = Hk * Pk_ * Hk.transpose() + Vk * m_R * Vk.transpose();
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
        ROS_INFO_THROTTLE(1.0, "[%s]: history published", m_nodename.c_str());

        if (m_history_velocity.size() <= 1) {
            ROS_INFO_THROTTLE(1.0, "[%s]: history size is smaller/equal 1", m_nodename.c_str());
            m_history_linear.push_back(std::pair{pos_origin, pos_s});
            m_history_velocity.push_back(std::tuple{pos_origin, pos_s, msg.header.stamp});
            return;
        }
        if ((pos_origin - std::get<0>(m_history_velocity.back())).norm() > m_correction_th) {
            ROS_INFO_THROTTLE(1.0, "[%s]:  the uav is quite far away, updating", m_nodename.c_str());
            m_history_linear.push_back(std::pair{pos_origin, pos_s});
            m_history_velocity.push_back(std::tuple{pos_origin, pos_s, msg.header.stamp});
        }
        if (m_history_velocity.size() >= 4) {
            ROS_INFO_THROTTLE(1.0, "[%s]: history size is more than 4", m_nodename.c_str());
            vec3 position, velocity;
            if (m_approach == "svd_dynamic") {
                std::tie(position, velocity) = m_find_intersection_svd_velocity_obj(m_history_velocity);
                velocity = {0, 0, 0};
            } else if (m_approach == "svd_static") {
                position = m_find_intersection_svd_static_obj(m_history_linear);
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
            postproc(state,
                     Eigen::Matrix<double, 6, 6>::Identity(),
                     msg.header.stamp);
            ROS_INFO_THROTTLE(1.0, "[%s]: visualising sphere...", m_nodename.c_str());
            visualise_sphere(state.head(3), msg.header.stamp);
        }
    }

    void Masters::postproc(const Eigen::Matrix<double, 6, 1> state,
                           const Eigen::Matrix<double, 6, 6> covariance,
                           const ros::Time &t) {
//            TODO: save/publish an additional data here
        ROS_INFO_THROTTLE(1.0, "[%s]: visualising odometry...", m_nodename.c_str());
        visualise_odometry(state, covariance, t);
    }

// | --------------------- timer callbacks -------------------- |


// | -------------------- other functions ------------------- |

    [[maybe_unused]] vec3
    Masters::m_find_intersection_svd_static_obj(const t_hist_vv &data) {

        const int n_pts = data.size();
        const int n_r = 3 * n_pts;
        const int n_c = 3 + n_pts;

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_r, n_c);
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
        const auto msg_target_odom = m_subh_target_odom.getMsg();
        const auto msg_eagle_odom = m_subh_eagle_odom.getMsg();
        const auto ps_tgt = msg_target_odom->pose.pose.position;
        const auto ps_eagle = msg_eagle_odom->pose.pose.position;

        const auto new_pose_st_opt = m_transformer.transformAsPoint(msg_target_odom->header.frame_id,
                                                                    {ps_tgt.x, ps_tgt.y, ps_tgt.z},
                                                                    m_name_world_origin,
                                                                    msg->header.stamp);
        const auto new_pose_eagle_opt = m_transformer.transformAsPoint(msg_eagle_odom->header.frame_id,
                                                                       {ps_eagle.x, ps_eagle.y, ps_eagle.z},
                                                                       m_name_world_origin,
                                                                       msg->header.stamp);

        Eigen::Vector3d dir_vec, dir_vec_tmp;
        if (new_pose_eagle_opt.has_value()) {
            Eigen::Vector3d pos_tgt = {ps_tgt.x, ps_tgt.y, ps_tgt.z};
            const auto pos_eagle = new_pose_eagle_opt.value();
            if (new_pose_st_opt.has_value()) {
                const auto pos_tgt_auto = new_pose_st_opt.value();
                pos_tgt = Eigen::Vector3d{pos_tgt_auto.x(), pos_tgt_auto.y(), pos_tgt_auto.z()};
            }
            std::cout << pos_tgt << std::endl;
            std::cout << pos_eagle << std::endl;
            visualise_arrow(pos_eagle, pos_tgt, msg->header.stamp);
//  state for the PLKF is the relative pos and vel of a target with respect to the interceptor
            dir_vec_tmp = pos_tgt - pos_eagle;
            const auto new_dirvec = m_transformer.transformAsVector(m_name_world_origin,
                                                                           dir_vec_tmp,
                                                                           msg->header.frame_id,
                                                                           msg->header.stamp);
            if (new_dirvec.has_value()) {
                dir_vec = new_dirvec.value();
            } else {
                std::cout << "FUCK IT ALL" << std::endl;
            }
            std::cout << dir_vec << std::endl;
        } else {
            ROS_ERROR("[%s]: pose transformation for detection init has no value", m_nodename.c_str());
            return std::nullopt;
        }
//        if (dir_vec.z() < 0) {
//            ROS_ERROR("[%s]: pose is behind the image plane", m_nodename.c_str());
//            return std::nullopt;
//        }
        const auto pt_ideal = m_camera_main.project3dToPixel(cv::Point3d(dir_vec.x(), dir_vec.y(), dir_vec.z()));
        std::random_device rseed;
        std::mt19937 rng(rseed());
        std::normal_distribution<double> dist(m_mean, m_stddev);
        double e_x = dist(rng), e_y = dist(rng);

        const cv::Point2d pt_noisy{pt_ideal.x + e_x, pt_ideal.y + e_y};
//        if (pt_noisy.x < 0 or pt_noisy.x > m_camera_main.cameraInfo().width or
//            pt_noisy.y < 0 or pt_noisy.y > m_camera_main.cameraInfo().height) {
//            return std::nullopt;
//        }
        std::cout << pt_noisy << std::endl;
        // visualize detection
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::circle(image, pt_noisy, 20, cv::Scalar(255, 0, 0), 2);
        m_pub_image_changed.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
        return pt_noisy;
    }

//    std::optional<cv::Point2d> Masters::m_detect_uav_old(const sensor_msgs::Image::ConstPtr &msg) {
//        auto T_eagle2drone_opt = m_transformer.getTransform(m_name_target + "/fcu",
//                                                            msg->header.frame_id,
//                                                            msg->header.stamp);
//        if (!T_eagle2drone_opt.has_value()) {
//            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR wrong detection");
//            return std::nullopt;
//        }
//        const geometry_msgs::TransformStamped T_eagle2drone = T_eagle2drone_opt.value();;
//        const Eigen::Vector3d dir_vec(tf2::transformToEigen(T_eagle2drone.transform).translation().data());
//        if (dir_vec.z() < 0) {
//            return std::nullopt;
//        }
////        if (dir_vec.z() < 0) {
////            ROS_ERROR("[%s]: pose is behind the image plane", m_nodename.c_str());
////            return std::nullopt;
////        }
//        const auto pt_ideal = m_camera_main.project3dToPixel(cv::Point3d(dir_vec.x(), dir_vec.y(), dir_vec.z()));
//        std::random_device rseed;
//        std::mt19937 rng(rseed());
//        std::normal_distribution<double> dist(m_mean, m_stddev);
//        double e_x = dist(rng), e_y = dist(rng);
//
//        const cv::Point2d pt_noisy{pt_ideal.x + e_x, pt_ideal.y + e_y};
////        if (pt_noisy.x < 0 or pt_noisy.x > m_camera_main.cameraInfo().width or
////            pt_noisy.y < 0 or pt_noisy.y > m_camera_main.cameraInfo().height) {
////            return std::nullopt;
////        }
//        return pt_noisy;
//    }

//    std::optional<std::tuple<cv::Point2d, double, double>>
//    Masters::m_detect_uav_with_bbox(const sensor_msgs::Image::ConstPtr &msg) {
//        auto T_eagle2drone_opt = m_transformer.getTransform(m_name_target + "/fcu",
//                                                            msg->header.frame_id,
//                                                            msg->header.stamp);
//        if (!T_eagle2drone_opt.has_value()) {
//            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR wrong detection");
//            return std::nullopt;
//        }
//        const geometry_msgs::TransformStamped T_eagle2drone = T_eagle2drone_opt.value();
//        const Eigen::Vector3d dir_vec(tf2::transformToEigen(T_eagle2drone.transform).translation().data());
//        std::vector<cv::Point3d> bbox_pts;
////                Eigen::Vector3d{dir_vec.x() + m_drone_w, dir_vec.y() + m_drone_w, dir_vec.z() + m_drone_h},
////                Eigen::Vector3d{dir_vec.x() - m_drone_w, dir_vec.y() + m_drone_w, dir_vec.z() + m_drone_h},
////                Eigen::Vector3d{dir_vec.x() + m_drone_w, dir_vec.y() - m_drone_w, dir_vec.z() + m_drone_h},
////                Eigen::Vector3d{dir_vec.x() - m_drone_w, dir_vec.y() - m_drone_w, dir_vec.z() + m_drone_h},
////                Eigen::Vector3d{dir_vec.x() + m_drone_w, dir_vec.y() + m_drone_w, dir_vec.z() - m_drone_h},
////                Eigen::Vector3d{dir_vec.x() - m_drone_w, dir_vec.y() + m_drone_w, dir_vec.z() - m_drone_h},
////                Eigen::Vector3d{dir_vec.x() + m_drone_w, dir_vec.y() - m_drone_w, dir_vec.z() - m_drone_h},
////                Eigen::Vector3d{dir_vec.x() - m_drone_w, dir_vec.y() - m_drone_w, dir_vec.z() - m_drone_h},
////      construct an array representing the bbox
//        for (int i = 0; i < 8; ++i) {
//            bbox_pts.push_back(cv::Point3d{dir_vec.x() + std::pow(-1, i) * m_drone_w,
//                                           dir_vec.y() + std::pow(-1, i / 2) * m_drone_w,
//                                           dir_vec.z() + std::pow(-1, i / 4) * m_drone_h});
//        }
//        if (dir_vec.z() < 0) {
//            return std::nullopt;
//        }
////        project the bounding box on the image
//        const auto pt_ideal = m_camera_main.project3dToPixel(cv::Point3d(dir_vec.x(), dir_vec.y(), dir_vec.z()));
//        std::vector<cv::Point2d> bbox_projected;
//        double leftmost{std::numeric_limits<double>::max()};
//        double rightmost{std::numeric_limits<double>::min()};
//        std::cout << "----------------------------" << std::endl;
//        for (const auto &pt: bbox_pts) {
//            const auto pt_proj = m_camera_main.project3dToPixel(pt);
//            std::cout << pt_proj << std::endl;
//            leftmost = pt_proj.x ? pt_proj.x < leftmost : leftmost;
//            rightmost = pt_proj.x ? pt_proj.x > rightmost : rightmost;
//            bbox_projected.push_back(pt_proj);
//        }
//
//        std::random_device rseed;
//        std::mt19937 rng(rseed());
//        std::normal_distribution<double> dist(m_mean, m_stddev);
//        double e_x = dist(rng), e_y = dist(rng);
//
//        const cv::Point2d pt_noisy{pt_ideal.x + e_x, pt_ideal.y + e_y};
//        if (pt_noisy.x < 0 or pt_noisy.x > m_camera_main.cameraInfo().width or
//            pt_noisy.y < 0 or pt_noisy.y > m_camera_main.cameraInfo().height) {
//            return std::nullopt;
//        }
//        return std::tuple{pt_noisy, leftmost, rightmost};
//    }
//
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
