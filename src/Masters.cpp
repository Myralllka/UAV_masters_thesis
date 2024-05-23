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
        double tgt_s_init;
        pl.loadParam("eagle_name", m_name_eagle);
        pl.loadParam("approach", m_approach);
        pl.loadParam("target_name", m_name_target);
        pl.loadParam("world_origin", m_name_world_origin);
        pl.loadParam("history_size", m_history_buf_size);
        pl.loadParam("real_world", m_is_real_world);
        pl.loadParam("main_camera", m_name_main_camera);
        pl.loadParam("approx_drone_size_for_bbox_height", m_tgt_h);
        pl.loadParam("approx_drone_size_for_bbox_width", m_tgt_w);
        pl.loadParam("mean", m_mean);
        pl.loadParam("imhint", m_image_transport_hint);
        pl.loadParam("dt_kalman", m_dt);
        pl.loadParam("deviation", m_stddev);
        pl.loadParam("eagle_gt_odometry", m_name_eagle_odom_msg);
        pl.loadParam("target_gt_odometry", m_name_target_odom_msg);
        pl.loadParam("time_th", m_time_th);
        pl.loadParam("correction_th", m_correction_th);
        pl.loadParam("lidar_tracker", m_name_lidar_tracker);
        pl.loadParam("angle_variance", m_angle_variance);
        pl.loadParam("object_size_variance", m_obj_size_variance);
        pl.loadParam("scenario", m_scenario);
        pl.loadParam("eagle_deviation", m_eagle_stddev);
        pl.loadParam("tgt_init_deviation", m_tgt_stddev);
        pl.loadParam("bearing_var", m_var_bearing_vector);
        pl.loadParam("target_init_size", tgt_s_init);
        pl.loadParam("use_Qpv", m_use_qpv);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", m_nodename.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", m_nodename.c_str());
        }

        // Dynamic reconfigure
        server.setCallback(boost::bind(&Masters::m_cbk_dynrec, this, _1, _2));

        // | ---------------- some data post-processing --------------- |
        ROS_INFO("[%s]: data post proc... :", m_nodename.c_str());
        m_history_linear = boost::circular_buffer<std::pair<vec3, vec3>>(m_history_buf_size);
        m_history_velocity = boost::circular_buffer<std::tuple<vec3, vec3, ros::Time>>(m_history_buf_size);

        // | ----------------- publishers initialize ------------------ |
        ROS_INFO("[%s]: pub initialisation... :", m_nodename.c_str());
        m_pub_image_changed = nh.advertise<sensor_msgs::Image>("changed", 1);
        m_pub_detection = nh.advertise<geometry_msgs::PointStamped>("detection", 1);
        m_pub_detection_svd = nh.advertise<geometry_msgs::PoseArray>("detection_svd", 1);
        m_pub_history1 = nh.advertise<geometry_msgs::PoseArray>("history_1", 1);
        m_pub_history2 = nh.advertise<geometry_msgs::PoseArray>("history_2", 1);
        m_pub_viz = nh.advertise<visualization_msgs::Marker>("detection_marker", 1);
        m_pub_target_odom = nh.advertise<nav_msgs::Odometry>(m_nodename + "/detected_target", 1);

        // | ---------------- subscribers initialize ------------------ |
        ROS_INFO("[%s]: subh initialisation... :", m_nodename.c_str());
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
//        mrs_lib::construct_object(m_subh_pcl_track,
//                                  shopt,
//                                  m_name_lidar_tracker);
        ROS_INFO("[%s]: subh initialised.", m_nodename.c_str());
        while (ros::ok()) {
            if (handler_caminfo.newMsg()) {
                cam_info = *handler_caminfo.getMsg().get();
                break;
            }
        }
        handler_caminfo.stop();
        ROS_INFO("[%s]: camera initialised.", m_nodename.c_str());

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer(nh, m_nodename, ros::Duration(3));
        m_transformer.setLookupTimeout(ros::Duration(1));
        ROS_INFO("[%s]: transformer initialised.", m_nodename.c_str());


        m_camera_main.fromCameraInfo(cam_info);
        image_transport::ImageTransport it(nh);
        const image_transport::TransportHints hint(m_image_transport_hint);
        ROS_INFO("[%s]: approach used: %s", m_nodename.c_str(), m_approach.c_str());
        if (m_approach == "dkf" or m_approach == "dkft" or m_approach == "plkf" or m_approach == "plkft") {
            m_sub_main_camera = it.subscribe(m_name_main_camera + "/image_raw", 1,
                                             &Masters::m_cbk_main_camera, this, hint);
            m_sub_detection = nh.subscribe("detection", 1, &Masters::m_cbk_detection, this);
        } else if (m_approach == "svd-static" or m_approach == "svd-dynamic") {
            m_sub_main_camera = it.subscribe(m_name_main_camera + "/image_raw", 1,
                                             &Masters::m_cbk_camera_image_to_detection_svd, this, hint);
            m_sub_main_camera_detection = nh.subscribe("detection_svd", 1, &Masters::m_cbk_posearray_svd_pos_vel, this);
        } else {
            ROS_ERROR("[%s]: unknown m_aproach %s", m_nodename.c_str(), m_approach.c_str());
            ros::shutdown();
        }

        m_init_tgt_width = tgt_s_init;
//        m_init_tgt_width = 1;

        std::stringstream fnamestream;
        fnamestream.precision(1);
        fnamestream << "/home/mrs/workspace/src/masters/logs/";
        fnamestream << m_approach << "_" << std::fixed
                    << m_correction_th << "_" << m_stddev << "_" << m_history_buf_size << "_" << m_tgt_w * 2
                    << "_" << m_angle_variance << "_" << m_eagle_stddev << "_" << m_tgt_stddev << "_" << m_scenario;
        m_fnamestream = fnamestream.str();

        // | -------------------- initialize timers ------------------- |
        // Some additional inits
//        https://www.mdpi.com/2072-4292/13/15/2915

        m_is_initialized = true;
        ROS_INFO_ONCE("[%s]: initialized", m_nodename.c_str());
    }
//}

// | ----------------- dynamic reconf callback ---------------- |
    void Masters::m_cbk_dynrec(masters::DynrecConfig &config, [[maybe_unused]] uint32_t level) {
        Eigen::Matrix<double, 6, 6> G;
//        https://www.mdpi.com/2072-4292/13/15/2915

        ROS_INFO("New dynamic reconfigure values received.");
        m_Q = Eigen::Matrix<double, 3, 3>::Identity() * config.s_Q_acc;
        m_Qpv = Eigen::Matrix<double, 6, 6>::Identity() * config.s_Q_pos;
        m_Qpv.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * config.s_Q_vel;
        m_R = Eigen::Matrix3d::Identity() * config.s_R;
        m_state_vec.P = Eigen::Matrix<double, 6, 6>::Zero();
        m_state_vec.P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config.s_P0_position;
        m_state_vec.P.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * config.s_P0_velocity;
        m_P0 = Eigen::Matrix<double, 6, 6>::Identity();
        m_P0.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config.s_P0_position;
        m_P0.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * config.s_P0_velocity;
        m_var_dkf_radius = config.dkf_var;
        m_var_bearing_vector = config.bearing_var;
    }

// | ---------------------- msg callbacks --------------------- |

    [[maybe_unused]] void Masters::m_cbk_detection(const geometry_msgs::PointStamped &msg) {
        std::unique_lock lk(m_mut_tgt_gt);
        m_cv.wait(lk, [this] { return not m_gt_used; });
        dkf_ang_t::statecov_t tgt_gt = m_tgt_gt;
        dkf_ang_t::statecov_t eagle_gt = m_eagle_gt;
        m_gt_used = true;
        ros::Time ts_gt = m_gt_ts;
        lk.unlock();

        if (not m_is_initialized) return;
        //instead of z in point there is a theta angle

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
                update_kalman(detection_vec, msg.point.z, msg.header.stamp, tgt_gt, eagle_gt, ts_gt);
            }
        } else {
            ROS_ERROR_STREAM("[" << m_nodename << "]: ERROR no transformation from " << msg.header.frame_id << " to "
                                 << m_name_world_origin);
        }
    }


    [[maybe_unused]] void Masters::m_cbk_main_camera(const sensor_msgs::ImageConstPtr &msg) {
        if (not m_is_initialized) return;
        ROS_INFO_THROTTLE(5.0, "[%s]: main camera cbk started", m_nodename.c_str());
//        const auto detection_opt = m_detect_uav(msg);
        m_name_main_camera_tf = msg->header.frame_id;

        const auto detection_opt = m_detect_uav_with_angle(msg);

        double theta;
        cv::Point2d detection;
        dkf_ang_t::statecov_t tgt_gt;
        dkf_ang_t::statecov_t eagle_gt;

        if (detection_opt.has_value()) {
            std::tie(detection, theta, tgt_gt, eagle_gt) = detection_opt.value();

            geometry_msgs::PointStamped det_ros_msg;
            det_ros_msg.header = msg->header;
            det_ros_msg.point.x = detection.x;
            det_ros_msg.point.y = detection.y;
            det_ros_msg.point.z = theta;
            {
                std::lock_guard lg(m_mut_tgt_gt);
                m_tgt_gt = tgt_gt;
                m_eagle_gt = eagle_gt;
                m_gt_used = false;
                m_gt_ts = msg->header.stamp;
            }
            m_pub_detection.publish(det_ros_msg);
            m_cv.notify_one();
        } else {
            ROS_ERROR_THROTTLE(5.0, "[%s]: No detection present;", m_nodename.c_str());
            return;
        }
    }


    [[maybe_unused]] void Masters::m_cbk_camera_image_to_detection_svd(const sensor_msgs::ImageConstPtr &msg) {
        if (not m_is_initialized) return;
        ROS_INFO_THROTTLE(5.0, "[%s]: SVD detection start %s", m_nodename.c_str(), m_approach.c_str());
        const auto detection_opt = m_detect_uav_with_angle(msg);
        m_name_main_camera_tf = msg->header.frame_id;

        double theta;
        cv::Point2d detection;
        dkf_ang_t::statecov_t tgt_gt;
        dkf_ang_t::statecov_t eagle_gt;
        if (detection_opt.has_value()) {
            std::tie(detection, theta, tgt_gt, eagle_gt) = detection_opt.value();
        } else {
            ROS_ERROR_THROTTLE(5.0, "[%s]: No detection present;", m_nodename.c_str());
            return;
        }
        cv::Point3d ray_to_detection;
        ray_to_detection = {detection.x, detection.y, 1};

        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // visualize detection
        cv::circle(image, detection, 20, cv::Scalar(255, 0, 0), 2);
        m_pub_image_changed.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());


        geometry_msgs::Pose p1, p2;
        p1.position.x = eagle_gt.x.x();
        p1.position.y = eagle_gt.x.y();
        p1.position.z = eagle_gt.x.z();
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
        {
            std::lock_guard lg(m_mut_tgt_gt);
            m_tgt_gt = tgt_gt;
            m_eagle_gt = eagle_gt;
            m_gt_used = false;
            m_gt_ts = msg->header.stamp;
        }
        m_pub_detection_svd.publish(res);
        m_cv.notify_one();
    }
// | --------------------- timer callbacks -------------------- |

    void Masters::update_kalman(Eigen::Vector3d detection_vec, double subtended_angle, ros::Time detection_time,
                                const dkf_ang_t::statecov_t &tgt_gt, const dkf_ang_t::statecov_t &eagle_gt,
                                ros::Time &ts_gt) {
        if (not m_is_initialized) return;
        ROS_INFO_THROTTLE(5.0, "[%s]: kalman cbk start", m_nodename.c_str());
        if (m_last_kalman_time == ros::Time(0)) {
            m_last_kalman_time = detection_time;
            return;
        }
        const double dt = (detection_time - m_last_kalman_time).toSec();
        m_last_kalman_time = detection_time;

        // Compute the state of the interceptor
        Eigen::Matrix<double, 6, 1> state_interceptor_new;
        const Eigen::Vector3d ps = eagle_gt.x.segment<3>(0);
        const Eigen::Vector3d tw = eagle_gt.x.segment<3>(3);
        state_interceptor_new.segment<3>(0) = ps;
        state_interceptor_new.segment<3>(3) = tw;

//        TODO handle track loss and do the re-initialisation
        if (m_is_kalman_initialized and dt <= m_time_th) {
//        if (m_is_kalman_initialized) {
            ROS_INFO_THROTTLE(5.0, "[%s]: kalman is initialised", m_nodename.c_str());

            if ((state_interceptor_new.head(3) - m_position_last_correction).norm() >= m_correction_th) {
                // Predict always when above th
                ROS_INFO_THROTTLE(5.0, "[%s]: kalman predict", m_nodename.c_str());
                if (m_approach == "dkf") {
                    dkf_t::A_t A = dkf_t::A_t::Identity();
                    Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Identity();
                    A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
                    B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt * dt / 2;
                    B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
                    m_dkf.A = A;

                    dkf_t::u_t u;
                    u.setZero();
                    Eigen::Matrix<double, 6, 6> Q;
                    if (m_use_qpv) {
                        Q = m_Qpv;
                    } else {
                        Q = B * m_Q * B.transpose();
                    }
                    try {
                        m_state_vec = m_dkf.predict(m_state_vec, u, Q, dt);
                    }
                    catch (const std::exception &e) {
                        ROS_ERROR("DKF predict failed: %s", e.what());
                    }
                } else if (m_approach == "dkft") {
                    m_state_vec_ang = dkf_pt_predict(m_state_vec_ang, dt);
                    m_state_vec.x = m_state_vec_ang.x.head(6);
                    m_state_vec.P = m_state_vec_ang.P.block<6, 6>(0, 0);
                } else if (m_approach == "plkf") {
                    std::tie(m_state_vec.x, m_state_vec.P) = plkf_predict(m_state_vec.x,
                                                                          m_state_vec.P,
                                                                          m_state_interceptor,
                                                                          state_interceptor_new,
                                                                          dt);
                } else if (m_approach == "plkft") {
                    std::tie(m_state_vec_ang.x, m_state_vec_ang.P) = plkf_pt_predict(m_state_vec_ang.x,
                                                                                     m_state_vec_ang.P,
                                                                                     dt);
                    m_state_vec.x = m_state_vec_ang.x.head(6);
                    m_state_vec.P = m_state_vec_ang.P.block<6, 6>(0, 0);
                }

                // Correct based on the distance moved from the previous position
                ROS_INFO_THROTTLE(5.0, "[%s]: kalman correct", m_nodename.c_str());
                m_position_last_correction = state_interceptor_new.head(3);
                if (m_approach == "dkf") {
                    try {
                        m_state_vec = m_dkf.correctLine(m_state_vec,
                                                        state_interceptor_new.head(3),
                                                        detection_vec,
                                                        m_var_dkf_radius);
                    }
                    catch (const std::exception &e) {
                        ROS_ERROR("DKF correct failed: %s", e.what());
                    }
                } else if (m_approach == "dkft") {
                    m_state_vec_ang = dkf_pt_correct(m_state_vec_ang,
                                                     state_interceptor_new.head(3),
                                                     detection_vec,
                                                     subtended_angle);
                    m_state_vec.x = m_state_vec_ang.x.head(6);
                    m_state_vec.P = m_state_vec_ang.P.block<6, 6>(0, 0);
                } else if (m_approach == "plkf") {
                    std::tie(m_state_vec.x, m_state_vec.P) = plkf_correct(m_state_vec.x,
                                                                          m_state_vec.P,
                                                                          state_interceptor_new.head(3),
                                                                          detection_vec);
                } else if (m_approach == "plkft") {
                    std::tie(m_state_vec_ang.x, m_state_vec_ang.P) = plkf_pt_correct(m_state_vec_ang.x,
                                                                                     m_state_vec_ang.P,
                                                                                     state_interceptor_new.head(3),
                                                                                     subtended_angle,
                                                                                     detection_vec);
                    m_state_vec.x = m_state_vec_ang.x.head(6);
                    m_state_vec.P = m_state_vec_ang.P.block<6, 6>(0, 0);
                }
            }
        } else {
            // initialise
            ROS_INFO("[%s]: INITIALISING KALMAN", m_nodename.c_str());
            m_dkf = dkf_t(dkf_t::A_t::Identity(), dkf_t::B_t::Zero());
            m_dkf_ang = dkf_ang_t(dkf_ang_t::A_t::Identity(), dkf_ang_t::B_t::Zero());
            Eigen::Matrix<double, 6, 1> x_k;
            x_k.setZero();
            if (not m_is_real_world and m_subh_target_odom.hasMsg()) {
                std::random_device rseed;
                std::mt19937 rng(rseed());
                std::normal_distribution<double> dist(0, m_tgt_stddev);
                double e_x = dist(rng), e_y = dist(rng), e_z = dist(rng);
                x_k.head(3) = tgt_gt.x.segment<3>(0) + Eigen::Vector3d{e_x, e_y, e_z};
                x_k.tail(3) = tgt_gt.x.segment<3>(3);
//                x_k.tail(3) = Eigen::Vector3d{0, 0, 0};

                m_state_vec.x = x_k;
                m_state_vec.P = m_P0;
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
// state for the DKF is the absolute position of the target in a common frame
                        x_k[0] = pos.x;
                        x_k[1] = pos.y;
                        x_k[2] = pos.z;
                        x_k[3] = vel.x;
                        x_k[4] = vel.y;
                        x_k[5] = vel.z;
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

            if (m_approach == "plkft" or m_approach == "dkft") {
                m_state_vec_ang.x.head(6) = m_state_vec.x;
                m_state_vec_ang.x(6) = m_init_tgt_width;
                m_state_vec_ang.P.block<6, 6>(0, 0) = m_P0;
                m_state_vec_ang.P(6, 6) = m_obj_size_variance;
            }
            ROS_INFO("[%s]: kalman is initialised", m_nodename.c_str());
            m_position_last_correction = state_interceptor_new.head(3);
            m_is_kalman_initialized = true;
        }

        dkf_ang_t::statecov_t est_res;
        if (m_approach == "plkft" or m_approach == "dkft") {
            est_res.x = m_state_vec_ang.x;
            est_res.P = m_state_vec_ang.P;
        } else {
            est_res.x.head(6) = m_state_vec.x;
            est_res.P.block<6, 6>(0, 0) = m_state_vec.P;
        }

        visualise_arrow(state_interceptor_new.head(3),
                        state_interceptor_new.head(3) + detection_vec,
                        detection_time, m_name_world_origin);
        postproc(est_res, detection_time, m_name_world_origin, tgt_gt, eagle_gt, ts_gt);

        m_state_interceptor = state_interceptor_new;
    }

// | -------------------- other functions ------------------- |
    std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>
    Masters::plkf_predict(const Eigen::Matrix<double, 6, 1> &xk_1,
                          const Eigen::Matrix<double, 6, 6> &Pk_1,
                          [[maybe_unused]] const Eigen::Matrix<double, 6, 1> &x_i,
                          [[maybe_unused]] const Eigen::Matrix<double, 6, 1> &x_i_,
                          const double &dt) {
        Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Identity();
        A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
        B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt * dt / 2;
        B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
        Eigen::Matrix<double, 6, 1> xk_ = A * xk_1;
        Eigen::Matrix<double, 6, 6> Q;
        if (m_use_qpv) {
            Q = m_Qpv;
        } else {
            Q = B * m_Q * B.transpose();
        }
        Eigen::Matrix<double, 6, 6> Pk_ = A * Pk_1 * A.transpose() + Q;
        return {xk_, Pk_};
    }

    std::tuple<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>>
    Masters::plkf_correct(const Eigen::Matrix<double, 6, 1> &xk_,
                          const Eigen::Matrix<double, 6, 6> &Pk_,
                          [[maybe_unused]]const Eigen::Vector3d &o,
                          const Eigen::Vector3d &lmb) {
        const Eigen::Matrix3d Plk = Eigen::Matrix3d::Identity() - lmb * lmb.transpose() / (lmb.norm() * lmb.norm());

        Eigen::Matrix<double, 3, 6> Hk;
        Hk.setZero();
        Hk.block<3, 3>(0, 0) = Plk;
//        it makes the uncertainty from ellipse to sphere...
        const auto rk = xk_.head(3).norm();
        const auto Vk = rk * Plk;
        Eigen::Matrix3d ps = Hk * Pk_ * Hk.transpose() + Vk * m_R * Vk.transpose();
//        Eigen::Matrix3d ps = Hk * Pk_ * Hk.transpose() + m_R;
        Eigen::Vector3d zk = Plk * o;
        Eigen::Matrix3d pinv = Masters::pseudoinverse(ps);
        Eigen::Matrix<double, 6, 3> K = Pk_ * Hk.transpose() * pinv;
        Eigen::Matrix<double, 6, 1> xk1 = xk_ + K * (zk - Hk * xk_);
        Eigen::Matrix<double, 6, 6> Pk1 = (Eigen::Matrix<double, 6, 6>::Identity() - K * Hk) * Pk_;
        return {xk1, Pk1};
    }


    std::tuple<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>>
    Masters::plkf_pt_predict(const Eigen::Matrix<double, 7, 1> &xk_1,
                             const Eigen::Matrix<double, 7, 7> &Pk_1,
                             const double &dt) {
        Eigen::Matrix<double, 7, 7> A = Eigen::Matrix<double, 7, 7>::Identity();
        A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Identity();
        B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt * dt / 2;
        B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
        Eigen::Matrix<double, 7, 7> sigma = Eigen::Matrix<double, 7, 7>::Identity();
        Eigen::Matrix<double, 6, 6> Q;
        if (m_use_qpv) {
            Q = m_Qpv;
        } else {
            Q = B * m_Q * B.transpose();
        }
        sigma.block<6, 6>(0, 0) = Q;
        sigma(6, 6) = m_obj_size_variance;
        Eigen::Matrix<double, 7, 1> xk_ = A * xk_1;
        Eigen::Matrix<double, 7, 7> Pk_ = A * Pk_1 * A.transpose() + sigma;
        return {xk_, Pk_};
    }

    std::tuple<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 7, 7>>
    Masters::plkf_pt_correct(const Eigen::Matrix<double, 7, 1> &xk_,
                             const Eigen::Matrix<double, 7, 7> &Pk_,
                             const Eigen::Vector3d &o,
                             const double theta,
                             const Eigen::Vector3d &lmb) {
        const Eigen::Matrix3d Plk = Eigen::Matrix3d::Identity() - lmb * lmb.transpose() / (lmb.norm() * lmb.norm());
        Eigen::Matrix<double, 6, 7> Hk = Eigen::Matrix<double, 6, 7>::Zero();

        Hk.block<3, 3>(0, 0) = Plk;
        Hk.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * theta;
        Hk.block<3, 1>(3, 6) = -lmb;
        const auto rk = xk_.head(3).norm();

        Eigen::Matrix<double, 6, 4> E = Eigen::Matrix<double, 6, 4>::Zero();
        E.block<3, 3>(0, 0) = rk * Plk;
        E.block<3, 3>(3, 0) = rk * Eigen::Matrix3d::Identity() * theta;
        E.block<3, 1>(3, 3) = -lmb;

//        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
//        R.block<3, 3>(0, 0) = m_R;
//        R(3, 3) = m_angle_variance;
//        R(4, 4) = m_angle_variance;
//        R(5, 5) = m_angle_variance;
        Eigen::Matrix<double, 4, 4> R =
                Eigen::Matrix<double, 4, 4>::Identity() * m_var_bearing_vector * m_var_bearing_vector;
        R(3, 3) = m_angle_variance * m_angle_variance;

        Eigen::Matrix<double, 6, 6> ps = Hk * Pk_ * Hk.transpose() + E * R * E.transpose();
//        Eigen::Matrix<double, 6, 6> ps = Hk * Pk_ * Hk.transpose() + R;
        Eigen::Matrix<double, 6, 6> pinv = Masters::invert_mat(ps);
        Eigen::Matrix<double, 6, 1> zk;
        zk.head(3) = Plk * o;
        zk.tail(3) = theta * o;
        Eigen::Matrix<double, 7, 6> K = Pk_ * Hk.transpose() * pinv;

        Eigen::Matrix<double, 7, 1> xk1 = xk_ + K * (zk - Hk * xk_);
        Eigen::Matrix<double, 7, 7> Pk1 = (Eigen::Matrix<double, 7, 7>::Identity() - K * Hk) * Pk_;
        return {xk1, Pk1};
    }

    Masters::dkf_ang_t::statecov_t
    Masters::dkf_pt_predict(const dkf_ang_t::statecov_t &sc, const double &dt) {
        Eigen::Matrix<double, 7, 7> A = Eigen::Matrix<double, 7, 7>::Identity();
        A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Identity();
        B.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt * dt / 2;
        B.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
        Eigen::Matrix<double, 7, 7> sigma = Eigen::Matrix<double, 7, 7>::Identity();
        Eigen::Matrix<double, 6, 6> Q;
        if (m_use_qpv) {
            Q = m_Qpv;
        } else {
            Q = B * m_Q * B.transpose();
        }
        sigma.block<6, 6>(0, 0) = Q;
        sigma(6, 6) = m_obj_size_variance;
        dkf_ang_t::statecov_t ret;
        ret.x = A * sc.x;
        ret.P = A * sc.P * A.transpose() + sigma;
        return ret;
    }

    Masters::dkf_ang_t::statecov_t
    Masters::dkf_pt_correct(const dkf_ang_t::statecov_t &sc, const Eigen::Vector3d &line_origin,
                            const Eigen::Vector3d &line_direction, const double &ang) {
        assert(line_direction.norm() > 0.0);
        // rotation between the (1, 0, 0) and line_direction
        const Eigen::Matrix3d rot = mrs_lib::geometry::rotationBetween(Eigen::Matrix<double, 3, 1>::UnitX(),
                                                                       line_direction);
        const Eigen::Matrix<double, 3, 2> N = rot.block<3, 2>(0, 1);

        const double theta = ang;
        Eigen::Matrix<double, 5, 1> z = Eigen::Matrix<double, 5, 1>::Zero();
        z.head(2) = N.transpose() * line_origin;
        z.tail(3) = theta * line_origin;
        auto rk = sc.x.head(3).norm();
        Eigen::Matrix<double, 5, 7> H = Eigen::Matrix<double, 5, 7>::Zero();
        H.block<2, 3>(0, 0) = N.transpose();
        H.block<3, 3>(2, 0) = Eigen::Matrix3d::Identity() * theta;
        H.block<3, 1>(2, 6) = -line_direction;
        Eigen::Matrix<double, 5, 7> E = Eigen::Matrix<double, 5, 7>::Zero();
        E.block<2, 3>(0, 0) = N.transpose();
        E.block<3, 3>(2, 3) = rk * Eigen::Matrix3d::Identity() * theta;
        E.block<3, 1>(5, 3) = -rk * line_direction;
        Eigen::Matrix<double, 7, 7> temp =
                Eigen::Matrix<double, 7, 7>::Identity() * m_var_dkf_radius * m_var_dkf_radius;
        temp.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * m_var_bearing_vector * m_var_bearing_vector;
        temp(6, 6) = m_angle_variance * m_angle_variance;

        const Eigen::Matrix<double, 5, 5> R = E * temp * E.transpose();
//        Eigen::Matrix<double, 5, 4> E = Eigen::Matrix<double, 5, 4>::Zero();
//        E.block<2, 3>(0, 0) = N.transpose();
//        E.block<3, 3>(2, 0) = rk * Eigen::Matrix3d::Identity() * theta;
//        E.block<3, 1>(2, 3) = -rk * line_direction;
//        Eigen::Matrix<double, 4, 4> temp = Eigen::Matrix4d::Identity() * m_var_dkf_radius;
//        temp(3, 3) = m_angle_variance;
//        const Eigen::Matrix<double, 5, 5> R = E * temp * E.transpose();
        // the correction phase
        dkf_ang_t::statecov_t ret;
        const dkf_ang_t::R_t W = H * sc.P * H.transpose() + R;
        const dkf_ang_t::R_t W_inv = invert_mat(W);
        const dkf_ang_t::K_t K = sc.P * H.transpose() * W_inv;
        auto inn = K * (z - (H * sc.x));
        ret.x = sc.x + inn;
        ret.P = (dkf_ang_t::P_t::Identity() - (K * H)) * sc.P;
        return ret;
    }

    [[maybe_unused]] void Masters::m_cbk_posearray_svd_pos_vel(const geometry_msgs::PoseArray &msg) {
        std::unique_lock lk(m_mut_tgt_gt);
        m_cv.wait(lk, [this] { return not m_gt_used; });
        dkf_ang_t::statecov_t tgt_gt = m_tgt_gt;
        dkf_ang_t::statecov_t eagle_gt = m_eagle_gt;
        m_gt_used = true;
        ros::Time ts_gt = m_gt_ts;
        lk.unlock();

        ROS_INFO_THROTTLE(5.0, "[%s]: SVD start %s", m_nodename.c_str(), m_approach.c_str());
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
        ROS_INFO_THROTTLE(5.0, "[%s]: history published", m_nodename.c_str());

        if (m_history_velocity.size() <= 1) {
            ROS_INFO_THROTTLE(5.0, "[%s]: history size is smaller/equal 1", m_nodename.c_str());
            m_history_linear.push_back(std::pair{pos_origin, pos_s});
            m_history_velocity.push_back(std::tuple{pos_origin, pos_s, msg.header.stamp});
            return;
        }
        if ((pos_origin - std::get<0>(m_history_velocity.back())).norm() > m_correction_th) {
            ROS_INFO_THROTTLE(5.0, "[%s]:  the uav is quite far away, updating", m_nodename.c_str());
            m_history_linear.push_back(std::pair{pos_origin, pos_s});
            m_history_velocity.push_back(std::tuple{pos_origin, pos_s, msg.header.stamp});
        }
        if (not m_t0.has_value()) {
            m_t0 = msg.header.stamp;
        }
        if (m_history_velocity.size() >= 4) {
//            ROS_INFO_THROTTLE(5.0, "[%s]: history size is more than 4", m_nodename.c_str());
            vec3 position, velocity;
            if (m_approach == "svd-dynamic") {
                std::tie(position, velocity) = m_find_intersection_svd_velocity_obj(m_history_velocity);
            } else if (m_approach == "svd-static") {
                position = m_find_intersection_svd_static_obj(m_history_linear);
                velocity = {0, 0, 0};
            } else {
                ROS_ERROR("[%s] unknown approach %s; shutting down...", m_nodename.c_str(), m_approach.c_str());
                // to avoid Wall unitialized
                position = {0, 0, 0};
                velocity = {0, 0, 0};
                ros::shutdown();
            }

            const auto time_now = msg.header.stamp;
            const double x = position.x() + velocity.x() * (time_now - m_t0.value()).toSec();
            const double y = position.y() + velocity.y() * (time_now - m_t0.value()).toSec();
            const double z = position.z() + velocity.z() * (time_now - m_t0.value()).toSec();
            dkf_ang_t::statecov_t res;
            Eigen::Matrix<double, 7, 1> state;
            state << x, y, z, velocity.x(), velocity.y(), velocity.z(), 0;
            res.x = state;
//            state << position.x(), position.y(), position.z(), velocity.x(), velocity.y(), velocity.z();
            postproc(res,
                     msg.header.stamp, m_name_world_origin, tgt_gt, eagle_gt, ts_gt);
            ROS_INFO_THROTTLE(5.0, "[%s]: visualising sphere...", m_nodename.c_str());
            visualise_sphere(state.head(3), msg.header.stamp, m_name_world_origin);
        }
    }

    void Masters::postproc(const dkf_ang_t::statecov_t &state, const ros::Time &t, const std::string &frame,
                           const dkf_ang_t::statecov_t &tgt_gt, const dkf_ang_t::statecov_t &eagle_gt, ros::Time &ts) {
        std::stringstream out;
        const Eigen::Matrix<double, 3, 3> c = state.P.block<3, 3>(0, 0);
        const Eigen::Vector3d ip = eagle_gt.x.head(3);
        const Eigen::Vector3d iv = eagle_gt.x.segment<3>(3);
        const Eigen::Vector3d tp = tgt_gt.x.head(3);
        const Eigen::Vector3d tv = tgt_gt.x.segment<3>(3);
        const Eigen::Vector3d ep = state.x.head(3);
        const Eigen::Vector3d ev = state.x.segment<3>(3);
        const double esz = state.x(6);
        out << ip.x() << ","
            << ip.y() << ","
            << ip.z() << ",";
        out << iv.x() << ","
            << iv.y() << ","
            << iv.z() << ",";
        out << tp.x() << ","
            << tp.y() << ","
            << tp.z() << ",";
        out << tv.x() << ","
            << tv.y() << ","
            << tv.z() << ",";
        out << ep.x() << ","
            << ep.y() << ","
            << ep.z() << ",";
        out << ev.x() << ","
            << ev.y() << ","
            << ev.z() << ",";
        out << esz << ","; // 18th
        out << c(0, 0) << ","
            << c(0, 1) << ","
            << c(0, 2) << ","
            << c(1, 0) << ","
            << c(1, 1) << ","
            << c(1, 2) << ","
            << c(2, 0) << ","
            << c(2, 1) << ","
            << c(2, 2) << ","; // 27th
        out << ts.sec << "," << ts.nsec;

        m_logfile.open(m_fnamestream, std::fstream::app);
        m_logfile << out.str() << std::endl;
//        std::cout << out.str() << std::endl;
        m_logfile.close();
        ROS_INFO_THROTTLE(5.0, "[%s]: visualising odometry...", m_nodename.c_str());
        visualise_odometry(state.x.head(6), state.P.block<6, 6>(0, 0), t, frame, m_pub_target_odom);
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
            A.block<3, 3>(r_idx, 3) = Eigen::Matrix3d::Identity() * (t - m_t0.value()).toSec();
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
        cv::Point2d pt;
        double theta;
        dkf_ang_t::statecov_t tgt_gt, eagle_gt;
        auto res = m_detect_uav_with_angle(msg);
        if (res.has_value()) {
            std::tie(pt, theta, tgt_gt, eagle_gt) = res.value();
            return pt;
        }
        return std::nullopt;
    }

    std::optional<std::tuple<cv::Point2d, double, Masters::dkf_ang_t::statecov_t, Masters::dkf_ang_t::statecov_t>>
    Masters::m_detect_uav_with_angle(const sensor_msgs::Image::ConstPtr &msg) {
        ROS_INFO_THROTTLE(5.0, "[%s]: detection started", m_nodename.c_str());
        if (not(m_subh_eagle_odom.hasMsg() and m_subh_target_odom.hasMsg())) {
            ROS_ERROR("[%s]: pose transformation for detection init has no value", m_nodename.c_str());
            return std::nullopt;
        }
        const auto msg_target_odom = m_subh_target_odom.getMsg();
        const auto msg_eagle_odom = m_subh_eagle_odom.getMsg();

        const auto ps_tgt = msg_target_odom->pose.pose.position;
        const auto vl_tgt = msg_target_odom->twist.twist.linear;
        const auto ps_egl = msg_eagle_odom->pose.pose.position;
        const auto vl_egl = msg_eagle_odom->twist.twist.linear;

        std::random_device rseed;
        std::mt19937 rng(rseed());
        std::normal_distribution<double> dist(0, m_eagle_stddev);
        double e_x = dist(rng), e_y = dist(rng), e_z = dist(rng);

        dkf_ang_t::statecov_t tgt_gt, eagle_gt;
        Eigen::Vector3d dir_vec, dir_vec_tmp;
        Eigen::Vector3d pos_tgt = {ps_tgt.x, ps_tgt.y, ps_tgt.z};
        Eigen::Vector3d vel_tgt = {vl_tgt.x, vl_tgt.y, vl_tgt.z};
        Eigen::Vector3d pos_eagle = {ps_egl.x, ps_egl.y, ps_egl.z};
        Eigen::Vector3d pos_eagle_noisy = {ps_egl.x + e_x, ps_egl.y + e_y, ps_egl.z + e_z};
        Eigen::Vector3d vel_eagle = {vl_egl.x, vl_egl.y, vl_egl.z};
        eagle_gt.x.head(3) = pos_eagle_noisy;
        eagle_gt.x.segment<3>(3) = vel_eagle;
        tgt_gt.x.head(3) = pos_tgt;
        tgt_gt.x.segment<3>(3) = vel_tgt;

        dir_vec_tmp = pos_tgt - pos_eagle;
        const auto new_dirvec = m_transformer.transformAsVector(m_name_world_origin,
                                                                dir_vec_tmp,
                                                                msg->header.frame_id,
                                                                msg->header.stamp);
        if (new_dirvec.has_value()) {
            dir_vec = new_dirvec.value();
        } else {
            ROS_ERROR("[%s]: unexpected error: no direction vector", m_nodename.c_str());
        }
        if (dir_vec.z() < 0) {
            ROS_ERROR("[%s]: pose is behind the image plane", m_nodename.c_str());
            return std::nullopt;
        }
        const auto pt_ideal = m_camera_main.project3dToPixel(cv::Point3d(dir_vec.x(), dir_vec.y(), dir_vec.z()));
        std::normal_distribution<double> dist_pt(m_mean, m_stddev);
        e_x = dist_pt(rng), e_y = dist_pt(rng);

        double offset = 10;
        const cv::Point2d pt_noisy{pt_ideal.x + e_x, pt_ideal.y + e_y};
        if (pt_noisy.x < offset or pt_noisy.x > m_camera_main.cameraInfo().width - offset or
            pt_noisy.y < offset or pt_noisy.y > m_camera_main.cameraInfo().height - offset) {
            return std::nullopt;
        }
        // compute the angle of the object
        // the optical frame is:
        // z fwd | x right | y down

//        orthogonal projection matrix
        const Eigen::Matrix3d rot = mrs_lib::geometry::rotationBetween(Eigen::Matrix<double, 3, 1>::UnitX(), dir_vec);
        const Eigen::Vector3d rotated_dirvec = rot.transpose() * dir_vec;
        std::cout << "rotated dirvec = " << rotated_dirvec << std::endl;
        Eigen::Vector3d vec_l = rot * Eigen::Vector3d{rotated_dirvec.x() - m_tgt_w, rotated_dirvec.y(), rotated_dirvec.z()};
        Eigen::Vector3d vec_r = rot * Eigen::Vector3d{rotated_dirvec.x() + m_tgt_w, rotated_dirvec.y(), rotated_dirvec.z()};
        std::cout << "l vec = " << vec_l << std::endl;
        std::cout << "r vec = " << vec_r << std::endl;

//        const auto vec_l = Eigen::Vector3d{dir_vec.x() - m_tgt_w, dir_vec.y(), dir_vec.z()};
//        const auto vec_r = Eigen::Vector3d{dir_vec.x() + m_tgt_w, dir_vec.y(), dir_vec.z()};
//        const auto vec_l = Eigen::Vector3d{dir_vec.x() - m_tgt_w, dir_vec.y(), dir_vec.z()};
//        const auto vec_r = Eigen::Vector3d{dir_vec.x() + m_tgt_w, dir_vec.y(), dir_vec.z()};
        const auto pt_l = m_camera_main.project3dToPixel(cv::Point3d{dir_vec.x() - m_tgt_w, dir_vec.y(), dir_vec.z()});
        const auto pt_r = m_camera_main.project3dToPixel(cv::Point3d{dir_vec.x() + m_tgt_w, dir_vec.y(), dir_vec.z()});
        // to avoid zero here because the "fake" tracking is used for the task
        if (pt_l.x > pt_r.x) {
            ROS_ERROR("[%s]: the bbox is wrong.", m_nodename.c_str());
            return std::nullopt;
        }
        const double s = std::max(pt_l.x - pt_r.x, 1.0);

        const auto cos_theta =
                (vec_l.norm() * vec_l.norm() + vec_r.norm() * vec_r.norm() - s * s) / (2 * vec_l.norm() * vec_r.norm());

        const double theta = std::acos(cos_theta);
        std::cout << "ANGLE = " << theta * 180 / 3.14 << std::endl;

        ROS_INFO_THROTTLE(5.0, "[%s]: uav detected", m_nodename.c_str());
        // visualize detection
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::circle(image, pt_noisy, 20, cv::Scalar(255, 0, 0), 2);
        m_pub_image_changed.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
        return std::tuple{pt_noisy, theta, tgt_gt, eagle_gt};
    }

    void Masters::visualise_odometry(const Eigen::Matrix<double, 6, 1> state,
                                     const Eigen::Matrix<double, 6, 6> covariance,
                                     const ros::Time &t,
                                     const std::string &frame,
                                     ros::Publisher &pb) {
        nav_msgs::Odometry msg;
        msg.header.stamp = t;
        msg.child_frame_id = frame;
        msg.header.frame_id = frame;
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

        pb.publish(msg);
    }

    void Masters::visualise_arrow(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &end,
                                  const ros::Time &detection_time,
                                  const std::string &frame) {

        visualization_msgs::Marker marker_detect;
        marker_detect.header.frame_id = frame;
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

    void Masters::visualise_sphere(const Eigen::Vector3d &pos, const ros::Time &t, const std::string &frame) {

        visualization_msgs::Marker marker_predict;
        marker_predict.header.frame_id = frame;
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
