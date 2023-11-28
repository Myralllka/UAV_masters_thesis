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
        pl.loadParam("deviation", m_stddev);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", m_nodename.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", m_nodename.c_str());
        }
        // | ---------------- some data post-processing --------------- |
        m_history = boost::circular_buffer<std::pair<vec3, vec3>>(m_history_bufsize);

        // | ----------------- publishers initialize ------------------ |
        m_pub_image_changed = nh.advertise<sensor_msgs::Image>(m_name_front_camera + "/changed", 1);
        m_pub_front_camera_detection = nh.advertise<geometry_msgs::PoseArray>(m_name_front_camera + "/detection", 1);
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

        m_camera_front.fromCameraInfo(camfront_info);

        m_sub_front_camera_detection = nh.subscribe(m_name_front_camera + "/detection", 1,
                                                    &Masters::m_cbk_front_camera_detection, this);
        m_sub_front_camera = nh.subscribe(m_name_front_camera + "/image_raw", 1,
                                          &Masters::m_cbk_front_camera, this);
        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer(nh, m_nodename);

        // | -------------------- initialize timers ------------------- |
        ROS_INFO_ONCE("[%s]: initialized", m_nodename.c_str());
        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |

    [[maybe_unused]] void Masters::m_cbk_front_camera_detection(const geometry_msgs::PoseArray &msg) {
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
        for (const auto &h: m_history) {
            geometry_msgs::Pose p;
            p.position.x = h.first.x();
            p.position.y = h.first.y();
            p.position.z = h.first.z();
            ps1.poses.push_back(p);

            geometry_msgs::Pose p2;
            p2.position.x = h.second.x();
            p2.position.y = h.second.y();
            p2.position.z = h.second.z();
            ps2.poses.push_back(p2);
        }
        m_pub_history1.publish(ps1);
        m_pub_history2.publish(ps2);

        if (m_history.size() <= 1) {
            m_history.push_back(std::pair{pos_origin, pos_s});
            return;
        }
        if ((pos_origin - m_history.back().first).norm() > m_upd_th) {
            m_history.push_back(std::pair{pos_origin, pos_s});
        }
        if (m_history.size() >= 3) {

            auto res = m_find_intersection_svd(m_history);
            std::cout << res << std::endl;

            visualization_msgs::Marker marker;
            marker.header.frame_id = m_name_world_origin;
            marker.header.stamp = msg.header.stamp;
            marker.ns = "my_namespace";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::MODIFY;

            marker.pose.position.x = res.x();
            marker.pose.position.y = res.y();
            marker.pose.position.z = res.z();

            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            m_pub_viz.publish(marker);
        }

    }

    [[maybe_unused]] void Masters::m_cbk_front_camera(const sensor_msgs::ImageConstPtr &msg) {

        if (not m_is_initialized) return;
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


// | -------------------- other functions ------------------- |
    vec3 Masters::m_find_intersection_svd(const boost::circular_buffer<std::pair<vec3, vec3>> &data) {

        const int n_pts = data.size();
        const int n_r = 3 * n_pts;
        const int n_c = 3 + n_pts;

        Eigen::MatrixXd A(n_r, n_c);
        A.fill(0);
        Eigen::VectorXd b(n_r);

        int r_idx = 0;
        std::cout << data.size() << std::endl;
        for (const auto &pair: data) {
            const Eigen::Vector3d &Os = pair.first;
            const Eigen::Vector3d &Ds = pair.second;

            Eigen::Vector3d ks = Ds - Os;

            A.block(r_idx, 0, 3, 3) = Eigen::Matrix3d::Identity();
            A.block(r_idx, 3 + r_idx / 3, 3, 1) = ks;
            b.segment(r_idx, 3) = Os;

            r_idx += 3;
        }
        Eigen::BDCSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

        Eigen::VectorXd x = SVD.solve(b);
        // Extract the first 3 elements of x to get the intersection point.
        Eigen::Vector3d intersection = x.segment(0, 3);

        return intersection;
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
        return cv::Point2d{pt_ideal.x + e_x, pt_ideal.y + e_y};
    }
}  // namespace masters

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(masters::Masters, nodelet::Nodelet)
