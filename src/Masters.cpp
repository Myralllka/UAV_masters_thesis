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
        mrs_lib::construct_object(m_handler_camera_image,
                                  shopt,
                                  m_name_front_camera + "/image_raw");
        while (ros::ok()) {
            if (handler_camfrontinfo.hasMsg()) {
                camfront_info = *handler_camfrontinfo.getMsg().get();
                break;
            }
        }

        m_camera_front.fromCameraInfo(camfront_info);

        m_sub_front_camera_detection = nh.subscribe(m_name_front_camera + "/detection", 1,
                                                    &Masters::m_cbk_front_camera_detection, this);
        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer(nh, m_nodename);

        // | -------------------- initialize timers ------------------- |
        // Assume 30fps
        m_tim_uav2cam_projector = nh.createTimer(ros::Duration(0.0333), &Masters::m_tim_cbk_uav2cam_projector, this);
        ROS_INFO_ONCE("[%s]: initialized", m_nodename.c_str());
        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |
//    [[maybe_unused]] void Masters::m_callb_example([[maybe_unused]] const nav_msgs::Odometry::ConstPtr &msg) {
//        if (not m_is_initialized) return;
//    }

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
            std::cout << "pos new = " << pos_origin << std::endl;
            std::cout << "pos last = " << m_history.back().first << std::endl;
            std::cout << "norm = " << (pos_origin - m_history.back().first).norm() << std::endl;
            m_history.push_back(std::pair{pos_origin, pos_s});
        }
        if (m_history.size() >= 3) {

            auto res = m_find_intersection_svd(m_history);
            std::cout << res << std::endl;

            visualization_msgs::Marker marker;
            marker.header.frame_id = m_name_world_origin;
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::MODIFY;

            marker.pose.position.x = res.x();
            marker.pose.position.y = res.y();
            marker.pose.position.z = res.z();

            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            m_pub_viz.publish(marker);
        }

    }

// | --------------------- timer callbacks -------------------- |
    [[maybe_unused]] void Masters::m_tim_cbk_uav2cam_projector([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;
        cv::Mat image;

        std::random_device rseed;
        std::mt19937 rng(rseed());

        std::normal_distribution<double> dist(m_mean, m_stddev);
        geometry_msgs::TransformStamped T_eagle2drone;
        auto T_eagle2drone_opt = m_transformer.getTransform("uav2/fcu",
                                                            "uav91/bluefox_front_optical",
                                                            ros::Time(0));
        if (T_eagle2drone_opt.has_value()) {
            T_eagle2drone = T_eagle2drone_opt.value();
        } else { return; }
        Eigen::Vector3d vec = Eigen::Vector3d(tf2::transformToEigen(T_eagle2drone.transform).translation().data());
        while (ros::ok()) {
            if (m_handler_camera_image.hasMsg()) {
                image = cv_bridge::toCvCopy(m_handler_camera_image.getMsg(), "bgr8").get()->image;
                break;
            }
        }
        geometry_msgs::PoseArray res;
        double e1 = dist(rng), e2 = dist(rng), e3 = dist(rng);
        res.header.stamp = ev.current_real;
        res.header.frame_id = m_name_world_origin;
        auto T_eagle2world_opt = m_transformer.getTransform("uav91/bluefox_front_optical",
                                                            m_name_world_origin,
                                                            ros::Time(0));

        geometry_msgs::Pose p1, p2;
        if (T_eagle2world_opt.has_value()) {
            auto T_eagle2world = T_eagle2world_opt.value().transform.translation;
            p1.position.x = T_eagle2world.x;
            p1.position.y = T_eagle2world.y;
            p1.position.z = T_eagle2world.z;
            res.poses.push_back(p1);
        } else {
            return;
        }
        auto T_target2world_opt = m_transformer.getTransform("uav2/fcu",
                                                             m_name_world_origin,
                                                             ros::Time(0));
        if (T_target2world_opt.has_value()) {
            auto T_target2world = T_target2world_opt.value().transform.translation;
            p2.position.x = T_target2world.x + e1;
            p2.position.y = T_target2world.y + e2;
            p2.position.z = T_target2world.z + e3;
            res.poses.push_back(p2);
        } else {
            return;
        }
        vec.x() += e1;
        vec.y() += e2;
        vec.z() += e3;

        m_pub_front_camera_detection.publish(res);
        cv::Point2d pt = m_camera_front.project3dToPixel(cv::Point3d(vec.x(), vec.y(), vec.z()));
        cv::circle(image, pt, 20, cv::Scalar(255, 0, 0), 2);
        m_pub_image_changed.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());

    }


// | -------------------- other functions ------------------- |
    vec3 Masters::m_find_intersection_svd(const boost::circular_buffer<std::pair<vec3, vec3>> &data) {

        const int numPoints = data.size();
        const int numRows = 3 * numPoints;
        const int numCols = 3 + numPoints;

        Eigen::MatrixXd A(numRows, numCols);
        Eigen::VectorXd b(numRows);

        int rowIndex = 0;

        for (const auto &pair: data) {
            const Eigen::Vector3d &Os = pair.first;
            const Eigen::Vector3d &Ds = pair.second;

            Eigen::Vector3d ks = Os - Ds;

            A.block(rowIndex, 0, 3, 3) = Eigen::Matrix3d::Identity();
            A.block(rowIndex, 3 + rowIndex / 3, 3, 1) = ks;
            b.segment(rowIndex, 3) = Os;

            rowIndex += 3;
        }

        Eigen::VectorXd x = (A.transpose() * A).ldlt().solve(A.transpose() * b);

        // Extract the first 3 elements of x to get the intersection point.
        Eigen::Vector3d intersection = x.segment(0, 3);

        return intersection;
    }
}  // namespace masters

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(masters::Masters, nodelet::Nodelet)
