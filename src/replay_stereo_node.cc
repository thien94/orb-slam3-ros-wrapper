/*******************************************************
Directly process from a rosbag, for the stereo case only.

Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc

 *******************************************************/
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include "common.h"

using namespace std;

class ImageGrabber {
   public:
    ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM) {}

    void GrabLeft(const sensor_msgs::ImageConstPtr& msgLeft);
    void GrabRight(const sensor_msgs::ImageConstPtr& msgRight);

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                    const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    cv::Mat M1l, M2l, M1r, M2r;

    sensor_msgs::ImageConstPtr left_image, right_image;
};

void ImageGrabber::GrabLeft(const sensor_msgs::ImageConstPtr& msgLeft)
{
    left_image = msgLeft;
    if (right_image) {
        const auto time_diff_s =
            abs(right_image->header.stamp.toSec() - left_image->header.stamp.toSec());
        if (time_diff_s > 0.01) {
            ROS_INFO("Time diff %0.3f s, skipping", time_diff_s);
            return;
        }

        GrabStereo(left_image, right_image);
    }

    left_image = nullptr;
    right_image = nullptr;
}

void ImageGrabber::GrabRight(const sensor_msgs::ImageConstPtr& msgRight)
{
    right_image = msgRight;
    if (left_image) {
        const auto time_diff_s =
            abs(right_image->header.stamp.toSec() - left_image->header.stamp.toSec());
        if (time_diff_s > 0.01) {
            ROS_INFO("Time diff %0.3f s, skipping", time_diff_s);
            return;
        }

        GrabStereo(left_image, right_image);
    }

    left_image = nullptr;
    right_image = nullptr;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                              const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Main algorithm runs here
    Sophus::SE3f Tcc0 = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image,
                                            cv_ptrLeft->header.stamp.toSec());
    Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();

    ros::Time msg_time = cv_ptrLeft->header.stamp;

    publish_ros_camera_pose(Twc, msg_time);
    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
    publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "orbslam3_ros_wrapper_stereo_replay");
    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // Read params from roslaunch
    std::string voc_file, settings_file, traj_save_file, rosbag_path;
    std::string left_cam_topic, right_cam_topic;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/traj_save_file", traj_save_file, "");
    node_handler.param<std::string>(node_name + "/rosbag_path", rosbag_path, "");
    node_handler.param<std::string>(node_name + "/left_cam_topic", left_cam_topic, "");
    node_handler.param<std::string>(node_name + "/right_cam_topic", right_cam_topic, "");

    if (voc_file == "file_not_set" || settings_file == "file_not_set") {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }

    if (rosbag_path == "") {
        ROS_ERROR("Please provide rosbag_path in the launch file");
        ros::shutdown();
        return 1;
    }

    if (left_cam_topic == "" || right_cam_topic == "") {
        ROS_ERROR("Please provide left_cam_topic and right_cam_topic in the launch file");
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // World frame orientation
    Eigen::Vector3d rpy_rad;
    std::string angle_names[3] = {"roll", "pitch", "yaw"};
    for (int i = 0; i < 3; i++) {
        node_handler.param<double>(node_name + "/world_" + angle_names[i], rpy_rad(i), 0);
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::STEREO;
    ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);
    ImageGrabber igb(&SLAM);

    ROS_INFO("Opening rosbag: %s", rosbag_path.c_str());
    // Open rosbag
    rosbag::Bag bag;
    bag.open(rosbag_path, rosbag::bagmode::Read);

    // TODO Setup ROS publishers for the relevant topcis
    // registerPub(n);
    // ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>(IMAGE0_TOPIC, 1000);
    // ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>(IMAGE1_TOPIC, 1000);

    // Read topics from rosbag
    std::vector<std::string> topics_to_read;
    topics_to_read.push_back(std::string(left_cam_topic));
    topics_to_read.push_back(std::string(right_cam_topic));
    rosbag::View view(bag, rosbag::TopicQuery(topics_to_read));

    // Check if we are faster than just rosbag play :)
    auto time_start = std::chrono::high_resolution_clock::now();

    for (rosbag::MessageInstance const m : view) {
        // ROS_DEBUG("Read topic [%s]", m.getTopic().c_str());
        if (!ros::ok()) {
            break;
        }

        if (m.getTopic() == std::string(left_cam_topic)) {
            sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
            igb.GrabLeft(img_msg);
        }
        else if (m.getTopic() == std::string(right_cam_topic)) {
            sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
            igb.GrabRight(img_msg);
        }
        else {
            ROS_WARN("Ignoring topic %s", m.getTopic().c_str());
            continue;
        }
    }

    auto time_end = std::chrono::high_resolution_clock::now();
    auto duration_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
    ROS_WARN("Loop duration: %0.2fd ms | Effective freq: %0.2f Hz", duration_ms,
             1000.0 / duration_ms);

    bag.close();

    // Save trajectory if requested
    if (traj_save_file != "") {
        ROS_INFO("Saving trajectory to %s", traj_save_file.c_str());
        SLAM.SaveTrajectoryTUM(traj_save_file);
    }
    else {
        ROS_INFO("Trajectory file not provided, not saved");
    }

    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}
