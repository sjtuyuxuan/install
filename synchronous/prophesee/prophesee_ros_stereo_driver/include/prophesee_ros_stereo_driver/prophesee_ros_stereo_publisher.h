/*******************************************************************
 * File : prophesee_ros_publisher.h                                *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef STEREO_ROS_PUBLISHER_H_
#define STEREO_ROS_PUBLISHER_H_

#include <sensor_msgs/CameraInfo.h>

#include <metavision/sdk/driver/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_device_control.h>

#include "ros/ros.h"

/// \brief Main class for ROS publisher
///
/// Publishes data from Prophesee sensor to ROS topics
class PropheseeWrapperStereoPublisher
{
public:
    /// \brief Constructor
    PropheseeWrapperStereoPublisher();

    /// \brief Destructor
    ~PropheseeWrapperStereoPublisher();

    /// \brief Starts the camera and starts publishing data
    void startPublishing(); ////TODO////

private:
    ///
    /// \brief Opens the camera that is being passed
    /// \param camera_ reference to one of the camera object (left/right)
    /// \return Returns if camera opening was successful
    ///
    bool openCamera(Metavision::Camera &camera_);

    /// \brief Publishes CD events
    void publishCDEvents(Metavision::Camera &camera, ros::Publisher &publisher);

    /// \brief ExtTrigger CallBack
    void extTriggerCallBack();

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh_;

    /// \brief Publisher for camera info
    ros::Publisher pub_info_left;
    ros::Publisher pub_info_right;

    /// \brief Publisher for CD events
    ros::Publisher pub_cd_events_left;
    ros::Publisher pub_cd_events_right;

    /// \brief Instance of Camera class for left camera
    ///
    /// Used to access data from the left camera. Serial
    /// numer to left/right are taken from ros parameter
    /// space
    Metavision::Camera camera_left;

    /// \brief Instance of Camera class for right camera
    ///
    /// Used to access data from the right camera. Serial
    /// numer to left/right are taken from ros parameter
    /// space
    Metavision::Camera camera_right;

    /// \brief Message for publishing the camera info
    sensor_msgs::CameraInfo cam_info_msg_left;
    sensor_msgs::CameraInfo cam_info_msg_right;

    /// \brief Path to the file with the camera settings (biases)
    std::string biases_file_;

    /// \brief left camera name in string format
    std::string camera_name_left;

    /// \brief Left camera serial in string format
    std::string left_camera_id;

    /// \brief right camera name in string format
    std::string camera_name_right;

    /// \brief Right camera serial in string format
    std::string right_camera_id;

    /// \brief Camera string time
    ros::Time start_timestamp_;

    bool master_left_;

    /// \brief If showing CD events
    bool publish_cd_;

    /// \brief Instance of t_current_
    ///
    /// Accumulated t when sync is used
    Metavision::timestamp t_indexed_gt_ = 0;

    /// \brief Instance of trigger flag
    ///
    /// Check wether external trigger is starated
    bool has_started_ = false;

    /// \brief Instance of t_current_
    ///
    /// Accumulated t when sync is used
    Metavision::timestamp t_current_internal_clock_ = 0;

    /// \brief If using reconstruct CD events with EXT trigger
    bool trigger_reconstruct_;

    /// \brief Time interval between two rising edge of EXT trigger in microseconds
    int trigger_separate_usec_;

    static constexpr double GRAVITY = 9.81; /** Mean gravity value at Earth surface [m/s^2] **/
};

#endif /* STEREO_PUBLISHER_H_ */
