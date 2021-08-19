/*******************************************************************
 * File : prophesee_ros_publisher.h                                *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#ifndef PROPHESEE_ROS_PUBLISHER_H_
#define PROPHESEE_ROS_PUBLISHER_H_

#include <sensor_msgs/CameraInfo.h>

#include <metavision/sdk/driver/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>

/// \brief Main class for ROS publisher
///
/// Publishes data from Prophesee sensor to ROS topics
class PropheseeWrapperPublisher {
public:
    /// \brief Constructor
    PropheseeWrapperPublisher();

    /// \brief Destructor
    ~PropheseeWrapperPublisher();

    /// \brief Starts the camera and starts publishing data
    void startPublishing();

private:
    /// \brief Opens the camera
    bool openCamera();

    /// \brief Publishes CD events
    void publishCDEvents();

    /// \brief Publishes ExtTrigger
    void extTriggerCallBack();

    /// \brief Node handler - the access point to communication with ROS
    ros::NodeHandle nh_;

    /// \brief Publisher for camera info
    ros::Publisher pub_info_;

    /// \brief Publisher for CD events
    ros::Publisher pub_cd_events_;

    /// \brief Instance of Camera class
    ///
    /// Used to access data from a camera
    Metavision::Camera camera_;

    /// \brief Instance of Events Array
    ///
    /// Accumulated Array of events
    std::vector<Metavision::EventCD> event_buffer_;

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

    /// \brief Message for publishing the camera info
    sensor_msgs::CameraInfo cam_info_msg_;

    /// \brief Path to the file with the camera settings (biases)
    std::string biases_file_;

    /// \brief Raw file to read instead of live camera
    std::string raw_file_to_read_;

    /// \brief Camera name in string format
    std::string camera_name_;

    /// \brief Wall time stamps
    ros::Time start_timestamp_, last_timestamp_;

    /// \brief If showing CD events
    bool publish_cd_;

    /// \brief If using reconstruct CD events with EXT trigger
    bool trigger_reconstruct_;

    /// \brief Time interval between two rising edge of EXT trigger in microseconds
    int trigger_separate_usec_;

    /// \brief Activity Filter Temporal depth (configuration)
    /// Desirable Temporal depth in micro seconds
    int activity_filter_temporal_depth_;

    /// \brief delta_time for packages cd_events
    /// Time step for packaging events in an array
    ros::Duration event_delta_t_;

    /// \brief Event buffer time stamps
    ros::Time event_buffer_start_time_, event_buffer_current_time_;

    /// \brief  Mean gravity value at Earth surface [m/s^2]
    static constexpr double GRAVITY = 9.81;

    /// \brief delta time of cd events fixed by the driver
    /// The delta time is set to a fixed number of 64 microseconds (1e-06)
    static constexpr double EVENT_DEFAULT_DELTA_T = 6.4e-05;
};

#endif /* PROPHESEE_ROS_PUBLISHER_H_ */
