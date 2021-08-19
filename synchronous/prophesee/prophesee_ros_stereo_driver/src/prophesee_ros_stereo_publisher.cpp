/*******************************************************************
 * File : prophesee_ros_publisher.cpp                              *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#include <mutex>
#include <thread>

#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>

#include "prophesee_ros_stereo_publisher.h"

PropheseeWrapperStereoPublisher::PropheseeWrapperStereoPublisher():
  nh_("~"),
  biases_file_(""),
  master_left_(true)
{
  camera_name_left = "camera_left";
  camera_name_right = "camera_right";

  // Load Parameters
  // nh_.getParam("camera_name_left", camera_name_left);
  // nh_.getParam("camera_name_right", camera_name_right);
  nh_.getParam("publish_cd", publish_cd_);
  nh_.getParam("bias_file", biases_file_);
  nh_.getParam("master_left",master_left_);
  nh_.getParam("left_camera_id",left_camera_id);
  nh_.getParam("right_camera_id", right_camera_id);
  nh_.getParam("trigger_reconstruct", trigger_reconstruct_);
  nh_.getParam("trigger_separate_usec", trigger_separate_usec_);

  const std::string topic_cam_info_left = "/prophesee/" + camera_name_left + "/camera_info";
  const std::string topic_cd_event_buffer_left = "/prophesee/" + camera_name_left + "/cd_events_buffer";

  const std::string topic_cam_info_right = "/prophesee/" + camera_name_right + "/camera_info";
  const std::string topic_cd_event_buffer_right = "/prophesee/" + camera_name_right + "/cd_events_buffer";

  const std::string topic_cd_event_buffer_left_triggered = "/prophesee/" + camera_name_left + "/cd_events_buffer_reconstructed";
  const std::string topic_cd_event_buffer_right_triggered = "/prophesee/" + camera_name_right + "/cd_events_buffer_reconstructed";

  pub_info_left = nh_.advertise<sensor_msgs::CameraInfo>(topic_cam_info_left, 1);
  pub_info_right = nh_.advertise<sensor_msgs::CameraInfo>(topic_cam_info_right, 1);

  if (publish_cd_){
    if (!trigger_reconstruct_) {
      pub_cd_events_left = nh_.advertise<prophesee_event_msgs::EventArray>(topic_cd_event_buffer_left, 500);
      pub_cd_events_right = nh_.advertise<prophesee_event_msgs::EventArray>(topic_cd_event_buffer_right, 500);
    } else {
      pub_cd_events_left = nh_.advertise<prophesee_event_msgs::EventArray>(topic_cd_event_buffer_left_triggered, 500);
      pub_cd_events_right = nh_.advertise<prophesee_event_msgs::EventArray>(topic_cd_event_buffer_right_triggered, 500);
    }
  }

  while (!openCamera(camera_left)) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_INFO("Trying to open left camera...");
  }

  while (!openCamera(camera_right)) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ROS_INFO("Trying to open right camera...");
  }

  // Add camera runtime error callback
  camera_left.add_runtime_error_callback(
        [](const Metavision::CameraException &e) { ROS_WARN("%s", e.what()); });
  camera_right.add_runtime_error_callback(
        [](const Metavision::CameraException &e) { ROS_WARN("%s", e.what()); });

  // Get the sensor config
  Metavision::CameraConfiguration config = camera_left.get_camera_configuration();
  ROS_INFO("[CONF] INFORMATION ON LEFT CAMERA");
  ROS_INFO("[CONF] Width:%i, Height:%i", camera_left.geometry().width(), camera_left.geometry().height());
  ROS_INFO("[CONF] Serial number: %s", config.serial_number.c_str());

  config = camera_right.get_camera_configuration();
  ROS_INFO("[CONF] INFORMATION ON RIGHT CAMERA");
  ROS_INFO("[CONF] Width:%i, Height:%i", camera_right.geometry().width(), camera_right.geometry().height());
  ROS_INFO("[CONF] Serial number: %s", config.serial_number.c_str());

  // Publish camera info message
  cam_info_msg_left.width = uint32_t(camera_left.geometry().width());
  cam_info_msg_left.height = uint32_t(camera_left.geometry().height());
  cam_info_msg_left.header.frame_id = "PropheseeCamera_optical_frame_left";

  cam_info_msg_right.width = uint32_t(camera_right.geometry().width());
  cam_info_msg_right.height = uint32_t(camera_right.geometry().height());
  cam_info_msg_right.header.frame_id = "PropheseeCamera_optical_frame_right";
}

PropheseeWrapperStereoPublisher::~PropheseeWrapperStereoPublisher() {
  camera_left.stop();
  camera_right.stop();

  nh_.shutdown();
}

bool PropheseeWrapperStereoPublisher::openCamera(Metavision::Camera & camera) {
  bool camera_is_opened = false;
  // Initialize the camera instance
  try {
    if(&camera==&camera_left){
      camera = Metavision::Camera::from_serial(left_camera_id);
      Metavision::I_DeviceControl * i_device_control = camera.get_device().get_facility<Metavision::I_DeviceControl>();
      ROS_INFO("Left camera was opened successfully");
      if (master_left_){
        if (i_device_control->set_mode_master())ROS_INFO("Left camera was opened as Master mode successfully");
        else ROS_ERROR("Left camera was UNABLE to open as Master mode!");
      } else {
        if (i_device_control->set_mode_slave()) ROS_INFO("Left camera was opened as Slave mode successfully");
        else ROS_ERROR("Left camera was UNABLE to open as Slave mode!");
      }
    } else if (&camera==&camera_right) {
      camera = Metavision::Camera::from_serial(right_camera_id);
      Metavision::I_DeviceControl * i_device_control = camera.get_device().get_facility<Metavision::I_DeviceControl>();
      ROS_INFO("Right Camera was opened successfully");
      if (master_left_){
        if (i_device_control->set_mode_slave()) ROS_INFO("Right camera was opened as Slave mode successfully");
        else ROS_ERROR("Right camera was UNABLE to open as Slave mode!");
      } else {
        if (i_device_control->set_mode_master())ROS_INFO("Right camera was opened as Master mode successfully");
        else ROS_ERROR("Right camera was UNABLE to open as Master mode!");
      }
      if (trigger_reconstruct_) {
        Metavision::I_TriggerIn *i_trigger_in = camera.get_device().get_facility<Metavision::I_TriggerIn>();
        if (!i_trigger_in->enable(0) && !i_trigger_in->enable(1)) ROS_WARN("Left camera NOT SYNC SUCC");
        else ROS_INFO("Left camera was opened external trigger mode successfully");
      }
    } else {
      ROS_WARN("Invalid camera Reference in call to PropheseeWrapperStereoPublisher::openCamera");
    }
    if (!biases_file_.empty()) {
      ROS_INFO("[CONF] Loading bias file: %s", biases_file_.c_str());
      camera.biases().set_from_file(biases_file_);
    }
    camera_is_opened = true;
  } catch (Metavision::CameraException &e) {
    ROS_WARN("%s", e.what());
  }
  return camera_is_opened;
}

void PropheseeWrapperStereoPublisher::startPublishing() {
  camera_right.start();
  camera_left.start();

  start_timestamp_ = ros::Time::now();
  ROS_INFO("Timestamp is %d seconds and %d nanoseconds",start_timestamp_.sec,start_timestamp_.nsec);

  if (publish_cd_){
    publishCDEvents(camera_left, pub_cd_events_left);
    publishCDEvents(camera_right, pub_cd_events_right);
    if (trigger_reconstruct_)
    extTriggerCallBack();
  }

  ros::Rate loop_rate(5);
  while(ros::ok()) {
    if (pub_info_left.getNumSubscribers() > 0) {
      cam_info_msg_left.header.stamp = ros::Time::now();
      cam_info_msg_right.header.stamp = ros::Time::now();
      pub_info_left.publish(cam_info_msg_left);
      pub_info_right.publish(cam_info_msg_right);
    }
    loop_rate.sleep();
  }
}

void PropheseeWrapperStereoPublisher::publishCDEvents(Metavision::Camera & camera, ros::Publisher & publisher) {
  // Initialize and publish a buffer of CD events
  try {
    auto cd_callback_fun=[this,&camera,&publisher](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
      // Check the number of subscribers to the topic
      if (publisher.getNumSubscribers() <= 0)
        return;

      if (ev_begin < ev_end) {
        // Define the message for a buffer of CD events
        prophesee_event_msgs::EventArray event_buffer_msg;
        const unsigned int buffer_size = unsigned(ev_end - ev_begin);
        event_buffer_msg.events.resize(buffer_size);

        // Header Timestamp of the message
        if (!trigger_reconstruct_){
          event_buffer_msg.header.stamp.fromNSec(start_timestamp_.toNSec() + (ev_begin->t * 1000.00));
        } else {
          event_buffer_msg.header.stamp.fromNSec(
              t_indexed_gt_ + ev_begin->t > t_current_internal_clock_ ?
                  (t_indexed_gt_ - t_current_internal_clock_ + ev_begin->t) * 1000 : 0);
        }

        // Sensor geometry in header of the message
        event_buffer_msg.height = unsigned(camera.geometry().height());
        event_buffer_msg.width = unsigned(camera.geometry().width());

        // if to long time no signal or not started turn off sycn and warning
        if (trigger_reconstruct_ && (!has_started_ ||
                                     ev_begin->t > t_current_internal_clock_ + trigger_separate_usec_ * 50))
        {
          has_started_ = false;
          t_indexed_gt_ = 0;
          ROS_WARN("There is no trigger now! Please Check");
          return;
        }

        // Add events to the message
        auto buffer_it = event_buffer_msg.events.begin();
        for (const Metavision::EventCD *it = ev_begin; it != ev_end; ++it, ++buffer_it) {
          prophesee_event_msgs::Event &event = *buffer_it;
          event.x = it->x;
          event.y = it->y;
          event.polarity = it->p;
          if (!trigger_reconstruct_) event.ts.fromNSec(start_timestamp_.toNSec() + (it->t * 1000.00));
          else {
            if (t_indexed_gt_ + it->t > t_current_internal_clock_)
                event.ts.fromNSec((t_indexed_gt_ - t_current_internal_clock_ + it->t) * 1000);
          }
        }

        // Publish the message
        publisher.publish(event_buffer_msg);

        ROS_DEBUG("CD data available, buffer size: %d at time: %llu", buffer_size, ev_begin->t);
      }
    };
    [[gnu::unused]] Metavision::CallbackId cd_callback_ = camera.cd().add_callback(cd_callback_fun);
    ROS_INFO("register camera cd callback with id %d", cd_callback_);
  } catch (Metavision::CameraException &e) {
    ROS_WARN("%s", e.what());
    publish_cd_ = false;
  }
}

void PropheseeWrapperStereoPublisher::extTriggerCallBack() {
  // Initialize and publish a buffer of CD events
  try {
    Metavision::CallbackId ext_trigger_callback = camera_right.ext_trigger().add_callback(
      [this](const Metavision::EventExtTrigger *begin, const Metavision::EventExtTrigger *end) {
        auto it_begin = begin;
        auto it_end   = end;
        for (it_begin; it_begin < it_end; it_begin++) {
          if (it_begin->p) {
            has_started_ = true;
            t_indexed_gt_ += trigger_separate_usec_;
            t_current_internal_clock_ = it_begin->t;
          }
        }
      });
    ROS_INFO("register camera ext_trigger callback with id %d", ext_trigger_callback);
  } catch (Metavision::CameraException &e) {
    ROS_WARN("%s", e.what());
    publish_cd_ = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "prophesee_ros_publisher");

  PropheseeWrapperStereoPublisher wp;
  wp.startPublishing();

  ros::shutdown();

  return 0;
}
