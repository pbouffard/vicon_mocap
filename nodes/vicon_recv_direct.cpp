/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <Client.h>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <vicon_mocap/viconGrabPose.h>
#include <iostream>
#include <algorithm>

using std::min;
using std::max;
using std::string;

using namespace ViconDataStreamSDK::CPP;

string Adapt(const Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

class ViconReceiver
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;
  double min_freq;
  double max_freq;
  diagnostic_updater::FrequencyStatus freq_status;
  // Parameters:
  string StreamMode;
  string HostName;
  string SubjectName;
  string SegmentName;
  string tf_ref_frame_id;
  string tf_tracked_frame_id;
  double update_rate;
  double vicon_capture_rate;
  // Publisher
  ros::Publisher pose_pub;
  // Timer
  ros::Timer updateTimer;
  // TF Broadcaster
  tf::TransformBroadcaster tf_broadcast;
  //geometry_msgs::PoseStamped vicon_pose;
  tf::Transform flyer_transform;
  ros::Time now_time;
  // TODO: Make the following configurable:
  ros::ServiceServer m_grab_vicon_pose_service_server;
  ViconDataStreamSDK::CPP::Client MyClient;
  double max_period_between_updates;
  double last_callback_duration;
  unsigned int lastFrameNumber;
  unsigned int frameCount;
  unsigned int droppedFrameCount;

public:
  ViconReceiver() :
    nh_priv("~"), diag_updater(), min_freq(0.1), max_freq(1000),
        freq_status(diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq)), StreamMode("ClientPullPreFetch"),
        HostName(""), SubjectName(""), SegmentName(""), tf_ref_frame_id("/enu"),
        tf_tracked_frame_id("/pelican1/flyer_vicon"), update_rate(100), vicon_capture_rate(50), lastFrameNumber(0),
        frameCount(0), droppedFrameCount(0)
  {
    // Diagnostics
    diag_updater.add("ViconReceiver Status", this, &ViconReceiver::diagnostics);
    diag_updater.add(freq_status);
    diag_updater.setHardwareID("none");
    diag_updater.force_update();
    // Parameters
    nh_priv.param("stream_mode", StreamMode, StreamMode);
    nh_priv.param("datastream_hostport", HostName, HostName);
    nh_priv.param("subject_name", SubjectName, SubjectName);
    nh_priv.param("segment_name", SegmentName, SegmentName);
    nh_priv.param("update_rate", update_rate, update_rate);
    nh_priv.param("vicon_capture_rate", vicon_capture_rate, vicon_capture_rate);
    nh_priv.param("tf_ref_frame_id", tf_ref_frame_id, tf_ref_frame_id);
    nh_priv.param("tf_tracked_frame_id", tf_tracked_frame_id, tf_tracked_frame_id);
    ROS_ASSERT(init_vicon());
    // Service Server
    ROS_INFO("setting up grab_vicon_pose service server ... ");
    m_grab_vicon_pose_service_server = nh_priv.advertiseService("grab_vicon_pose",
                                                                &ViconReceiver::grab_vicon_pose_callback, this);
    // Publisher
    pose_pub = nh_priv.advertise<geometry_msgs::TransformStamped> ("output", 1);
    // Timer
    double update_timer_period = 1 / update_rate;
    min_freq = 0.95 * update_rate;
    max_freq = 1.05 * update_rate;
    updateTimer = nh.createTimer(ros::Duration(update_timer_period), &ViconReceiver::updateCallback, this);
  }

  ~ViconReceiver()
  {
    ROS_ASSERT(shutdown_vicon());
  }

private:
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    stat.add("max period between updates", max_period_between_updates);
    stat.add("latest callback runtime", last_callback_duration);
    stat.add("latest VICON frame number", lastFrameNumber);
    stat.add("dropped frames", droppedFrameCount);
    stat.add("framecount", frameCount);
  }

  bool init_vicon()
  {
    ROS_INFO_STREAM("Connecting to Vicon DataStream SDK at " << HostName << " ...");

    while (!MyClient.IsConnected().Connected)
    {
      MyClient.Connect(HostName);
      ROS_INFO(".");
      sleep(1);
      ros::spinOnce();
      if (!ros::ok())
        return false;
    }
    ROS_ASSERT(MyClient.IsConnected().Connected);
    ROS_INFO_STREAM("... connected!");
    MyClient.EnableSegmentData();
    ROS_ASSERT(MyClient.IsSegmentDataEnabled().Enabled);
    if (StreamMode == "ServerPush")
    {
      MyClient.SetStreamMode(StreamMode::ServerPush); // TODO: make configurable(?)
    }
    else if (StreamMode == "ClientPull")
    {
      MyClient.SetStreamMode(StreamMode::ClientPull);
    }
    else if (StreamMode == "ClientPullPreFetch")
    {
      MyClient.SetStreamMode(StreamMode::ClientPullPreFetch);
    }
    else
    {
      ROS_FATAL("Unknown stream mode -- options are ServerPush, ClientPull, ClientPullPreFetch");
      ros::shutdown();
    }
    MyClient.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up); // 'Z-up'
    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    ROS_INFO_STREAM("Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
        << Adapt(_Output_GetAxisMapping.YAxis) << " Z-" << Adapt(_Output_GetAxisMapping.ZAxis));
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    ROS_INFO_STREAM("Version: " << _Output_GetVersion.Major << "." << _Output_GetVersion.Minor << "."
        << _Output_GetVersion.Point);
    return true;
  }

  void updateCallback(const ros::TimerEvent& e)
  {
    static bool got_first = false;
    if (MyClient.GetFrame().Result == Result::Success)
    {
      now_time = ros::Time::now(); // try to grab as close to getting message as possible
      freq_status.tick();
      if (got_first)
      {
        max_period_between_updates = max(max_period_between_updates, (e.current_real - e.last_real).toSec());
        last_callback_duration = e.profile.last_duration.toSec();
      }
      process_frame();
      got_first = true;
    }
    diag_updater.update();
  }

  bool shutdown_vicon()
  {
    ROS_INFO_STREAM("Disconnecting from Vicon DataStream SDK");
    MyClient.Disconnect();
    ROS_ASSERT(!MyClient.IsConnected().Connected);
    ROS_INFO_STREAM("... disconnected.");
    return true;
  }

  bool process_frame()
  {
    static ros::Time lastTime;
    Output_GetFrameNumber OutputFrameNum = MyClient.GetFrameNumber();
    //frameCount++;
    //ROS_INFO_STREAM("Grabbed a frame: " << OutputFrameNum.FrameNumber);
    int frameDiff = 0;
    if (lastFrameNumber != 0)
    {
      frameDiff = OutputFrameNum.FrameNumber - lastFrameNumber;
      frameCount += frameDiff;
      if ((frameDiff) > 1)
      {
        droppedFrameCount += frameDiff;
        double droppedFramePct = (double)droppedFrameCount / frameCount * 100;
        ROS_DEBUG_STREAM(frameDiff << " more (total " << droppedFrameCount << "/" << frameCount << ", "
            << droppedFramePct << "%) frame(s) dropped. Consider adjusting rates.");
      }
    }
    double latencyInMs = MyClient.GetLatencyTotal().Total * 1000;
    // We know the subject and segment names a priori, so don't bother enumerating, just grab the data:
    // Flyer:
    Output_GetSegmentGlobalTranslation trans = MyClient.GetSegmentGlobalTranslation(SubjectName, SegmentName);
    Output_GetSegmentGlobalRotationQuaternion quat = MyClient.GetSegmentGlobalRotationQuaternion(SubjectName,
                                                                                                 SegmentName);
    if ((!trans.Occluded) && (!quat.Occluded))
    {
      flyer_transform.setOrigin(tf::Vector3(trans.Translation[0] / 1000, trans.Translation[1] / 1000,
                                            trans.Translation[2] / 1000));
      flyer_transform.setRotation(
                                  tf::Quaternion(quat.Rotation[0], quat.Rotation[1], quat.Rotation[2], quat.Rotation[3]));
      ros::Time thisTime = now_time - ros::Duration(latencyInMs / 1000);
      tf::StampedTransform transform = tf::StampedTransform(flyer_transform, thisTime, tf_ref_frame_id,
                                                            tf_tracked_frame_id);
      tf_broadcast.sendTransform(transform);

      geometry_msgs::TransformStamped pose_msg;
      tf::transformStampedTFToMsg(transform, pose_msg);
      double dt_from_clock = (thisTime - lastTime).toSec();
      double dt_from_framediff = (1 / vicon_capture_rate) * frameDiff; // how much time there *should* be between the last frame and this one
      if (fabs(1 - dt_from_clock / dt_from_framediff) < 0.5)
      {
        pose_pub.publish(pose_msg);
      }
      else
      {
        ROS_WARN_STREAM("Did not publish: dt_from_clock=" << dt_from_clock << " dt_from_framediff=" << dt_from_framediff);
      }
      lastTime = pose_msg.header.stamp;
    }

    lastFrameNumber = OutputFrameNum.FrameNumber;
    return true;
  }

  bool grab_vicon_pose_callback(vicon_mocap::viconGrabPose::Request& req, vicon_mocap::viconGrabPose::Response& resp)
  {
    ROS_INFO("Got request for a VICON pose");
    tf::StampedTransform transform;
    //tf::Quaternion quat;

    // Gather data:
    int N = req.n_measurements;
    double accum_trans[3] = {0, 0, 0};
    double accum_quat[4] = {0, 0, 0, 0};
    for (int k = 0; k < N; k++)
    {
      try
      {
        Output_GetSegmentGlobalTranslation trans = MyClient.GetSegmentGlobalTranslation(req.subject_name,
                                                                                        req.segment_name);
        Output_GetSegmentGlobalRotationQuaternion quat = MyClient.GetSegmentGlobalRotationQuaternion(req.subject_name,
                                                                                                     req.segment_name);
        if ((!trans.Occluded) && (!quat.Occluded))
        {
          accum_trans[0] += trans.Translation[0] / 1000;
          accum_trans[1] += trans.Translation[1] / 1000;
          accum_trans[2] += trans.Translation[2] / 1000;
          accum_quat[0] += quat.Rotation[3];
          accum_quat[1] += quat.Rotation[0];
          accum_quat[2] += quat.Rotation[1];
          accum_quat[3] += quat.Rotation[2];
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        resp.success = false;
        return false; // TODO: should we really bail here, or just try again?
      }
    }

    // Average the data:
    double normalized_quat[4];
    double quat_norm = sqrt(accum_quat[0] * accum_quat[0] + accum_quat[1] * accum_quat[1] + accum_quat[2]
        * accum_quat[2] + accum_quat[3] * accum_quat[3]);
    for (int i = 0; i < 4; i++)
    {
      normalized_quat[i] = accum_quat[i] / quat_norm;
    }
    double normalized_vector[3];
    // Copy to inputs:
    for (int i = 0; i < 3; i++)
    {
      normalized_vector[i] = accum_trans[i] / N;
    }

    // copy what we used to service call response:
    resp.success = true;
    resp.pose.header.stamp = ros::Time::now();
    resp.pose.pose.position.x = normalized_vector[0];
    resp.pose.pose.position.y = normalized_vector[1];
    resp.pose.pose.position.z = normalized_vector[2];
    resp.pose.pose.orientation.w = normalized_quat[0];
    resp.pose.pose.orientation.x = normalized_quat[1];
    resp.pose.pose.orientation.y = normalized_quat[2];
    resp.pose.pose.orientation.z = normalized_quat[3];
    return true;

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon_recv_direct");
  ViconReceiver vr;
  ros::spin();
  return 0;
}
