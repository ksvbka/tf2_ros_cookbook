#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf2_cookbook");
  ros::NodeHandle node;

  // To publish and read the topics regarding frame transformation
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_ls(tf_buffer);
  tf2_ros::TransformBroadcaster tf_br;
  tf2_ros::StaticTransformBroadcaster tf_static_br;

  // Trans formation object for robot, the camera on board of the robot,
  // the external camera
  geometry_msgs::TransformStamped map_to_odom_tf;
  geometry_msgs::TransformStamped odom_to_robot_tf;
  geometry_msgs::TransformStamped robot_to_camera_tf;
  geometry_msgs::TransformStamped map_to_ext_camera_tf;

  // Static transform (prefer using URDF)
  map_to_ext_camera_tf.header.frame_id = "/map";
  map_to_ext_camera_tf.child_frame_id = "/ext_camera";
  map_to_ext_camera_tf.transform.translation = tf2::toMsg(tf2::Vector3(-1.0, 1.0, 2.0));
  map_to_ext_camera_tf.transform.rotation =
      tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  robot_to_camera_tf.header.frame_id = "/robot";
  robot_to_camera_tf.child_frame_id = "/camera";
  robot_to_camera_tf.transform.translation = tf2::toMsg(tf2::Vector3(0.15, 0.0, 0.3));
  robot_to_camera_tf.transform.rotation =
      tf2::toMsg(tf2::Quaternion(0.5, -0.5, 0.5, -0.5));

  auto stamp = ros::Time::now();
  map_to_ext_camera_tf.header.stamp = stamp;
  robot_to_camera_tf.header.stamp = stamp;

  tf_static_br.sendTransform(map_to_ext_camera_tf);
  tf_static_br.sendTransform(robot_to_camera_tf);

  // Robot start position, it will then follow a circular trajectory
  float R = 1.0;
  float x = 0.0, y = 0.0, phi = 0.0;

  // Orientation for robot trajectory
  tf2::Quaternion robot_ori;

  // main control cycle with a defined frequency
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    // set robot's pose to follow a circular trajectory
    x = R * std::cos(phi);
    y = R * std::sin(phi);
    phi += 0.01f;
    robot_ori.setRPY(0.0, 0.0, phi + M_PI / 2.0);

    // robot's relative pose
    odom_to_robot_tf.header.frame_id = "/odom";
    odom_to_robot_tf.child_frame_id = "/robot";
    odom_to_robot_tf.transform.translation = tf2::toMsg(tf2::Vector3(x, y, 0));
    odom_to_robot_tf.transform.rotation = tf2::toMsg(robot_ori);

    // Dynamic transformj map -> odom (maybe using slam package)
    map_to_odom_tf.header.frame_id = "/map";
    map_to_odom_tf.child_frame_id = "/odom";
    map_to_odom_tf.transform.translation = tf2::toMsg(tf2::Vector3(0.3, 0.0, 0.0));
    map_to_odom_tf.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    auto stamp = ros::Time::now();
    map_to_odom_tf.header.stamp = stamp;
    odom_to_robot_tf.header.stamp = stamp;

    // parent-child relation between the frame attated to the considered entities are
    // defined
    tf_br.sendTransform(map_to_odom_tf);
    tf_br.sendTransform(odom_to_robot_tf);

    try {
      // computing the homogeneous transformation from camera frame in robot to the
      // external camera frame
      geometry_msgs::Pose pose_in_camera;
      pose_in_camera.position.x = 1.0;
      pose_in_camera.position.y = 1.0;
      pose_in_camera.position.z = 1.0;
      pose_in_camera.orientation.w = 1.0;

      geometry_msgs::Pose pose_in_ext_camera;

      // Transfrom using TF utils
      auto tf_ex_cam_to_robot_cam =
          tf_buffer.lookupTransform("ext_camera", "camera", ros::Time(0));

      tf2::doTransform(pose_in_camera, pose_in_ext_camera, tf_ex_cam_to_robot_cam);

      // Verify by using matrix calculation
      tf2::Stamped<tf2::Transform> pose;
      pos[0] = pose_in_camera.position.x;
      pos[1] = pose_in_camera.position.y;
      pos[2] = pose_in_camera.position.z;

      auto& cam_ori = pose_in_camera.orientation;
      tf2::Quaternion rot(cam_ori.x, cam_ori.y, cam_ori.z, cam_ori.w);

      tf2::Stamped<tf2::Transform> tf;
      // tf2::fromMsg(tf_ex_cam_to_robot_cam, tf);
      // Or using
      tf2::convert(tf_ex_cam_to_robot_cam, tf);
      auto trans = tf * pos;
      auto orien = tf * rot;

      assert(trans[0] == pose_in_ext_camera.position.x);
      assert(trans[0] == pose_in_ext_camera.position.x);
      assert(trans[0] == pose_in_ext_camera.position.x);

      assert(orien.x() == pose_in_ext_camera.orientation.x);
      assert(orien.y() == pose_in_ext_camera.orientation.y);
      assert(orien.z() == pose_in_ext_camera.orientation.z);
      assert(orien.w() == pose_in_ext_camera.orientation.w);

      ROS_INFO(
          "Pose in /ext_camera - position: %3.2f, %3.2f, %3.2f - orientation: %3.2f, "
          "%3.2f, %3.2f",
          pose_in_ext_camera.position.x, pose_in_ext_camera.position.y,
          pose_in_ext_camera.position.z, pose_in_ext_camera.orientation.x,
          pose_in_ext_camera.orientation.y, pose_in_ext_camera.orientation.z);
    }
    catch (tf2::TransformException& e) {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}