#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_cookbook");
  ros::NodeHandle node;

  // To publish and read the topics regarding frame transformation
  tf::TransformListener tf_ls;
  tf::TransformBroadcaster tf_br;

  // Trans formation object for robot, the camera on board of the robot,
  // the external camera
  tf::Transform map_to_odom_tf;
  tf::Transform odom_to_robot_tf;
  tf::Transform robot_to_camera_tf;
  tf::Transform map_to_ext_camera_tf;

  // Set a displacement (translation + rotation) for odom
  map_to_odom_tf.setOrigin(tf::Vector3(0.3, 0.0, 0.0));
  map_to_odom_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  // Set a displacement (translation + rotation) for external camera
  map_to_ext_camera_tf.setOrigin(tf::Vector3(-1.0, 1.0, 2.0));
  map_to_ext_camera_tf.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  // Set a displacement (translation + rotation) for camera
  robot_to_camera_tf.setOrigin(tf::Vector3(0.15, 0.0, 0.30));
  robot_to_camera_tf.setRotation(tf::Quaternion(0.5, -0.5, 0.5, -0.5));

  // Robot start position, it will then follow a circular trajectory
  float R = 1.0;
  float x = 0.0, y = 0.0, phi = 0.0;

  // Orientation for robot trajectory
  tf::Quaternion robot_ori;

  // main control cycle with a defined frequency
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    // set robot's pose to follow a circular trajectory
    x = R * std::cos(phi);
    y = R * std::sin(phi);
    phi += 0.01;
    robot_ori.setRPY(0.0, 0.0, phi + M_PI / 2.0);

    // robot's relative pose
    odom_to_robot_tf.setOrigin(tf::Vector3(x, y, 0.0));
    odom_to_robot_tf.setRotation(robot_ori);

    ros::Time stamp = ros::Time::now();

    // parent-child relation between the frame attated to the considered entities
    // are defined
    tf_br.sendTransform(tf::StampedTransform(map_to_odom_tf, stamp, "/map", "/odom"));
    tf_br.sendTransform(tf::StampedTransform(odom_to_robot_tf, stamp, "/odom", "/robot"));
    tf_br.sendTransform(
        tf::StampedTransform(robot_to_camera_tf, stamp, "/robot", "/camera"));
    tf_br.sendTransform(
        tf::StampedTransform(map_to_ext_camera_tf, stamp, "/map", "/ext_camera"));

    try {
      // computing the homogeneous transformation from camera frame in robot to the
      // external camera frame
      tf::Pose pose_in_camera;
      pose_in_camera.setOrigin(tf::Vector3(1.0, 1.0, 1.0));
      pose_in_camera.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

      tf::StampedTransform tf_ex_cam_to_robot_cam;
      tf_ls.lookupTransform("camera", "ext_camera", ros::Time(0), tf_ex_cam_to_robot_cam);
      tf::Pose pose_in_ext_camera = tf_ex_cam_to_robot_cam * pose_in_camera;

      ROS_INFO(
          "Pose in /ext_camera - position: %3.2f, %3.2f, %3.2f - orientation: %3.2f, "
          "%3.2f, %3.2f",
          pose_in_ext_camera.getOrigin().x(), pose_in_ext_camera.getOrigin().y(),
          pose_in_ext_camera.getOrigin().z(), pose_in_ext_camera.getRotation().x(),
          pose_in_ext_camera.getRotation().y(), pose_in_ext_camera.getRotation().z());
    }
    catch (tf::TransformException& e) {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
