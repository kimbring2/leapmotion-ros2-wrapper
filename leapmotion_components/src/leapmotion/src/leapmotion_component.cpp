#include <sstream>
#include <type_traits>
#include <vector>

#include "leapmotion_component.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <chrono>
#include <thread>

using namespace std::this_thread;
using namespace std::chrono;
using namespace Leap;


LeapMotion::LeapMotion(const rclcpp::NodeOptions & options)
: Node("leapmotion_node", options),
  mMappingQos(1)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "      LeapMotion Component ");
  RCLCPP_INFO(get_logger(), "********************************");

  //controller.addListener(listener);
  // Parameters initialization
  //initParameters();

  // Init services
  initServices();
  initPublishers();
  startSensor();
}


LeapMotion::~LeapMotion()
{
  RCLCPP_DEBUG(get_logger(), "Destroying node");
}


void LeapMotion::initServices()
{
  RCLCPP_INFO(get_logger(), "*** SERVICES ***");
}


void LeapMotion::initPublishers()
{
  RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

  std::string marker_topic = "plane_marker";
  std::string plane_topic = "plane";
  mPubMarker = create_publisher<visualization_msgs::msg::Marker>(marker_topic, mMappingQos);
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubMarker->get_topic_name());

  std::string marker_array_topic = "plane_marker_array";
  mPubMarkerArray = create_publisher<visualization_msgs::msg::MarkerArray>(marker_array_topic, mMappingQos);
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubMarkerArray->get_topic_name());

  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  //timer_ = this->create_wall_timer(500ms, std::bind(&LeapMotion::timer_callback, this));
}


bool LeapMotion::startSensor()
{
  RCLCPP_INFO(get_logger(), "***** STARTING SENSOR *****");

  controller.addListener(listener);

  // Start grab thread
  mGrabThread = std::thread(&LeapMotion::threadFunc_leapGrab, this);
  mSensThread = std::thread(&LeapMotion::threadFunc_pubSensorsData, this);

  if (mGrabThread.joinable()) {
    mGrabThread.join();
  }

  if (mSensThread.joinable()) {
    mSensThread.join();
  }
}


void LeapMotion::threadFunc_leapGrab()
{
  RCLCPP_DEBUG(get_logger(), "Grab thread started");

  while (1) {
    //RCLCPP_INFO(get_logger(), "threadFunc_leapGrab() loop");
    ;
  }
}

/*
void LeapMotion::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}
*/


void LeapMotion::threadFunc_pubSensorsData()
{
  // RCLCPP_DEBUG_ONCE(get_logger(), "Sensors callback called");
  RCLCPP_DEBUG(get_logger(), "Sensors thread started");

  while (1) {
    int hand_number = listener.hands_vector.size();

    if (hand_number == 2) {
      RCLCPP_INFO(this->get_logger(), "hand_number: '%d'", hand_number);
      markerArrayMsgPtr hand_marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
      
      for (int i = 0; i < listener.hands_vector.size(); i++) {
        visualization_msgs::msg::Marker joint_marker;
        joint_marker.id = 0;

        int finger_number = listener.hands_vector[i].fingers.size();

        RCLCPP_INFO(this->get_logger(), "listener.hands_vector['%d'].id: '%d'", i, listener.hands_vector[i].id);
        RCLCPP_INFO(this->get_logger(), "finger_number: '%d'", finger_number);
        for (int j = 0; j < finger_number; j++) {
          RCLCPP_INFO(this->get_logger(), "listener.hands_vector['%d'].fingers['%d'].id: '%d'", 
                                          i, j, listener.hands_vector[i].fingers[j].id);
          
          int bone_number = listener.hands_vector[i].fingers[j].bones.size();
          RCLCPP_INFO(this->get_logger(), "bone_number: '%d'", bone_number);
          
          for (int k = 0; k < 4; k++) {
            RCLCPP_INFO(this->get_logger(), "k: '%d'", k);
            
            geometry_msgs::msg::Point point;
            point.x = listener.hands_vector[i].fingers[j].bones[k].prev_joint_x;
            point.y = listener.hands_vector[i].fingers[j].bones[k].prev_joint_y;
            point.z = listener.hands_vector[i].fingers[j].bones[k].prev_joint_z;
            joint_marker.points.push_back(point);

            point.x = listener.hands_vector[i].fingers[j].bones[k].next_joint_x;
            point.y = listener.hands_vector[i].fingers[j].bones[k].next_joint_y;
            point.z = listener.hands_vector[i].fingers[j].bones[k].next_joint_z;
            joint_marker.points.push_back(point);

            hand_marker_array->markers.push_back(joint_marker);
            joint_marker.id++;
          }
        }
      }

      mPubMarkerArray->publish(std::move(hand_marker_array));
      RCLCPP_INFO(this->get_logger(), "\n");
    }

    sleep_for(nanoseconds(1000000));

    /*
    for (int i = 0; i < listener.hands_count + 1; i++) {
      rclcpp::Time ts = get_clock()->now();

      //RCLCPP_INFO(get_logger(), "threadFunc_leapGrab() loop");
      visualization_msgs::msg::Marker pt_marker;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      static int hit_pt_id = 0;
      pt_marker.header.stamp = ts;

      // Set the marker action.  Options are ADD and DELETE
      pt_marker.action = visualization_msgs::msg::Marker::ADD;
      pt_marker.lifetime = rclcpp::Duration(0, 0);

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      pt_marker.ns = "plane_hit_points";
      pt_marker.id = hit_pt_id++;
      std::string mMapFrameId = "map";
      pt_marker.header.frame_id = mMapFrameId;

      // Set the marker type.
      pt_marker.type = visualization_msgs::msg::Marker::SPHERE;

      // Set the pose of the marker.
      // This is a full 6DOF pose relative to the frame/time specified in the header
      float X = 0.1;
      float Y = 0.2;
      float Z = 0.3;

      pt_marker.pose.position.x = X;
      pt_marker.pose.position.y = Y;
      pt_marker.pose.position.z = Z;
      pt_marker.pose.orientation.x = 0.0;
      pt_marker.pose.orientation.y = 0.0;
      pt_marker.pose.orientation.z = 0.0;
      pt_marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      pt_marker.scale.x = 0.025;
      pt_marker.scale.y = 0.025;
      pt_marker.scale.z = 0.025;

      // Set the color -- be sure to set alpha to something non-zero!
      pt_marker.color.r = 0.2f;
      pt_marker.color.g = 0.1f;
      pt_marker.color.b = 0.75f;
      pt_marker.color.a = 0.8;

      pt_marker_array->markers.push_back(pt_marker);

      sleep_for(nanoseconds(1000));
    }
    */

    // Publish the marker array
    //mPubMarkerArray->publish(std::move(pt_marker_array));
  }
}