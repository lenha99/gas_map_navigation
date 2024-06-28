#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>

float gas_ppm = 0.0;  // 초기화된 가스 농도 값

void gasSensorCallback(const std_msgs::Int32::ConstPtr& msg) {
  gas_ppm = static_cast<float>(msg->data);
  ROS_INFO("Received gas sensor data: %d", msg->data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "position_marker");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  ros::Subscriber gas_sub = nh.subscribe<std_msgs::Int32>("analog_value", 1, gasSensorCallback);  // 가스 센서 데이터 구독

  tf::TransformListener listener;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    tf::StampedTransform transform;
    try {
      // "map" 프레임이 올바르게 브로드캐스트되는지 확인
      if (!listener.frameExists("map")) {
        ROS_ERROR("Frame 'map' does not exist.");
        rate.sleep();
        continue;
      }

      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

      double x = transform.getOrigin().x();
      double y = transform.getOrigin().y();
      double z = transform.getOrigin().z();

      ROS_INFO("Current Position: x=%f, y=%f, z=%f, PPM=%f", x, y, z, gas_ppm);

      // 새로운 마커 생성
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "current_position";
      marker.id = ros::Time::now().nsec;  // 고유한 ID 부여
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;

      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;

      // 색상 보간
      float min_ppm = 300.0;
      float max_ppm = 800.0;
      float normalized_ppm = (gas_ppm - min_ppm) / (max_ppm - min_ppm);
      normalized_ppm = std::max(0.0f, std::min(1.0f, normalized_ppm));  // 0과 1 사이로 클램프

      marker.color.r = normalized_ppm; // 빨간색으로 변함
      marker.color.g = 1.0f - normalized_ppm; // 초록색으로 변함
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.push_back(marker);

      marker_pub.publish(marker_array);
      ROS_INFO("nomppm=%f,Published marker at x=%f, y=%f, z=%f with color (r=%f, g=%f, b=%f)", 
               normalized_ppm, x, y, z, marker.color.r, marker.color.g, marker.color.b);

    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
