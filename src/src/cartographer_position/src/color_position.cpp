
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <limits>
#include <algorithm>

float gas_ppm = 0.0;  // 초기화된 가스 농도 값
double min_ppm = std::numeric_limits<double>::max();
double max_ppm = std::numeric_limits<double>::min();
visualization_msgs::MarkerArray marker_array;

void gasSensorCallback(const std_msgs::Int32::ConstPtr& msg) {
    gas_ppm = static_cast<float>(msg->data);
    ROS_INFO("Received gas sensor data: %d", msg->data);
}

void updateMinMaxPPM(float gas_ppm) {
    if (gas_ppm < min_ppm) {
        min_ppm = gas_ppm;
    }
    if (gas_ppm > max_ppm) {
        max_ppm = gas_ppm;
    }
}

void setMarkerColor(visualization_msgs::Marker& marker, float gas_ppm) {
    if (gas_ppm == max_ppm) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    } else if (gas_ppm == min_ppm) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    } else {
        float normalized_ppm = (gas_ppm - min_ppm) / (max_ppm - min_ppm);
        normalized_ppm = std::max(0.0f, std::min(1.0f, normalized_ppm));  // 0과 1 사이로 클램프

        marker.color.r = normalized_ppm;
        marker.color.g = 1.0f - normalized_ppm;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_marker");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    ros::Subscriber gas_sub = nh.subscribe<std_msgs::Int32>("analog_value", 1, gasSensorCallback);  // 가스 센서 데이터 구독

    tf::TransformListener listener;
    ros::Rate rate(0.5); // 2초마다 업데이트

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

            // 최댓값과 최솟값 업데이트
            updateMinMaxPPM(gas_ppm);

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

            // 마커 색상 설정
            setMarkerColor(marker, gas_ppm);

            marker_array.markers.push_back(marker);

            // 모든 마커의 색상 업데이트
            for (auto& existing_marker : marker_array.markers) {
                setMarkerColor(existing_marker, gas_ppm);  // gas_ppm 값을 사용하여 색상 업데이트
            }

            // 추가된 마커 배열 퍼블리시
            marker_pub.publish(marker_array);
            ROS_INFO("Published marker at x=%f, y=%f, z=%f with color (r=%f, g=%f, b=%f)",
                     x, y, z, marker.color.r, marker.color.g, marker.color.b);

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

