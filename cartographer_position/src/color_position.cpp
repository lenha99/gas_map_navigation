#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <fstream>
#include <sys/stat.h>
#include <limits>
#include <algorithm>

float gas_ppm = 0.0;  // 가스 센서 값 초기화
ros::Time last_publish_time;  // 마지막 발행 시간
std::ofstream data_file;  // 데이터를 저장할 파일 스트림

// 가스 센서 콜백 함수
void gasSensorCallback(const std_msgs::Int32::ConstPtr& msg) {
    gas_ppm = static_cast<float>(msg->data);
    ROS_INFO("Received gas sensor data: %d", msg->data);
}

// 디렉토리 존재 확인 함수
bool directoryExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        return false; // 접근할 수 없음
    }
    return (info.st_mode & S_IFDIR); // 디렉토리 여부 확인
}

// 마커의 색상과 타입을 설정하는 함수
void getColorAndType(float gas_ppm, std_msgs::ColorRGBA &color, int &marker_type) {
    int color_section = static_cast<int>(gas_ppm) / 100;
    float normalized_ppm = (gas_ppm - (color_section * 100)) / 100.0f;
    normalized_ppm = std::max(0.0f, std::min(1.0f, normalized_ppm));  // 0과 1 사이로 클램프

    // 색상 설정
    color.r = 1.0f - normalized_ppm; // 빨강에서 초록으로 변함
    color.g = normalized_ppm;        // 초록에서 빨강으로 변함
    color.b = 0.0f;
    color.a = 0.8f; // 투명도 조절

    // 마커 타입 설정
    if (gas_ppm < 200.0) {
        marker_type = visualization_msgs::Marker::ARROW; // 화살표
    } else if (gas_ppm < 300.0) {
        marker_type = visualization_msgs::Marker::ARROW; // 화살표
    } else if (gas_ppm < 400.0) {
        marker_type = visualization_msgs::Marker::CUBE; // 사각형
    } else if (gas_ppm < 500.0) {
        marker_type = visualization_msgs::Marker::CUBE; // 사각형
    } else if (gas_ppm < 600.0) {
        marker_type = visualization_msgs::Marker::SPHERE; // 구형
    } else if (gas_ppm < 700.0) {
        marker_type = visualization_msgs::Marker::SPHERE; // 구형
    } else if (gas_ppm < 800.0) {
        marker_type = visualization_msgs::Marker::CYLINDER; // 실린더형
    } else {
        marker_type = visualization_msgs::Marker::CYLINDER; // 실린더형
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_and_data_logger");
    ros::NodeHandle nh;

    // 가스 센서 데이터 구독
    ros::Subscriber gas_sub = nh.subscribe<std_msgs::Int32>("analog_value", 1, gasSensorCallback);

    // 마커 발행 설정
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    // TF 리스너 설정
    tf::TransformListener listener;

    // 데이터 디렉토리 경로 설정
    std::string directory_path = "/home/ubuntu/catkin_ws/data/";
    std::string file_path = directory_path + "robot_data.csv";

    // 디렉토리 존재 확인
    if (!directoryExists(directory_path)) {
        ROS_ERROR("Directory %s does not exist. Please create it first.", directory_path.c_str());
        return 1;
    }

    // CSV 파일 열기
    data_file.open(file_path);
    if (!data_file.is_open()) {
        ROS_ERROR("Failed to open file for writing. Path: %s", file_path.c_str());
        return 1;
    }

    // CSV 파일 헤더 작성
    data_file << "timestamp,x,y,gas_ppm\n";

    // 1Hz로 루프 실행 (1초마다 데이터를 기록하고 마커를 발행)
    ros::Rate rate(0.7);  // 1초에 한 번씩 실행
    last_publish_time = ros::Time::now();  // 마지막 발행 시간 초기화

    visualization_msgs::MarkerArray marker_array;

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        if ((current_time - last_publish_time).toSec() >= 2.0) {  // 2초마다 마커 발행 및 데이터 저장
            tf::StampedTransform transform;
            try {
                listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

                double x = transform.getOrigin().x();
                double y = transform.getOrigin().y();
                double z = transform.getOrigin().z();

                ROS_INFO("Current Position: x=%f, y=%f, z=%f, PPM=%f", x, y, z, gas_ppm);

                // 현재 시간을 타임스탬프로 저장
                double timestamp = current_time.toSec();

                // 좌표와 센서 데이터를 CSV 형식으로 파일에 저장
                data_file << timestamp << "," << x << "," << y << "," << gas_ppm << "\n";
                ROS_INFO("Saved data: timestamp=%f, x=%f, y=%f, gas_ppm=%f", timestamp, x, y, gas_ppm);

                // 마커 생성
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "current_position";
                marker.id = ros::Time::now().nsec;  // 고유한 ID 부여
                marker.pose.orientation.w = 1.0;
                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = z;

                // 마커 색상 및 타입 결정
                std_msgs::ColorRGBA color;
                int marker_type;
                getColorAndType(gas_ppm, color, marker_type);

                marker.type = marker_type;
                marker.scale.x = 0.1;  // 마커 크기 조절
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color = color;

                // 기존의 마커를 삭제하고 새로운 마커를 추가
                marker_array.markers.clear();
                marker_array.markers.push_back(marker);

                marker_pub.publish(marker_array);  // 마커 발행
                ROS_INFO("Published marker at x=%f, y=%f, z=%f with color (r=%f, g=%f, b=%f) and type=%d",
                         x, y, z, color.r, color.g, color.b, marker_type);

                last_publish_time = current_time;  // 마지막 발행 시간 갱신

            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    // 파일 닫기
    data_file.close();
    return 0;
}

