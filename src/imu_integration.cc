#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "imu_integration.h"

bool loadIMUMsgFromRosBag(const std::string& path, std::vector<IMUState>& imu_info) {
    rosbag::Bag bag;
    try {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return false;
    }
    std::vector<std::string> types;
    types.push_back(std::string("sensor_msgs/Imu"));
    rosbag::View view(bag, rosbag::TypeQuery(types));
    // traverse messages
    for(const rosbag::MessageInstance& m : view) {
        sensor_msgs::Imu imu_msg = *(m.instantiate<sensor_msgs::Imu>());
        double time_stamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nsec / 1000000000.0;
        auto acc = Eigen::Vector3d(imu_msg.linear_acceleration.x,
                                   imu_msg.linear_acceleration.y,
                                   imu_msg.linear_acceleration.z);
        auto ang = Eigen::Vector3d(imu_msg.angular_velocity.x,
                                   imu_msg.angular_velocity.y,
                                   imu_msg.angular_velocity.z);
        imu_info.push_back(IMUState(time_stamp, ang, acc));
    }
    return false;
}

int main(int argc, char* argv[]) {
    std::string path(argv[1]);
    std::vector<IMUState> imu_info;
    loadIMUMsgFromRosBag(path, imu_info);
    IMUIntegration handler;
    for(const auto& imu : imu_info) {
        handler.Integrate(imu);
        std::cout << std::setprecision(10) << handler.getPos()(0) << ' '
                  << handler.getPos()(1) << ' '
                  << handler.getPos()(2) << '\n';
    }
    return 0;
}
