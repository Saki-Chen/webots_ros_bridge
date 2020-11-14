#pragma once

#include <string>

#include <ros/ros.h>
#include <webots/Lidar.hpp>

// #include <webots/

namespace webots_ros_bridge
{
    using std::string;
    using webots::LidarPoint;
    class WebotsRosPubisher
    {
    public:
        void enableSimulationMode() const;
        void publishClock(uint64_t micro_seconds);
        void publishSteeringAngle(const double angle);
        void publishWheelEncoder(const double fl, const double fr, const double rl, const double rr);
        void publishPointCloud(const LidarPoint* points, const int size, const float max_range);

        WebotsRosPubisher(const WebotsRosPubisher &) = delete;
        WebotsRosPubisher &operator=(const WebotsRosPubisher &) = delete;

        static WebotsRosPubisher &getInstance();

    protected:
        WebotsRosPubisher();

    private:
        ros::NodeHandle _nh;
        ros::Time _cur_stamp;
    }; // class WebotsRosPublisher

} // namespace webots_ros_bridge