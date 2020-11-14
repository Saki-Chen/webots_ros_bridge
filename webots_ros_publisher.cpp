#include "webots_ros_publisher.hpp"

#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <apa_msg/SteeringAngleStamped.h>
#include <apa_msg/WheelEncoderStamped.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace webots_ros_bridge
{   
    WebotsRosPubisher& WebotsRosPubisher::getInstance()
    {
        static WebotsRosPubisher instance;
        return instance;
    }

    void WebotsRosPubisher::enableSimulationMode() const
    {
        _nh.setParam("/use_sim_time", true);
    }

    void WebotsRosPubisher::publishClock(uint64_t micro_seconds)
    {
        static ros::Publisher pub = _nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

        _cur_stamp.fromNSec(micro_seconds * 1000);
        rosgraph_msgs::Clock clock;
        clock.clock = _cur_stamp;

        pub.publish(clock);
    }

    void WebotsRosPubisher::publishSteeringAngle(const double angle)
    {
        static ros::Publisher pub = _nh.advertise<apa_msg::SteeringAngleStamped>("front_wheel_steering_angle_rad", 1);

        apa_msg::SteeringAngleStamped angle_stamped;
        angle_stamped.header.frame_id = "chasis";
        angle_stamped.header.stamp = _cur_stamp;
        angle_stamped.data = angle;
        pub.publish(angle_stamped);
    }

    void WebotsRosPubisher::publishWheelEncoder(const double fl, const double fr, const double rl, const double rr)
    {
        static ros::Publisher pub = _nh.advertise<apa_msg::WheelEncoderStamped>("wheel_encoder_rad", 1);

        apa_msg::WheelEncoderStamped wheel_encoder;
        wheel_encoder.header.frame_id = "chasis";
        wheel_encoder.header.stamp = _cur_stamp;
        wheel_encoder.FL = fl;
        wheel_encoder.FR = fr;
        wheel_encoder.RL = rl;
        wheel_encoder.RR = rr;
        pub.publish(wheel_encoder);
    }

    WebotsRosPubisher::WebotsRosPubisher()
    {
    }

    void WebotsRosPubisher::publishPointCloud(const LidarPoint* points, const int size, const float max_range)
    {
        static ros::Publisher pub = _nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
        
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.header.frame_id = "rslidar";
        pcl_cloud.header.stamp = points[size-1].time * 1e6;
        pcl_cloud.height = 1;

        pcl_cloud.is_dense = true;
        int valid_count = 0;
        for(int i = 0; i < size; ++i)
        { 
            float x = points[i].x;
            float y = points[i].y;
            float z = points[i].z;
            if(x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl_cloud.points.emplace_back(z,x,y);  
            ++valid_count;
        }
        pcl_cloud.width = valid_count;
        
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(pcl_cloud, cloud_msg);
        pub.publish(cloud_msg);
    }

} // namespace webots_ros_publisher