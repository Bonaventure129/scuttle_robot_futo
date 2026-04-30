#include "scuttle_mapping/mapping_with_known_poses.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// Using namespaces
using namespace std;
using namespace std::chrono_literals;
using namespace scuttle_mapping;
using namespace rclcpp;
using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;

namespace scuttle_mapping
{

double prob2logodds(double p)
{
    return log(p / (1 - p));
}

double logodds2prob(double l)
{
    return 1 - (1 / (1 + exp(l)));
}

unsigned int poseToCell(const Pose & pose, const MapMetaData & map_info)
{
    return map_info.width * pose.y + pose.x;
}

Pose coordinatesToPose(const double px, const double py, const MapMetaData & map_info)
{
    Pose pose;
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution);
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution);
    return pose;
}

bool poseOnMap(const Pose & pose, const MapMetaData & map_info)
{
    return pose.x < static_cast<int>(map_info.width)  && pose.x >= 0 &&
           pose.y < static_cast<int>(map_info.height) && pose.y >= 0;
}

vector<Pose> bresenham(const Pose & start, const Pose & end)
{
    vector<Pose> line;

    int dx = end.x - start.x;
    int dy = end.y - start.y;

    int xsign = dx > 0 ? 1 : -1;
    int ysign = dy > 0 ? 1 : -1;

    dx = abs(dx);
    dy = abs(dy);

    int xx, xy, yx, yy;
    if(dx > dy)
    {
        xx = xsign; xy = 0;
        yx = 0;     yy = ysign;
    }
    else
    {
        int tmp = dx;
        dx = dy;
        dy = tmp;
        xx = 0;     xy = ysign;
        yx = xsign; yy = 0;
    }

    int D = 2 * dy - dx;
    int y = 0;

    line.reserve(dx + 1);
    for (int i = 0; i < dx + 1; i++)
    {
        line.emplace_back(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy));
        if(D >= 0)
        {
            y++;
            D -= 2 * dx;
        }
        D += 2 * dy;
    }

    return line;
}

vector<pair<Pose, double>> inverseSensorModel(const Pose & p_robot, const Pose & p_beam)
{
    vector<pair<Pose, double>> occ_values;
    vector<Pose> line = bresenham(p_robot, p_beam);
    
    if (line.empty()) return occ_values;

    occ_values.reserve(line.size());

    // Free space along the ray
    for (size_t i = 0; i < line.size() - 1u; i++)
    {
        occ_values.emplace_back(make_pair(line.at(i), FREE_PROB)); 
    }

    // Occupied at the end
    occ_values.emplace_back(make_pair(line.back(), OCC_PROB));
    return occ_values;
}

MappingWithKnownPoses::MappingWithKnownPoses(const string &name)
    : Node(name)
{
    declare_parameter<double>("width", 50.0);
    declare_parameter<double>("height", 50.0);
    declare_parameter<double>("resolution", 0.1);

    double width = get_parameter("width").as_double();
    double height = get_parameter("height").as_double();
    map_.info.resolution = get_parameter("resolution").as_double();
    
    map_.info.width = round(width / map_.info.resolution);
    map_.info.height = round(height / map_.info.resolution);
    map_.info.origin.position.x = - round(width / 2.0);
    map_.info.origin.position.y = - round(height / 2.0);
    map_.header.frame_id = "odom";

    // Init map with prior probability (log odds)
    map_.data = vector<int8_t>(map_.info.height * map_.info.width, -1);
    probability_map_ = vector<double>(map_.info.height * map_.info.width, prob2logodds(PRIOR_PROB));

    // FIX: Use std::make_unique to avoid conflict with rclcpp::Node::make_unique
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // FIX: Use std::make_shared to avoid conflict with rclcpp::Node::make_shared
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    scan_sub_ = create_subscription<LaserScan>(
        "scan", 10, bind(&MappingWithKnownPoses::scanCallback, this, std::placeholders::_1));
        
    map_pub_ = create_publisher<OccupancyGrid>("map", 1);
    
    timer_ = create_wall_timer(2s, bind(&MappingWithKnownPoses::timerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Mapping Node Started (C++).");
}

void MappingWithKnownPoses::scanCallback(const LaserScan &scan)
{
    TransformStamped t;
    try
    {
        // Lookup transform from Map Frame (odom) to Sensor Frame
        t = tf_buffer_->lookupTransform(
            map_.header.frame_id, 
            scan.header.frame_id, 
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
            "Could not transform %s to %s: %s", 
            scan.header.frame_id.c_str(), 
            map_.header.frame_id.c_str(), 
            ex.what());
        return;
    }

    // Check if robot pose is on the map
    Pose robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, map_.info);
    if(!poseOnMap(robot_p, map_.info))
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot is out of map bounds!");
        return;
    }

    // Get Yaw
    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Raytracing
    // Optional: Add 'step' here (e.g. i+=5) if CPU usage is too high
    for (size_t i = 0; i < scan.ranges.size(); i++)
    {
      float r = scan.ranges.at(i);
      
      // Filter invalid ranges
      if (std::isinf(r) || std::isnan(r) || r < scan.range_min || r > scan.range_max)
          continue;

      // Polar to cartesian coordinates
      double angle = scan.angle_min + (i * scan.angle_increment) + yaw;
      double px = r * cos(angle) + t.transform.translation.x;
      double py = r * sin(angle) + t.transform.translation.y;

      Pose beam_p = coordinatesToPose(px, py, map_.info);
      
      // Skip if beam lands outside map
      if(!poseOnMap(beam_p, map_.info))
      {
        continue;
      }
      
      vector<pair<Pose, double>> poses = inverseSensorModel(robot_p, beam_p);

      for(const auto & pose : poses)
      {
        if(poseOnMap(pose.first, map_.info))
        {
            unsigned int cell = poseToCell(pose.first, map_.info);
            // Update Log Odds
            probability_map_.at(cell) += prob2logodds(pose.second) - prob2logodds(PRIOR_PROB);
        }
      }
    }
}

void MappingWithKnownPoses::timerCallback()
{
    // Convert Log Odds to occupancy grid (0-100) and publish
    // This runs efficiently every 2 seconds
    for (size_t i = 0; i < probability_map_.size(); ++i) {
        map_.data[i] = static_cast<int8_t>(logodds2prob(probability_map_[i]) * 100);
    }
    
    map_.header.stamp = get_clock()->now();
    map_pub_->publish(map_);
}

}  // namespace scuttle_mapping

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<scuttle_mapping::MappingWithKnownPoses>("mapping_with_known_poses");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}