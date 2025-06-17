#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <filters/filter_chain.hpp>

using GridMapMsg = grid_map_msgs::msg::GridMap;
using namespace grid_map;

#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

inline void unpackRGB(float rgb_float, uint8_t &r, uint8_t &g, uint8_t &b) {
  uint32_t rgb;
  std::memcpy(&rgb, &rgb_float, sizeof(rgb));
  r = (rgb >> 16) & 0xFF;
  g = (rgb >>  8) & 0xFF;
  b = (rgb      ) & 0xFF;
}

class TimeSyncNode : public rclcpp::Node
{
  public:
  TimeSyncNode() : Node("sync_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_), filterChain_("grid_map::GridMap")
  {
    rclcpp::QoS qos = rclcpp::QoS(10);
   
    semantic_map_.subscribe(this, "/semantic_grid_map", qos.get_rmw_qos_profile());
    elevation_map_.subscribe(this, "/elevation_map_raw_post", qos.get_rmw_qos_profile());
    grid_map_pub_ = this->create_publisher<GridMapMsg>("/merged_grid_map", qos);

    this->declare_parameter("filter_chain_parameter_name", std::string("filters"));
    this->declare_parameter<std::string>("map_frame_id");
    this->declare_parameter<std::string>("robot_base_frame_id");

    this->get_parameter("filter_chain_parameter_name", filterChainParametersName_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("robot_base_frame_id", robot_base_frame_id_);

    uint32_t queue_size = 10;
    sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
      ApproximateTime<GridMapMsg, GridMapMsg>>>(
      message_filters::sync_policies::ApproximateTime<GridMapMsg,
      GridMapMsg>(queue_size), semantic_map_, elevation_map_);

    sync->setAgePenalty(0.50);
    sync->registerCallback(std::bind(&TimeSyncNode::SyncCallback, this, _1, _2));

    class_names_ = {"bush",          "dirt",       "fence",    "grass",
                    "gravel",        "log",        "mud",      "object",
                    "other-terrain", "rock",       "sky",      "structure",
                    "tree-foliage",  "tree-trunk", "water",    "unlabeled",
                    "unlabeled",     "unlabeled",  "unlabeled"};

    color_to_class_ = {
        {{34, 139, 34}, "bush"},          {{139, 69, 19}, "dirt"},
        {{255, 215, 0}, "fence"},         {{124, 252, 0}, "grass"},
        {{169, 169, 169}, "gravel"},      {{160, 82, 45}, "log"},
        {{101, 67, 33}, "mud"},           {{255, 0, 255}, "object"},
        {{128, 128, 0}, "other-terrain"}, {{112, 128, 144}, "rock"},
        {{135, 206, 235}, "sky"},         {{178, 34, 34}, "structure"},
        {{0, 100, 0}, "tree-foliage"},    {{139, 115, 85}, "tree-trunk"},
        {{0, 191, 255}, "water"},         {{0, 0, 0}, "unlabeled"},
        {{0, 0, 0}, "unlabeled"},         {{0, 0, 0}, "unlabeled"},
        {{0, 0, 0}, "unlabeled"}};

    for (const auto &pair : color_to_class_) {
      const auto &rgb_tuple = pair.first;
      class_to_color_[pair.second] = {std::get<0>(rgb_tuple),
                                      std::get<1>(rgb_tuple),
                                      std::get<2>(rgb_tuple)};
    }

    // Setup filter chain.
    if (filterChain_.configure(
        filterChainParametersName_, this->get_node_logging_interface(),
        this->get_node_parameters_interface()))
    {
      RCLCPP_INFO(this->get_logger(), "Filter chain configured.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain!");
      rclcpp::shutdown();
      return;
    }

    map_pointer_set_ = false;
  }

  private:

    message_filters::Subscriber<GridMapMsg> semantic_map_;
    message_filters::Subscriber<GridMapMsg> elevation_map_;

    rclcpp::Publisher<GridMapMsg>::SharedPtr grid_map_pub_;


    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<GridMapMsg, GridMapMsg>>> sync;

    std::vector<std::string> class_names_;
    std::map<std::tuple<uint8_t, uint8_t, uint8_t>, std::string> color_to_class_;
    std::map<std::string, std::array<uint8_t, 3>> class_to_color_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    filters::FilterChain<grid_map::GridMap> filterChain_;
    std::string filterChainParametersName_; 
    std::string map_frame_id_;
    std::string robot_base_frame_id_;

    grid_map::Matrix* ground_class_;
    grid_map::Matrix* obstacle_;
    grid_map::Matrix* sky_map_;
    grid_map::Matrix* elevation_;
    grid_map::Matrix* min_height_smooth_;

    bool map_pointer_set_;


  private:

    void SyncCallback(const GridMapMsg::ConstSharedPtr & semantic_map,
        const GridMapMsg::ConstSharedPtr & elevation_map)
    {
      // RCLCPP_INFO(this->get_logger(), "Sync callback with %u and %u as times", semantic_map->header.stamp.sec, elevation_map->header.stamp.sec);
      grid_map::GridMap semantic_grid_map;
      grid_map::GridMap elevation_grid_map;
      grid_map::GridMapRosConverter::fromMessage(*semantic_map, semantic_grid_map);
      grid_map::GridMapRosConverter::fromMessage(*elevation_map, elevation_grid_map);

      // Initialize map pointer
      ground_class_ = &semantic_grid_map["ground_class"];
      obstacle_ = &semantic_grid_map["obstacle"];
      sky_map_ = &semantic_grid_map["sky_map"];
      min_height_smooth_ = &semantic_grid_map["min_height_smooth"];
      elevation_ = &elevation_grid_map["elevation"];


      //===============================================================================================================
      // Update the position of both maps
      geometry_msgs::msg::TransformStamped robot_transform;
      try {
        robot_transform = tf_buffer_.lookupTransform(map_frame_id_, robot_base_frame_id_, tf2::TimePointZero);

        double robot_x = robot_transform.transform.translation.x;
        double robot_y = robot_transform.transform.translation.y;
        semantic_grid_map.move(grid_map::Position(robot_x, robot_y));
        elevation_grid_map.move(grid_map::Position(robot_x, robot_y));
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        return;
      }

      // Iterate over dominant_class layer of semantic map -> every cell which class grass -> set elevation_map to min_height of semantic map
      for (grid_map::GridMapIterator it(semantic_grid_map); !it.isPastEnd(); ++it) {
        grid_map::Index idx = *it;


        bool obstacle = (*obstacle_)(idx(0), idx(1)) == 1000;
        bool sky = (*sky_map_)(idx(0), idx(1)) == 1000;
        if(!std::isnan((*ground_class_)(idx(0), idx(1))))
        {
          uint8_t r, g, b;
          float rgb_float = (*ground_class_)(idx(0), idx(1));
          unpackRGB(rgb_float, r, g, b);
          std::tuple<uint8_t, uint8_t, uint8_t> color{r, g, b};
          std::string cls = color_to_class_[color];
          
          //==================================================
          // Set height to min height if ground class is no obstacle
          if ((cls == "grass" || cls == "dirt" || cls == "gravel" || cls == "mud") && !obstacle) 
          {
            (*elevation_)(idx(0), idx(1)) = std::min((*elevation_)(idx(0), idx(1)), (*min_height_smooth_)(idx(0), idx(1)));
          }
        }

        //==================================================
        // Set height to min height if sky is blocked but no obstacle
        if(sky && !obstacle)
        {
          (*elevation_)(idx(0), idx(1)) = std::min((*elevation_)(idx(0), idx(1)), (*min_height_smooth_)(idx(0), idx(1)));
        }

      }

      // Copy dominant_class layer from semantic map to merged map
      if(semantic_grid_map.exists("ground_class")) {
        elevation_grid_map.add("ground_class", semantic_grid_map.get("ground_class"));
      }

      // Apply filter chain.
      grid_map::GridMap min_height_filtered;
      if (!filterChain_.update(elevation_grid_map, min_height_filtered)) {
        RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
        return;
      }
      
      // Convert the modified elevation grid map back to a message
      GridMapMsg merged_map_msg;
      merged_map_msg = *grid_map::GridMapRosConverter::toMessage(min_height_filtered);
      
      // Set the header
      merged_map_msg.header.stamp = semantic_map->header.stamp;
      merged_map_msg.header.frame_id = semantic_map->header.frame_id;
  
      // Publish the merged grid map
      grid_map_pub_->publish(merged_map_msg);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimeSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}