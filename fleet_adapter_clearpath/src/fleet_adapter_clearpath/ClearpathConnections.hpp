/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SRC__FLEET_ADAPTER__CLEARPATH__CLEARPATHCONNECTIONS_HPP
#define SRC__FLEET_ADAPTER__CLEARPATH__CLEARPATHCONNECTIONS_HPP

#include <clearpath_api/fleet_api.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/Time.hpp>

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <vector>
#include <unordered_map>
#include <optional>
#include <unordered_set>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

namespace fleet_adapter_clearpath {

//==============================================================================
class ClearpathConnections
  : public std::enable_shared_from_this<ClearpathConnections>
{
public:

  // Forward declare
  class ClearpathCommandHandle;

  /// Store coordinate transformation data
  struct Transformation
  {
    double rotation;
    double scale;
    Eigen::Vector2d translation;

    Transformation(
        double rotation_,
        double scale_,
        double trans_x_,
        double trans_y_)
        : rotation(rotation_),
          scale(scale_)
    {
      translation = Eigen::Vector2d{trans_x_, trans_y_};
    }
  };

  // Used to determine if a robot can be added. Once a robot is added,
  // the update methods in ClearpathCommandHandle are called to directly update
  // RMF
  struct RobotData
  {
    std::optional<Eigen::Vector3d> pose; // in RMF coordinates
    std::optional<double> battery_soc;

    RobotData()
    {
      pose = std::nullopt;
      battery_soc = std::nullopt;
    }
  };

  using ClearpathCommandHandlePtr = std::shared_ptr<ClearpathCommandHandle>;
  using Farsight = clearpath_api::FarsightUplink<clearpath_api::FleetApi>;
  using FarsightMsg = Farsight::Ros2MsgT;
  using FarsightSub = rclcpp::Subscription<FarsightMsg>::SharedPtr;
  using BatteryState = clearpath_fleet_api::BatteryState;
  using BatteryStateMsg = BatteryState::Ros2MsgT;
  using BatteryStateSub = rclcpp::Subscription<BatteryStateMsg>::SharedPtr;
  using StepSequence = clearpath_fleet_api::StepSequence;
  using Action = StepSequence::Ros2ActionT;

  /// Create a shared_ptr to ClearpathConnections
  static std::shared_ptr<ClearpathConnections> make(
      const rmf_fleet_adapter::agv::AdapterPtr &adapter);

private:
  ClearpathConnections();

  void farsight_cb(
    const std::string& robot_name,
    const FarsightMsg::SharedPtr msg);

  void battery_cb(
    const std::string& robot_name,
    const BatteryStateMsg::SharedPtr msg);

  const Eigen::Vector3d transform(
    const Transformation& transformation,
    const Eigen::Vector3d& pose) const;

  void add_robot(const std::string& robot_name);

  rmf_fleet_adapter::agv::AdapterPtr _adapter;
  std::shared_ptr<rmf_traffic::agv::VehicleTraits> _traits;
  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
  rmf_fleet_adapter::agv::FleetUpdateHandlePtr _fleet;

  std::string _fleet_name;
  // Map robot name to respective data structures
  std::unordered_map<std::string, YAML::Node> _robot_configs;
  // When the data field is nullopt, it means the robot has been added.
  std::unordered_map<std::string, std::optional<RobotData>> _robot_states;
  std::unordered_map<std::string, FarsightSub> _farsight_subs;
  std::unordered_map<std::string, BatteryStateSub> _battery_subs;
  std::unordered_map<
    std::string, ClearpathCommandHandlePtr> _command_handles;

  std::shared_ptr<Transformation> _rmf_to_robot_transformation;
  std::shared_ptr<Transformation> _robot_to_rmf_transformation;
};


//==============================================================================
class ClearpathConnections::ClearpathCommandHandle
: public rmf_fleet_adapter::agv::RobotCommandHandle,
  public std::enable_shared_from_this<ClearpathConnections::ClearpathCommandHandle>
{
public:

  using RobotData = ClearpathConnections::RobotData;
  using Action = ClearpathConnections::StepSequence::Ros2ActionT;
  using GoalHandlePtr = rclcpp_action::ClientGoalHandle<Action>::SharedPtr;
  using Goal = rclcpp_action::Client<Action>::Goal;
  using FeedbackConstPtr = const std::shared_ptr<const Action::Feedback>;
  using Result = rclcpp_action::ClientGoalHandle<Action>::WrappedResult;
  using SendGoalOptions = rclcpp_action::Client<Action>::SendGoalOptions;
  using Transformer = std::function<Eigen::Vector3d(Eigen::Vector3d)>;

  enum class RobotState : uint8_t
  {
    Idle = 0,
    Waiting,
    Moving
  };

  struct PlanWaypoint
  {
    std::size_t index; // Index in follow_new_path
    Eigen::Vector3d position;
    rmf_traffic::Time time;
    std::optional<std::size_t> graph_index;
    std::vector<std::size_t> approach_lanes;

    PlanWaypoint(std::size_t index_, const rmf_traffic::agv::Plan::Waypoint& wp)
    : index(index_),
      position(wp.position()),
      time(wp.time()),
      graph_index(wp.graph_index()),
      approach_lanes(wp.approach_lanes())
    {
      // Do nothing
    }
  };

  /// Constructor
  ClearpathCommandHandle(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& name,
    std::shared_ptr<rmf_traffic::agv::Graph> graph,
    std::shared_ptr<rmf_traffic::agv::VehicleTraits> traits,
    rclcpp_action::Client<Action>::SharedPtr navigation_client,
    Transformer rmf_to_robot_transformer,
    const std::string& map_name,
    const rmf_traffic::agv::Planner::Start& start,
    const Eigen::Vector3d& initial_position,
    double initial_battery_soc,
    const std::size_t charger_waypoint,
    const bool filter_waypoints,
    const double filter_threshold = 0.7);

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) final;

  void stop() final;

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final;

  void set_updater(rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater);

  void update_position(const Eigen::Vector3d& position); // in RMF coordinates

  void update_battery_soc(double soc);

  ~ClearpathCommandHandle();

private:

  void parse_waypoints(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints);

  void start_follow();

  void navigate(const Eigen::Vector3d target); // target is in robot coordinates

  std::optional<std::size_t> get_current_lane();

  double dist(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

  std::shared_ptr<rclcpp::Node> _node;
  std::string _name;
  std::shared_ptr<rmf_traffic::agv::Graph> _graph;
  std::shared_ptr<rmf_traffic::agv::VehicleTraits> _traits;
  rclcpp_action::Client<Action>::SharedPtr _navigation_client;
  Transformer _rmf_to_robot_transformer;
  std::string _map_name;
  Eigen::Vector3d _position;
  double _battery_soc;
  std::size_t _charger_waypoint;
  bool _filter_waypoints;
  double _filter_threshold;
  rmf_fleet_adapter::agv::RobotUpdateHandlePtr _updater;
  bool _is_charger_set;
  RobotState _state;

  std::optional<std::size_t> _on_waypoint = std::nullopt;
  std::optional<std::size_t> _last_known_waypoint = std::nullopt;
  std::optional<std::size_t> _on_lane = std::nullopt;

  std::mutex _mutex;

  std::optional<PlanWaypoint> _target_waypoint;
  std::vector<PlanWaypoint> _remaining_waypoints;

  std::thread _follow_thread;
  std::atomic_bool _stop_follow_thread;
  std::atomic_bool _navigation_completed;
  GoalHandlePtr _goal_handle; // Result of std::future<GoalHandlePtr>::get();
  RequestCompleted _path_finished_callback;
  ArrivalEstimator _next_arrival_estimator;

};
} // namespace fleet_adapter_clearpath

#endif // SRC__FLEET_ADAPTER__CLEARPATH__CLEARPATHCONNECTIONS_HPP
