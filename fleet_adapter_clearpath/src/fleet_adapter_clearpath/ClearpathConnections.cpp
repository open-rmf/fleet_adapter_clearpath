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

#include "ClearpathConnections.hpp"

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>

#include <rmf_task/requests/ChargeBatteryFactory.hpp>
#include <rmf_task/requests/ParkRobotFactory.hpp>

#include <rmf_task_msgs/msg/task_type.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <command_control_msgs/msg/step.hpp>

#include <command_control_msgs_utils/attribute_helper.h>

#include <functional>
#include <iostream>

namespace fleet_adapter_clearpath {

using namespace std::placeholders;
using namespace std::chrono_literals;

//==============================================================================
ClearpathConnections::ClearpathCommandHandle::ClearpathCommandHandle(
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
  const double filter_threshold)
: _node(std::move(node)),
  _name(std::move(name)),
  _graph(std::move(graph)),
  _traits(std::move(traits)),
  _navigation_client(std::move(navigation_client)),
  _rmf_to_robot_transformer(rmf_to_robot_transformer),
  _map_name(std::move(map_name)),
  _position(initial_position),
  _battery_soc(initial_battery_soc),
  _charger_waypoint(charger_waypoint),
  _filter_waypoints(filter_waypoints),
  _filter_threshold(filter_threshold)
{
  _updater = nullptr;
  _is_charger_set = false;
  _stop_follow_thread = false;

  if (start.lane().has_value())
    _on_lane = start.lane().value();
  else
  {
    _on_waypoint = start.waypoint();
    _last_known_waypoint = start.waypoint();
  }

  _goal_handle = nullptr;
}

//==============================================================================
ClearpathConnections::ClearpathCommandHandle::~ClearpathCommandHandle()
{
  if (_follow_thread.joinable())
  {
    _stop_follow_thread = true;
    _follow_thread.join();
  }
}

//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::follow_new_path(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  ArrivalEstimator next_arrival_estimator,
  RequestCompleted path_finished_callback)
{
  if (waypoints.empty() ||
    next_arrival_estimator == nullptr ||
    path_finished_callback == nullptr)
  {
    RCLCPP_WARN(
      _node->get_logger(),
      "Robot [%s] received follow_new_path request with invalid parameters. "
      " Ignoring...",
      _name.c_str());
    return;
  }

  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] received a new path to follow...",
    _name.c_str());
  std::cout << "Received new path with waypoints:" << std::endl;
  for (const auto& wp : waypoints)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "\t[%.2f,%.2f, %.2f]: %d",
      wp.position()[0], wp.position()[1], wp.position()[1], wp.time().time_since_epoch().count());
  }

  // Sending a new goal will preempt the previous goal. So we do not cancel
  // the previous goal
  // stop();
  if (_follow_thread.joinable())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "[stop] _follow_thread present. Calling join");
    _stop_follow_thread = true;
    _follow_thread.join();
  }

  _target_waypoint = std::nullopt;
  _remaining_waypoints.clear();
  _state = RobotState::Idle;

  parse_waypoints(waypoints);
  RCLCPP_DEBUG(
    _node->get_logger(),
    "_remaining_waypoints: [%d]. _target_waypoint: [%d]",
    _remaining_waypoints.size(), _target_waypoint.has_value());
  for (const auto& wp : _remaining_waypoints)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "[%.2f,%.2f, %.2f]: %d",
      wp.position[0], wp.position[1], wp.position[1], wp.time.time_since_epoch().count());
  }

  _next_arrival_estimator = std::move(next_arrival_estimator);
  _path_finished_callback = std::move(path_finished_callback);

  // Robot needs to wait
  if (_target_waypoint.has_value())
  {
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[follow_new_path] Target waypoint has a value. Robot [%s] will wait...",
      _name.c_str());
    _state = RobotState::Waiting;
    _on_waypoint = _target_waypoint.value().graph_index;
  }

  _stop_follow_thread = false;
  _follow_thread = std::thread(
    [w = weak_from_this()]()
    {
      auto me = w.lock();
      if (!me)
        return;
      me->start_follow();
    });
}
//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::start_follow()
{
  while ((!_remaining_waypoints.empty() ||
    _state == RobotState::Moving ||
    _state == RobotState::Waiting) && !_stop_follow_thread)
  {
    if (_state == RobotState::Idle)
    {
      _target_waypoint = _remaining_waypoints[0];
      const auto& target_pose = _target_waypoint.value().position;
      const auto& transformed_pose = _rmf_to_robot_transformer(target_pose);
      RCLCPP_INFO(
        _node->get_logger(),
        "Requesting robot [%s] to navigate to RMF coordinates:[%.2f, %.2f, %.2f] "
        "Robot coordinates:[%.2f, %.2f, %.2f]",
        _name.c_str(),
        target_pose[0], target_pose[1], target_pose[2],
        transformed_pose[0], transformed_pose[1], transformed_pose[2]);
      navigate(transformed_pose);
      if (_goal_handle)
      {
        _remaining_waypoints.erase(_remaining_waypoints.begin());
        _state = RobotState::Moving;
      }
      else
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] failed to navigate. Retrying...",
          _name.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }
    else if (_state == RobotState::Moving)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (_navigation_completed)
      {
        RCLCPP_INFO(
          _node->get_logger(),
          "Robot [%s] has reached its target waypoint",
          _name.c_str());
        _state = RobotState::Waiting;
        if (_target_waypoint.has_value())
        {
          _on_waypoint = _target_waypoint.value().graph_index;
          _last_known_waypoint = _on_waypoint;
        }
      }
      else
      {
        _on_lane = get_current_lane();
        if (_on_lane.has_value())
        {
          std::lock_guard<std::mutex> lock(_mutex);
          _on_waypoint = std::nullopt;
        }
        else
        {
          // The robot may either be on the previous or target waypoint
          const auto& last_location =
            _graph->get_waypoint(_last_known_waypoint.value()).get_location();
          const Eigen::Vector3d last_pose =
            {last_location[0], last_location[1], 0.0};
          if (_target_waypoint.value().graph_index.has_value() &&
            dist(_position, _target_waypoint.value().position) < 0.5)
          {
            _on_waypoint = _target_waypoint.value().graph_index.value();
          }
          else if (_last_known_waypoint.has_value() &&
            dist(_position, last_pose) < 0.5)
          {
            _on_waypoint = _last_known_waypoint.value();
          }
          else
          {
            // The robot is probably off-grid
            std::lock_guard<std::mutex> lock(_mutex);
            _on_waypoint = std::nullopt;
            _on_lane = std::nullopt;
          }
        }

        // Update arrival estimate
        if (_target_waypoint.has_value())
        {
          // TODO(YV) Subscribe to topic with distance to target information
          const auto& target_position = _target_waypoint.value().position;
          const auto& now = std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(_node->get_clock()->now().nanoseconds()));
          const auto& trajectory = rmf_traffic::agv::Interpolate::positions(
            *_traits,
            now,
            {_position, target_position});
          const auto finish_time = trajectory.finish_time();
          if (finish_time)
          {
            _next_arrival_estimator(
              _target_waypoint.value().index,
              *finish_time - now);
          }
        }
      }
    }
    else if (_state == RobotState::Waiting)
    {
      RCLCPP_DEBUG(
        _node->get_logger(),
        "State: Waiting");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      const rmf_traffic::Time& now = std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(_node->get_clock()->now().nanoseconds()));

      if (_target_waypoint.has_value())
      {
        const auto& wait_until = _target_waypoint.value().time;
        if (wait_until < now)
        {
          // Robot can move to next waypoint
          _state = RobotState::Idle;
          RCLCPP_INFO(
            _node->get_logger(),
            "Robot [%s] is done waiting. Remaining waypoints: %d",
            _name.c_str(),
            _remaining_waypoints.size());
        }
        else
        {
          const auto& waiting_duration = wait_until - now;
          RCLCPP_DEBUG(
            _node->get_logger(),
            "Robot [%s] waiting  at target for [%.1f]seconds",
            _name.c_str(),
            waiting_duration.count() / 1e9);

          _next_arrival_estimator(
            _target_waypoint.value().index,
            waiting_duration);
        }
      }
      else
      {
        RCLCPP_ERROR(
          _node->get_logger(),
          "State: Waiting but _target_waypoint is nullopt");
      }
    }
    else
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "[%s][start_follow]. Invalid state [%d]. Report this bug.",
        _name.c_str(),
        _state);
      _state = RobotState::Idle;
    }
  }

  // The robot is done navigating through all the waypoints
  assert(_path_finished_callback);
  _path_finished_callback();
  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] has successfully completed navigating along requested path.",
    _name.c_str());
  _path_finished_callback = nullptr;
  _next_arrival_estimator = nullptr;
}

//==============================================================================
std::optional<std::size_t>
ClearpathConnections::ClearpathCommandHandle::get_current_lane()
{
  const auto projection = [](
    const Eigen::Vector2d& current_position,
    const Eigen::Vector2d& target_position,
    const Eigen::Vector2d& lane_entry,
    const Eigen::Vector2d& lane_exit) -> double
  {
    return (current_position - target_position).dot(lane_exit - lane_entry);
  };

  if (!_target_waypoint.has_value())
    return std::nullopt;
  const auto& approach_lanes = _target_waypoint.value().approach_lanes;
  // Empty approach lanes signifies the robot will rotate at the waypoint.
  // Here we rather update that the robot is at the waypoint rather than
  // approaching it.
  if (approach_lanes.empty())
    return std::nullopt;

  for (const auto& lane_index : approach_lanes)
  {
    const auto& lane = _graph->get_lane(lane_index);
    const auto& p0 =
      _graph->get_waypoint(lane.entry().waypoint_index()).get_location();
    const auto& p1 =
      _graph->get_waypoint(lane.exit().waypoint_index()).get_location();
    const auto& p = _position.block<2, 1>(0, 0);
    const bool before_lane = projection(p, p0, p0, p1) < 0.0;
    const bool after_lane = projection(p, p1, p0, p1) >= 0.0;
    if (!before_lane && !after_lane) // the robot is on this lane
        return lane_index;
  }

  return std::nullopt;
}

//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::navigate(
  const Eigen::Vector3d target)
{
  _navigation_completed = false;

  if (!_navigation_client->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(
      _node->get_logger(),
      "Navigation action server for robot [%s] not available after waiting",
      _name.c_str());
    return;
  }

  RCLCPP_INFO(
    _node->get_logger(),
    "Navigation action server for robot [%s] available.",
    _name.c_str());

  // Convert to robot coordinates
  geometry_msgs::msg::Pose pose;
  pose.position.x = target[0];
  pose.position.y = target[1];
  pose.position.z = 0.0;
  const double yaw = target[2];
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(0.0);
  double sp = sin(0.0);
  double cr = cos(0.0);
  double sr = sin(0.0);
  pose.orientation.w = cy * cp * cr + sy * sp * sr;
  pose.orientation.x = cy * cp * sr - sy * sp * cr;
  pose.orientation.y = sy * cp * sr + cy * sp * cr;
  pose.orientation.z = sy * cp * cr - cy * sp * sr;

  Goal goal = Goal();
  command_control_msgs::msg::Step move_step;
  move_step.type = "strategy_management/MoveStep";
  move_step.interruptible = true;
  command_control_msgs::utils::addAttribute(move_step, "target", pose);
  goal.steps.push_back(move_step);
  goal.job_id = std::to_string(_node->get_clock()->now().seconds());

  SendGoalOptions goal_options;
  goal_options.goal_response_callback =
    [me = shared_from_this()](std::shared_future<GoalHandlePtr> gh_future)
    {
      std::lock_guard<std::mutex> lock(me->_mutex);
      RCLCPP_INFO(
        me->_node->get_logger(),
        "Received goal response for robot [%s]",
        me->_name.c_str());
      me->_goal_handle = gh_future.get();
      if (!me->_goal_handle)
      {
        RCLCPP_ERROR(
          me->_node->get_logger(),
          "Navigation goal for robot [%s] was rejected by the server",
          me->_name.c_str());
      }
      else
      {
        RCLCPP_INFO(
          me->_node->get_logger(),
          "Valid goal handle received for robot [%s]",
          me->_name.c_str());
      }
    };
  goal_options.result_callback =
    [me = shared_from_this()](const Result& result)
    {
      auto num_unfinished = result.result->unfinished_steps.size();
      RCLCPP_INFO(
        me->_node->get_logger(),
        "Navigation goal result for robot [%s]: Code[%d] #finished_steps:[%d] "
        "#unfinished_steps:[%d]",
        me->_name.c_str(),
        result.code,
        result.result->finished_steps.size(),
        num_unfinished);

        if (num_unfinished == 0)
        {
          std::lock_guard<std::mutex> lock(me->_mutex);
          me->_navigation_completed = true;
          // me->_goal_handle = nullptr;
        }
    };

  // auto future = _navigation_client->async_send_goal(goal, goal_options);
  // _goal_handle = future.get();
  _navigation_client->async_send_goal(goal, goal_options);



  RCLCPP_INFO(
    _node->get_logger(),
    "Async goal for robot [%s] sent.",
    _name.c_str());

}

//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::parse_waypoints(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints)
{
  if (waypoints.empty())
    return;

  std::vector<PlanWaypoint> wps;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    wps.push_back(PlanWaypoint(i, waypoints[i]));

  // Fix backtracking
  auto last_position = _position;
  const auto&  first_position = wps[0].position;
  if (waypoints.size() > 2 &&
    dist(first_position, last_position) > _filter_threshold)
  {
    bool changed = false;
    std::size_t index = 0;
    while (!changed)
    {
      if (dist(wps[0].position, first_position) > 0.1)
      {
        changed = true;
        break;
      }
      wps[index].position = last_position;
      ++index;
    }
  }

  if (!_filter_waypoints)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _target_waypoint = std::nullopt;
    _remaining_waypoints = std::move(wps);
    return;
  }

  // Perform filtering
  _remaining_waypoints.clear();
  bool changed = false;
  std::size_t index = 0;
  while (!changed && index < wps.size())
  {
    const auto d = dist(last_position, wps[index].position);
    RCLCPP_DEBUG(
      _node->get_logger(),
      "parse_waypoints d:[%.3f], filter_threshold: [%.3f]",
      d, _filter_threshold);
    if ( d < _filter_threshold)
    {
      _target_waypoint = wps[index];
      last_position = wps[index].position;
    }
    else
    {
      break;
    }
    ++index;
  }

  while (index < wps.size())
  {
    const auto parent_index = index;
    auto wp = wps[index];
    if (dist(wp.position, last_position) >= _filter_threshold)
    {
      changed = false;
      while (!changed)
      {
        auto next_index = index + 1;
        if (next_index < wps.size())
        {
          if (dist(wps[next_index].position, wps[index].position) < _filter_threshold)
          {
            if (next_index == wps.size() - 1)
            {
              // append last waypoint
              changed = true;
              wp = wps[next_index];
              wp.approach_lanes = wps[parent_index].approach_lanes;
              _remaining_waypoints.push_back(wp);
            }
          }
          else
          {
            // append if next waypoint changes
            changed = true;
            wp = wps[index];
            wp.approach_lanes = wps[parent_index].approach_lanes;
            _remaining_waypoints.push_back(wp);
          }
        }
        else
        {
          // we add the current index to second
          changed = true;
          wp = wps[index];
          wp.approach_lanes = wps[parent_index].approach_lanes;
          _remaining_waypoints.push_back(wp);
        }
        last_position = wps[index].position;
        index = next_index;
      }
    }
    else
    {
      ++index;
    }
  }
}

//==============================================================================
double ClearpathConnections::ClearpathCommandHandle::dist(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return (a.block<2, 1>(0, 0) - b.block<2, 1>(0, 0)).norm();
}

//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::stop()
{
  // Lock mutex
  std::lock_guard<std::mutex> lock(_mutex);

  if (_goal_handle)
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Calling async_cancel_goal with _goal_handle");
    // const auto cancel_future =
    //   _navigation_client->async_cancel_goal(_goal_handle);
    // const auto cancel_response = cancel_future.get();
    _navigation_client->async_cancel_goal(_goal_handle);
  }

  // _path_finished_callback = nullptr;
  // _next_arrival_estimator = nullptr;

  RCLCPP_INFO(
    _node->get_logger(),
    "Robot [%s] has stopped.",
    _name.c_str());
}

//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::dock(
  const std::string& dock_name,
  RequestCompleted docking_finished_callback)
{
  RCLCPP_WARN(
    _node->get_logger(),
    "ClearpathCommandHandle::dock(~) is not implemented yet. Robot [%s] will "
    "not dock at [%s]. Calling docking_finished_callback() to unblock task.",
    _name.c_str(), dock_name.c_str());

  docking_finished_callback();
}

//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::set_updater(
  rmf_fleet_adapter::agv::RobotUpdateHandlePtr updater)
{
  _updater = std::move(updater);
}

//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::update_position(
  const Eigen::Vector3d& position)
{
  if (!_updater)
    return;
  std::lock_guard<std::mutex> lock(_mutex);
  _position = position;
  if (_on_waypoint.has_value())
  {
    const std::size_t& wp = _on_waypoint.value();
    const double& ori = _position[2];
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with waypoint[%d] and orientation [%.2f]",
      _name.c_str(), wp, ori);
    _updater->update_position(wp, ori);
  }
  else if (_on_lane.has_value())
  {
    // TODO(YV) perform these calculations inside get_current_lane()
    std::vector<std::size_t> lanes = {_on_lane.value()};
    const auto& forward_lane = _graph->get_lane(_on_lane.value());
    const auto& entry_index = forward_lane.entry().waypoint_index();
    const auto& exit_index = forward_lane.exit().waypoint_index();
    const auto reverse_lane = _graph->lane_from(exit_index, entry_index);
    if (reverse_lane)
      lanes.push_back(reverse_lane->index());
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and lane count [%d]",
      _name.c_str(), _position[0], _position[1], _position[2], lanes.size());
    _updater->update_position(_position, lanes);
  }
  else if(_target_waypoint.has_value() &&
    _target_waypoint.value().graph_index.has_value())
  {
    const auto& graph_index = _target_waypoint.value().graph_index.value();
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with position [%.2f, %.2f, %.2f] and target waypoint [%d]",
      _name.c_str(), _position[0], _position[1], _position[2], graph_index);
    _updater->update_position(_position, graph_index);
  }
  else
  {
    RCLCPP_DEBUG(
      _node->get_logger(),
      "[%s] Calling update with map_name [%s] and position [%.2f, %.2f, %.2f]",
      _name.c_str(), _map_name.c_str(), _position[0], _position[1], _position[2]);
    _updater->update_position(_map_name, _position);
  }
}
//==============================================================================
void ClearpathConnections::ClearpathCommandHandle::update_battery_soc(
  double soc)
{
  if (!_updater)
    return;
  std::lock_guard<std::mutex> lock(_mutex);
  _battery_soc = soc;
  if (!_is_charger_set)
  {
    _updater->set_charger_waypoint(_charger_waypoint);
    _updater->maximum_delay(60s);
    _is_charger_set = true;
  }
  RCLCPP_DEBUG(
    _node->get_logger(),
    "[%s] Calling update_battery_soc with [%.2f]",
    _name.c_str(), _battery_soc);
  _updater->update_battery_soc(_battery_soc);
}

//==============================================================================
std::shared_ptr<ClearpathConnections> ClearpathConnections::make(
  const rmf_fleet_adapter::agv::AdapterPtr& adapter)
{
  std::shared_ptr<ClearpathConnections> connections(new ClearpathConnections());
  connections->_adapter = adapter;
  const auto& node = adapter->node();

  const std::string nav_graph_param_name = "nav_graph_file";
  const std::string graph_file =
    node->declare_parameter(nav_graph_param_name, std::string());
  if (graph_file.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", nav_graph_param_name.c_str());

    return nullptr;
  }

  const std::string config_param_name = "config_file";
  const std::string config_file =
    node->declare_parameter(config_param_name, std::string());
  if (config_file.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Missing [%s] parameter", config_param_name.c_str());

    return nullptr;
  }

  // Read config file
  RCLCPP_INFO(
    node->get_logger(),
    "Parsing config_file: %s",
    config_file.c_str());
  const YAML::Node config = YAML::LoadFile(config_file);
  if (!config)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Unable to load [%s] file", config_file.c_str());

    return nullptr;
  }

  try
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Parsing nav_graph_file: %s",
      graph_file.c_str());
    connections->_graph =
      std::make_shared<rmf_traffic::agv::Graph>(
      rmf_fleet_adapter::agv::parse_graph(graph_file, *connections->_traits));
    const YAML::Node rmf_config = config["rmf_fleet"];
    connections->_fleet_name = rmf_config["name"].as<std::string>();
    if (connections->_fleet_name.empty())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Fleet name cannot be empty");

    return nullptr;
    }

    const double v_nom = rmf_config["limits"]["linear"][0].as<double>();
    const double a_nom = rmf_config["limits"]["linear"][1].as<double>();
    const double w_nom = rmf_config["limits"]["angular"][0].as<double>();
    const double b_nom = rmf_config["limits"]["angular"][1].as<double>();
    const double r_f = rmf_config["profile"]["footprint"].as<double>();
    const double r_v = rmf_config["profile"]["vicinity"].as<double>();
    const bool reversible = rmf_config["reversible"].as<bool>();
    const double voltage =
      rmf_config["battery_system"]["voltage"].as<double>();
    const double capacity =
      rmf_config["battery_system"]["capacity"].as<double>();
    const double charging_current =
      rmf_config["battery_system"]["charging_current"].as<double>();
    const double mass = rmf_config["mechanical_system"]["mass"].as<double>();
    const double inertia =
      rmf_config["mechanical_system"]["moment_of_inertia"].as<double>();
    const double friction =
      rmf_config["mechanical_system"]["friction_coefficient"].as<double>();
    const double ambient_power =
      rmf_config["ambient_system"]["power"].as<double>();
    const double tool_power =
      rmf_config["tool_system"]["power"].as<double>();
    const double recharge_threshold =
      rmf_config["recharge_threshold"].as<double>();
    const double recharge_soc =
      rmf_config["recharge_soc"].as<double>();
    const bool publish_fleet_state =
      rmf_config["publish_fleet_state"].as<bool>();
    const bool drain_battery =
      rmf_config["account_for_battery_drain"].as<bool>();
    const bool perform_loop =
      rmf_config["task_capabilities"]["loop"].as<bool>();
    const bool perform_delivery =
      rmf_config["task_capabilities"]["delivery"].as<bool>();
    const bool perform_clean =
      rmf_config["task_capabilities"]["clean"].as<bool>();
    const std::string finishing_request_string =
      rmf_config["task_capabilities"]["finishing_request"].as<std::string>();
    connections->_rmf_to_robot_transformation =
      std::make_shared<ClearpathConnections::Transformation>(
        config["rmf_to_robot_transformation"]["rotation"].as<double>(),
        config["rmf_to_robot_transformation"]["scale"].as<double>(),
        config["rmf_to_robot_transformation"]["trans_x"].as<double>(),
        config["rmf_to_robot_transformation"]["trans_y"].as<double>());
    connections->_robot_to_rmf_transformation =
      std::make_shared<ClearpathConnections::Transformation>(
        config["robot_to_rmf_transformation"]["rotation"].as<double>(),
        config["robot_to_rmf_transformation"]["scale"].as<double>(),
        config["robot_to_rmf_transformation"]["trans_x"].as<double>(),
        config["robot_to_rmf_transformation"]["trans_y"].as<double>());

    const std::string server_uri_string =
      node->declare_parameter("server_uri", std::string());

    std::optional<std::string> server_uri = std::nullopt;
    if (!server_uri_string.empty())
      server_uri = server_uri_string;

    auto traits = rmf_traffic::agv::VehicleTraits{
      {v_nom, a_nom},
      {w_nom, b_nom},
      rmf_traffic::Profile{
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(r_f),
        rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(r_v)
      }
    };
    traits.get_differential()->set_reversible(reversible);

    connections->_traits =
      std::make_shared<rmf_traffic::agv::VehicleTraits>(traits);

    connections->_fleet = adapter->add_fleet(
      connections->_fleet_name,
      *connections->_traits,
      *connections->_graph,
      server_uri);

    if (!publish_fleet_state)
      connections->_fleet->fleet_state_publish_period(std::nullopt);

    const auto battery_opt = rmf_battery::agv::BatterySystem::make(
      voltage, capacity, charging_current);

    if (!battery_opt.has_value())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Invalid battery parameters");

      return nullptr;
    }
    const auto battery_system =
      std::make_shared<rmf_battery::agv::BatterySystem>(*battery_opt);

    const auto mechanical_opt = rmf_battery::agv::MechanicalSystem::make(
      mass, inertia, friction);
    if (!mechanical_opt.has_value())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Invalid mechanical parameters");

      return nullptr;
    }

    std::shared_ptr<rmf_battery::agv::SimpleMotionPowerSink> motion_sink =
      std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
      *battery_system, mechanical_opt.value());

    const auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
      ambient_power);
    if (!ambient_power_system)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Invalid values supplied for ambient power system");

      return nullptr;
    }
    std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> ambient_sink =
      std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
      *battery_system, *ambient_power_system);

    auto tool_power_system = rmf_battery::agv::PowerSystem::make(
      tool_power);
    if (!tool_power_system)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Invalid values supplied for tool power system");

      return nullptr;
    }
    std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> tool_sink =
      std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
      *battery_system, *tool_power_system);

    rmf_task::ConstRequestFactoryPtr finishing_request = nullptr;
    if (finishing_request_string == "charge")
    {
      finishing_request =
        std::make_shared<rmf_task::requests::ChargeBatteryFactory>();
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet is configured to perform ChargeBattery as finishing request");
    }
    else if (finishing_request_string == "park")
    {
      finishing_request =
        std::make_shared<rmf_task::requests::ParkRobotFactory>();
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet is configured to perform ParkRobot as finishing request");
    }
    else if (finishing_request_string == "nothing")
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet is not configured to perform any finishing request");
    }
    else
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Provided finishing request [%s] is unsupported. The valid "
        "finishing requests are [charge, park, nothing]. The task planner will "
        " default to [nothing].",
        finishing_request_string.c_str());
    }

    if (!connections->_fleet->set_task_planner_params(
        battery_system,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery,
        finishing_request))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Failed to initialize task planner parameters");

      return nullptr;
    }

    std::unordered_set<uint8_t> task_types;
    if (perform_loop)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] is configured to accept Loop tasks",
        connections->_fleet_name.c_str()
      );
      task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_LOOP);
    }
    if (perform_delivery)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] is configured to accept Delivery tasks",
        connections->_fleet_name.c_str()
      );
      task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_DELIVERY);
    }
    if (perform_clean)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "Fleet [%s] is configured to accept Clean tasks",
        connections->_fleet_name.c_str()
      );
      task_types.insert(rmf_task_msgs::msg::TaskType::TYPE_CLEAN);
    }

    connections->_fleet->accept_task_requests(
    [task_types](const rmf_task_msgs::msg::TaskProfile& msg)
    {
      if (task_types.find(msg.description.task_type.type) != task_types.end())
        return true;

      return false;
    });

  }

  catch(const std::exception& e)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Error while initializing fleet: [%s]", e.what());

    return nullptr;
  }

  // Parse robots
  const YAML::Node robots = config["robots"];
  if (!robots || !robots.IsMap())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "No robots in the config");

    return nullptr;
  }
  for (const auto& robot : robots)
  {
    const std::string& robot_name = robot.first.as<std::string>();
    connections->_robot_states.insert(
      {robot_name, RobotData()});
    connections->_robot_configs.insert({robot_name, robot.second});
    // Create interfaces for each robot
    const std::string& farsight_endpoint_name =
      clearpath_api::get_endpoint_name<Farsight>(robot_name);
    const auto farsight_sub = node->create_subscription<FarsightMsg>(
      farsight_endpoint_name,
      Farsight::qos_profile(),
      [robot_name, w = connections->weak_from_this()](
        const FarsightMsg::SharedPtr msg)
      {
        if (const auto self = w.lock())
          self->farsight_cb(robot_name, msg);
      });
    connections->_farsight_subs.insert({robot_name, farsight_sub});

    const std::string& battery_endpoint_name =
      clearpath_api::get_endpoint_name<BatteryState>(robot_name);
    const auto battery_sub = node->create_subscription<BatteryStateMsg>(
      battery_endpoint_name,
      BatteryState::qos_profile(),
      [robot_name, w = connections->weak_from_this()](
        const BatteryStateMsg::SharedPtr msg)
      {
        if (const auto self = w.lock())
          self->battery_cb(robot_name, msg);
      });
    connections->_battery_subs.insert({robot_name, battery_sub});
    // TODO other interfaces
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Successfully initialized fleet [%s].",
    connections->_fleet_name.c_str()
  );

  return connections;
}

//==============================================================================
void ClearpathConnections::farsight_cb(
  const std::string& robot_name,
  const FarsightMsg::SharedPtr msg)
{
  if (msg->objects.empty())
  {
    RCLCPP_WARN(
      _adapter->node()->get_logger(),
      "Received empty FarsightMsg for robot [%s].",
      robot_name.c_str()
    );
    return;
  }

  const auto pose = msg->objects[0].odom.pose;

  RCLCPP_DEBUG(
    _adapter->node()->get_logger(),
    "Received new position [%.2f, %.2f, %.2f] in robot [%s] coordinates",
    pose.x,
    pose.y,
    pose.yaw,
    robot_name.c_str()
  );

  const auto rmf_pose = transform(
      *_robot_to_rmf_transformation, {pose.x, pose.y, pose.yaw});
  const auto it = _robot_states.find(robot_name);
  if (it == _robot_states.end())
  {
    RCLCPP_ERROR(
      _adapter->node()->get_logger(),
      "Robot name missing in robot states map. This is a bug.",
      robot_name.c_str()
    );
    return;
  }

  RCLCPP_DEBUG(
    _adapter->node()->get_logger(),
    "Position [%.2f, %.2f, %.2f] for robot [%s] in RMF coordinates",
    rmf_pose[0],
    rmf_pose[1],
    rmf_pose[2],
    robot_name.c_str()
  );

  if (it->second.has_value())
  {
    it->second->pose = rmf_pose;
    add_robot(robot_name);
    return;
  }

  if (_command_handles.find(robot_name) != _command_handles.end())
    _command_handles[robot_name]->update_position(rmf_pose);

}

//==============================================================================
void ClearpathConnections::battery_cb(
  const std::string& robot_name,
  const BatteryStateMsg::SharedPtr msg)
{
  const auto it = _robot_states.find(robot_name);
  if (it == _robot_states.end())
  {
    RCLCPP_ERROR(
      _adapter->node()->get_logger(),
      "Robot name missing in robot states map. This is a bug.",
      robot_name.c_str()
    );
    return;
  }

  RCLCPP_DEBUG(
    _adapter->node()->get_logger(),
    "Received new battery state [%.2f] for robot [%s]",
    msg->percentage,
    robot_name.c_str()
  );

  // TODO(YV): Avoid this logic duplication
  if (it->second.has_value())
  {
    it->second->battery_soc = msg->percentage;
    add_robot(robot_name);
    return;
  }

  if (_command_handles.find(robot_name) != _command_handles.end())
  {
    _command_handles[robot_name]->update_battery_soc(msg->percentage);
  }
}

//==============================================================================
const Eigen::Vector3d ClearpathConnections::transform(
  const ClearpathConnections::Transformation& transformation,
  const Eigen::Vector3d& pose) const
{
  // const auto& scale = transformation.scale;
  // const auto& rotation = transformation.rotation;
  // const auto& dx = transformation.translation[0];
  // const auto& dy = transformation.translation[1];

  // return Eigen::Vector3d{
  //   (pose[0] * std::cos(rotation) * scale) - (pose[1] * std::sin(rotation) * scale) + dx,
  //   (pose[0] * std::sin(rotation) * scale) + (pose[1] * std::cos(rotation) * scale) + dy,
  //   pose[2] + rotation
  // };

  const auto& rotated =
    Eigen::Rotation2D<double>(transformation.rotation) *
    (transformation.scale * pose.block<2, 1>(0, 0));
  const auto& translated = rotated + transformation.translation;

  return Eigen::Vector3d{
    translated[0], translated[1], pose[2] + transformation.rotation};
}

//==============================================================================
void ClearpathConnections::add_robot(const std::string& robot_name)
{
  RCLCPP_ERROR(
    _adapter->node()->get_logger(),
    "Inside add_robot");

  const auto state_it = _robot_states.find(robot_name);
  if (state_it == _robot_states.end() || !state_it->second.has_value())
    return;

  const auto& pose = state_it->second.value().pose;
  const auto& battery_soc = state_it->second.value().battery_soc;
  const bool ready = pose.has_value() && battery_soc.has_value();

  if (!ready)
  {
    RCLCPP_INFO(
      _adapter->node()->get_logger(),
      "Robot [%s] is still missing some state parameters. Skipping add_robot(~)",
      robot_name.c_str());
    return;
  }

  // Create navigation action client for this robot
  const auto navigation_client = rclcpp_action::create_client<Action>(
    _adapter->node(),
    clearpath_api::get_endpoint_name<StepSequence>(robot_name)
  );

  // Create command handle for this robot
  const auto rmf_to_robot_transformer =
    [w = weak_from_this()](Eigen::Vector3d from) -> Eigen::Vector3d
    {
      auto me = w.lock();
      if (!me)
      {
        // TODO(YV): Fix
        return from;
      }
      return me->transform(*(me->_rmf_to_robot_transformation), from);
    };
  const auto config_it = _robot_configs.find(robot_name);
  assert(config_it != _robot_configs.end());
  const bool filter_waypoints =
    config_it->second["robot_config"]["filter_waypoints"].as<bool>();
  const std::string& map_name =
    config_it->second["rmf_config"]["start"]["map_name"].as<std::string>();
  const std::string& charger_wp_name =
     config_it->second["rmf_config"]["charger"]["waypoint"].as<std::string>();
  const auto charger_wp_ptr = _graph->find_waypoint(charger_wp_name);
  if (!charger_wp_ptr)
  {
    RCLCPP_ERROR(
      _adapter->node()->get_logger(),
      "Invalid charger name [%s] supplied for robot [%s]. Unable to add_robot",
      charger_wp_name.c_str(),
      robot_name.c_str());
    return;
  }
  const std::size_t& charger_wp_index = charger_wp_ptr->index();
  const auto& loc = pose.value();
  const rmf_traffic::Time& now = std::chrono::steady_clock::time_point(
    std::chrono::nanoseconds(_adapter->node()->now().nanoseconds()));
  const auto& starts = rmf_traffic::agv::compute_plan_starts(
    *_graph,
    map_name,
    loc,
    now,
    0.3);
  if (starts.empty())
  {

    RCLCPP_ERROR(
      _adapter->node()->get_logger(),
      "Unable to compute a StartSet for robot [%s] using level_name [%s] and "
      "location [%f, %f, %f] specified in its RobotState message. This can "
      "happen if the level_name in the RobotState message does not match any "
      "of the map names in the navigation graph supplied or if the location "
      "reported in the RobotState message is far way from the navigation "
      "graph. This robot will not be added to the fleet [%s].",
      robot_name.c_str(),
      map_name.c_str(),
      loc[0], loc[1], loc[2],
      _fleet_name.c_str());

    return;
  }

  const auto command = std::make_shared<ClearpathCommandHandle>(
    _adapter->node(),
    robot_name,
    _graph,
    _traits,
    navigation_client,
    rmf_to_robot_transformer,
    map_name,
    starts[0],
    loc,
    battery_soc.value(),
    charger_wp_index,
    filter_waypoints);

  _fleet->add_robot(
    command,
    robot_name,
    _traits->profile(),
    {starts[0]},
    [c = weak_from_this(), command, robot_name = std::move(robot_name)](
      const rmf_fleet_adapter::agv::RobotUpdateHandlePtr& updater)
    {
      const auto connections = c.lock();
      if (!connections)
        return;
      command->set_updater(updater);
      connections->_command_handles[robot_name] = command;
    });

  // Once a robot is added we update its state in Connections to nullptr
  // so that the callbacks will call the update() methods.
  _robot_states[robot_name] = std::nullopt;
}

//==============================================================================
ClearpathConnections::ClearpathConnections()
{
  // Do nothing
}

} // namespace fleet_adapter_clearpath
