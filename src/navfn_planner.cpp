// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// #define BENCHMARK_TESTING

#include "rbs_navfn_planner/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rbs_navfn_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace rbs_navfn_planner
{

NavfnPlanner::NavfnPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

NavfnPlanner::~NavfnPlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type NavfnPlanner",
    name_.c_str());
}

void
NavfnPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  node_ = parent;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type NavfnPlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  declare_parameter_if_not_declared(node, name + ".use_astar", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_astar", use_astar_);
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

  declare_parameter_if_not_declared(
    node, name + ".enable_path_smoothing", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".enable_path_smoothing", enable_path_smoothing_);
  declare_parameter_if_not_declared(node, name + ".use_rdp", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_rdp", smoothing_options_.use_rdp);
  declare_parameter_if_not_declared(node, name + ".rdp_epsilon", rclcpp::ParameterValue(0.08));
  node->get_parameter(name + ".rdp_epsilon", smoothing_options_.rdp_epsilon);
  declare_parameter_if_not_declared(node, name + ".output_spacing", rclcpp::ParameterValue(0.04));
  node->get_parameter(name + ".output_spacing", smoothing_options_.output_spacing);
  declare_parameter_if_not_declared(
    node, name + ".maximum_deviation", rclcpp::ParameterValue(0.10));
  node->get_parameter(name + ".maximum_deviation", smoothing_options_.maximum_deviation);
  declare_parameter_if_not_declared(
    node, name + ".max_smoothing_time_ms", rclcpp::ParameterValue(100.0));
  node->get_parameter(name + ".max_smoothing_time_ms", smoothing_options_.max_smoothing_time_ms);
  if (!rbs_navfn_smoother::validSmoothingOptions(smoothing_options_)) {
    throw nav2_core::PlannerException("Invalid path smoothing parameters");
  }

  raw_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("raw_plan", 1);
  smoothed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("smoothed_plan", 1);

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<NavFn>(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());
}

void
NavfnPlanner::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
  raw_plan_pub_->on_activate();
  smoothed_plan_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&NavfnPlanner::dynamicParametersCallback, this, _1));
}

void
NavfnPlanner::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
  raw_plan_pub_->on_deactivate();
  smoothed_plan_pub_->on_deactivate();
}

void
NavfnPlanner::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type NavfnPlanner",
    name_.c_str());
  raw_plan_pub_.reset();
  smoothed_plan_pub_.reset();
  planner_.reset();
  costmap_ros_.reset();
}

nav_msgs::msg::Path NavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std::lock_guard<std::mutex> planner_lock(planner_mutex_);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
    *(costmap_->getMutex()));

#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();
#endif
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    throw nav2_core::StartOutsideMapBounds(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    throw nav2_core::GoalOutsideMapBounds(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }

  if (tolerance_ == 0 && costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw nav2_core::GoalOccupied(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was in lethal cost");
  }

  // Update planner based on the new costmap size
  if (isPlannerOutOfDate()) {
    planner_->setNavArr(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());
  }

  nav_msgs::msg::Path path;

  // Corner case of the start(x,y) = goal(x,y)
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
  } else {
    const auto original_start_cost = costmap_->getCost(mx_start, my_start);
    try {
      if (!makePlan(start.pose, goal.pose, tolerance_, cancel_checker, path)) {
        throw nav2_core::NoValidPathCouldBeFound(
                "Failed to create plan with tolerance of: " + std::to_string(tolerance_) );
      }
    } catch (...) {
      if (enable_path_smoothing_) {costmap_->setCost(mx_start, my_start, original_start_cost);}
      throw;
    }
    // Navfn clears this cell for search. Safety validation and later requests
    // must still see the observed obstacle, not the search's temporary override.
    if (enable_path_smoothing_) {costmap_->setCost(mx_start, my_start, original_start_cost);}
  }

  const auto publish_raw = [&]() {
      if (raw_plan_pub_) {raw_plan_pub_->publish(path);}
    };
  if (enable_path_smoothing_) {
    const auto smoothing_start = std::chrono::steady_clock::now();
    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Path smoothing cancelled");
    }
    if (!costmap_ros_) {
      throw nav2_core::NoValidPathCouldBeFound("Missing costmap footprint for smoothing");
    }
    nav2_msgs::msg::Costmap snapshot;
    snapshot.header = path.header;
    snapshot.metadata.size_x = costmap_->getSizeInCellsX();
    snapshot.metadata.size_y = costmap_->getSizeInCellsY();
    snapshot.metadata.resolution = costmap_->getResolution();
    snapshot.metadata.origin.position.x = costmap_->getOriginX();
    snapshot.metadata.origin.position.y = costmap_->getOriginY();
    snapshot.metadata.origin.orientation.w = 1.0;
    const size_t cell_count = static_cast<size_t>(snapshot.metadata.size_x) *
      snapshot.metadata.size_y;
    snapshot.data.assign(costmap_->getCharMap(), costmap_->getCharMap() + cell_count);
    const auto footprint = costmap_ros_->getRobotFootprint();
    costmap_lock.unlock();
    publish_raw();

    auto options = smoothing_options_;
    options.allow_unknown = allow_unknown_;
    options.use_final_approach_orientation = use_final_approach_orientation_;
    const auto process_start = std::chrono::steady_clock::now();
    auto result = rbs_navfn_smoother::processPath(
      path, snapshot, footprint, options, cancel_checker, &start.pose, smoothing_start);
    const auto finish = std::chrono::steady_clock::now();
    using Milliseconds = std::chrono::duration<double, std::milli>;
    RCLCPP_INFO(
      logger_, "RBS_NAVFN_SMOOTHING process_ms=%.3f total_ms=%.3f result=%s raw_poses=%zu output_poses=%zu",
      Milliseconds(finish - process_start).count(), Milliseconds(finish - smoothing_start).count(),
      result.reason.c_str(), path.poses.size(), result.path.poses.size());
    if (result.cancelled) {
      throw nav2_core::PlannerCancelled("Path smoothing cancelled");
    }
    if (!result.valid) {
      if (result.timed_out) {
        throw nav2_core::PlannerTimedOut("Path smoothing timed out without a validated path");
      }
      throw nav2_core::NoValidPathCouldBeFound("Path smoothing failed: " + result.reason);
    }
    path = std::move(result.path);
    if (smoothed_plan_pub_) {smoothed_plan_pub_->publish(path);}
  } else {
    costmap_lock.unlock();
    publish_raw();
  }


#ifdef BENCHMARK_TESTING
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  std::cout << "It took " << time_span.count() * 1000 << std::endl;
#endif

  return path;
}

bool
NavfnPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return true;
  }
  return false;
}

bool
NavfnPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(
    logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  worldToMap(wx, wy, mx, my);
  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  if (use_astar_) {
    planner_->calcNavFnAstar(cancel_checker);
  } else {
    planner_->calcNavFnDijkstra(cancel_checker, true);
  }

  double resolution = costmap_->getResolution();
  geometry_msgs::msg::Pose p, best_pose;

  bool found_legal = false;

  p = goal;
  double potential = getPointPotential(p.position);
  if (potential < POT_HIGH) {
    // Goal is reachable by itself
    best_pose = p;
    found_legal = true;
  } else {
    // Goal is not reachable. Trying to find nearest to the goal
    // reachable point within its tolerance region
    double best_sdist = std::numeric_limits<double>::max();

    p.position.y = goal.position.y - tolerance;
    while (p.position.y <= goal.position.y + tolerance) {
      p.position.x = goal.position.x - tolerance;
      while (p.position.x <= goal.position.x + tolerance) {
        potential = getPointPotential(p.position);
        double sdist = squared_distance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      smoothApproachToGoal(best_pose, plan);

      // Interior headings must describe both incoming and outgoing translation.
      // Bisect their wrapped angle so neither adjacent segment is favored.
      // Keep coincident samples aligned and leave the goal orientation policy below intact.
      auto orientation = start.orientation;
      double incoming = 0.0;
      bool has_incoming = false;
      for (size_t i = 0; i + 1 < plan.poses.size();) {
        size_t next = i + 1;
        while (next < plan.poses.size() &&
          squared_distance(plan.poses[i].pose, plan.poses[next].pose) < 1e-16)
        {
          ++next;
        }
        if (next < plan.poses.size()) {
          const auto & from = plan.poses[i].pose.position;
          const auto & to = plan.poses[next].pose.position;
          const double outgoing = std::atan2(to.y - from.y, to.x - from.x);
          const double heading = has_incoming ?
            incoming + 0.5 * std::remainder(outgoing - incoming, 2.0 * M_PI) : outgoing;
          orientation = nav2_util::geometry_utils::orientationAroundZAxis(heading);
          incoming = outgoing;
          has_incoming = true;
        }
        for (; i < std::min(next, plan.poses.size() - 1); ++i) {
          plan.poses[i].pose.orientation = orientation;
        }
      }

      // If use_final_approach_orientation=true, interpolate the last pose orientation from the
      // previous pose to set the orientation to the 'final approach' orientation of the robot so
      // it does not rotate.
      // And deal with corner case of plan of length 1
      if (use_final_approach_orientation_) {
        size_t plan_size = plan.poses.size();
        if (plan_size == 1) {
          plan.poses.back().pose.orientation = start.orientation;
        } else if (plan_size > 1) {
          double dx, dy, theta;
          auto last_pose = plan.poses.back().pose.position;
          auto approach_pose = plan.poses[plan_size - 2].pose.position;
          // Deal with the case of NavFn producing a path with two equal last poses
          if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
            std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
          {
            approach_pose = plan.poses[plan_size - 3].pose.position;
          }
          dx = last_pose.x - approach_pose.x;
          dy = last_pose.y - approach_pose.y;
          theta = atan2(dy, dx);
          plan.poses.back().pose.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(theta);
        }
      }
    } else {
      RCLCPP_ERROR(
        logger_,
        "Failed to create a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }

  return !plan.poses.empty();
}

void
NavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  goal_copy.header = plan.header;
  const size_t n = plan.poses.size();
  if (n >= 2 &&
    (squared_distance(plan.poses[n - 1].pose, plan.poses[n - 2].pose) >
    squared_distance(goal, plan.poses[n - 2].pose) ||
    squared_distance(plan.poses[n - 1].pose, goal) < 1e-6))
  {
    plan.poses.back() = goal_copy;
  } else {
    plan.poses.push_back(goal_copy);
  }

  // Repair only a short grid-to-exact-goal kink; ordinary corners stay unchanged.
  if (plan.poses.size() < 3) {return;}
  const auto tangent = [](const auto & a, const auto & b) {
      return std::atan2(b.position.y - a.position.y, b.position.x - a.position.x);
    };
  const size_t last = plan.poses.size() - 1;
  const double final_turn = std::remainder(
    tangent(plan.poses[last - 1].pose, goal) -
    tangent(plan.poses[last - 2].pose, plan.poses[last - 1].pose), 2.0 * M_PI);
  constexpr double max_join_angle = M_PI / 4.0 - 1e-6;
  if (std::abs(final_turn) <= max_join_angle) {return;}

  unsigned int gx, gy;
  if (!costmap_->worldToMap(goal.position.x, goal.position.y, gx, gy)) {return;}
  unsigned char original_max_cost = costmap_->getCost(gx, gy);
  const double resolution = costmap_->getResolution();
  double suffix_length = 0.0;
  for (size_t anchor = last; --anchor > 0;) {
    const auto & from = plan.poses[anchor].pose;
    suffix_length += std::sqrt(squared_distance(from, plan.poses[anchor + 1].pose));
    if (suffix_length > 4.0 * resolution) {break;}
    unsigned int mx, my;
    if (!costmap_->worldToMap(from.position.x, from.position.y, mx, my)) {return;}
    original_max_cost = std::max(original_max_cost, costmap_->getCost(mx, my));
    const auto & previous = plan.poses[anchor - 1].pose;
    const double distance = std::sqrt(squared_distance(from, goal));
    if (distance < 1e-4 || squared_distance(previous, from) < 1e-16 ||
      std::abs(std::remainder(tangent(from, goal) - tangent(previous, from),
      2.0 * M_PI)) > max_join_angle)
    {
      continue;
    }

    // Conservatively check the entire chord's cell rectangle, with one cell around
    // its boundaries. This covers every crossed cell, including diagonal corner
    // contacts. The inflated costmap carries Navfn's circular-body collision model.
    // Optional shortcuts never enter unknown space or a higher-cost region.
    const unsigned int low_x = std::min(mx, gx), low_y = std::min(my, gy);
    const unsigned int high_x = std::min(std::max(mx, gx) + 1, costmap_->getSizeInCellsX() - 1);
    const unsigned int high_y = std::min(std::max(my, gy) + 1, costmap_->getSizeInCellsY() - 1);
    bool safe = true;
    for (unsigned int y = low_y > 0 ? low_y - 1 : 0; y <= high_y && safe; ++y) {
      for (unsigned int x = low_x > 0 ? low_x - 1 : 0; x <= high_x; ++x) {
        const auto cost = costmap_->getCost(x, y);
        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost > original_max_cost) {
          safe = false;
          break;
        }
      }
    }
    if (!safe) {continue;}
    const auto start_position = from.position;
    const size_t steps = static_cast<size_t>(std::ceil(distance / (0.5 * resolution)));
    plan.poses.resize(anchor + 1);
    for (size_t step = 1; step < steps; ++step) {
      auto pose = goal_copy;
      const double t = static_cast<double>(step) / steps;
      pose.pose.position.x = start_position.x + t * (goal.position.x - start_position.x);
      pose.pose.position.y = start_position.y + t * (goal.position.y - start_position.y);
      plan.poses.push_back(pose);
    }
    plan.poses.push_back(goal_copy);
    return;
  }
}

bool
NavfnPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  worldToMap(wx, wy, mx, my);

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();
  RCLCPP_DEBUG(
    logger_,
    "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double
NavfnPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

bool
NavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    logger_,
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void
NavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void
NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

rcl_interfaces::msg::SetParametersResult
NavfnPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  auto smoothing = smoothing_options_;
  bool enabled = enable_path_smoothing_;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == name_ + ".enable_path_smoothing") {enabled = parameter.as_bool();}
    if (name == name_ + ".use_rdp") {smoothing.use_rdp = parameter.as_bool();}
    if (name == name_ + ".rdp_epsilon") {smoothing.rdp_epsilon = parameter.as_double();}
    if (name == name_ + ".output_spacing") {smoothing.output_spacing = parameter.as_double();}
    if (name == name_ + ".maximum_deviation") {smoothing.maximum_deviation = parameter.as_double();}
    if (name == name_ + ".max_smoothing_time_ms") {
      smoothing.max_smoothing_time_ms = parameter.as_double();
    }
  }
  if (!rbs_navfn_smoother::validSmoothingOptions(smoothing)) {
    result.successful = false;
    result.reason =
      "Smoothing spacing/time must be positive, epsilon/deviation nonnegative and finite";
    return result;
  }
  smoothing_options_ = smoothing;
  enable_path_smoothing_ = enabled;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".tolerance") {
        tolerance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_astar") {
        use_astar_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        allow_unknown_ = parameter.as_bool();
      } else if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace rbs_navfn_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rbs_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)
