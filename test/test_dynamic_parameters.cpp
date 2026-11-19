// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "rbs_navfn_planner/navfn_planner.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(NavfnTest, RenamedPluginLoadsFromRbsPackage)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  EXPECT_EQ(loader.getClassPackage("rbs_navfn_planner::NavfnPlanner"), "rbs_navfn_planner");
  EXPECT_NE(loader.createSharedInstance("rbs_navfn_planner::NavfnPlanner"), nullptr);
}

class OpenMapPlanner : public rbs_navfn_planner::NavfnPlanner
{
public:
  OpenMapPlanner(bool astar, bool final_approach)
  : map_(200, 200, 0.05, -5.0, -5.0, 0)
  {
    costmap_ = &map_;
    planner_ = std::make_unique<rbs_navfn_planner::NavFn>(200, 200);
    clock_ = std::make_shared<rclcpp::Clock>();
    global_frame_ = "map";
    tolerance_ = 0.0;
    allow_unknown_ = false;
    use_astar_ = astar;
    use_final_approach_orientation_ = final_approach;
  }

  void finish(const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & path)
  {
    smoothApproachToGoal(goal, path);
  }

  void setCost(double x, double y, unsigned char cost)
  {
    unsigned int mx, my;
    ASSERT_TRUE(map_.worldToMap(x, y, mx, my));
    map_.setCost(mx, my, cost);
  }

private:
  nav2_costmap_2d::Costmap2D map_;
};

TEST(NavfnTest, GoalShortcutPreservesPrefixGoalAndCostBoundary)
{
  nav_msgs::msg::Path original;
  original.header.frame_id = "map";
  for (double x : {-0.15, -0.10, -0.05, 0.0}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = original.header;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    original.poses.push_back(pose);
  }
  auto goal = original.poses.back();
  goal.pose.position.x = 0.001;
  goal.pose.position.y = 0.015;
  goal.pose.orientation.z = std::sin(0.6);
  goal.pose.orientation.w = std::cos(0.6);
  auto unchanged = original;
  unchanged.poses.push_back(goal);
  for (unsigned char cost : {0, 100, 253, 254, 255}) {
    OpenMapPlanner planner(true, false);
    // Adjacent to the old tail: higher inflation, inscribed, lethal and unknown
    // cells must all prevent this optional, conservatively bounded shortcut.
    planner.setCost(0.02, 0.06, cost);
    auto path = original;
    planner.finish(goal.pose, path);
    EXPECT_EQ(path.poses.back(), goal);
    EXPECT_EQ(path.poses.front(), original.poses.front());
    if (cost != 0) {
      EXPECT_EQ(path, unchanged);
    } else {
      EXPECT_NE(path, unchanged);
      EXPECT_EQ(path.poses[1], original.poses[1]);
      for (size_t i = 2; i + 1 < path.poses.size(); ++i) {
        const auto & a = path.poses[i - 1].pose.position;
        const auto & b = path.poses[i].pose.position;
        const auto & c = path.poses[i + 1].pose.position;
        EXPECT_LE(std::abs(std::remainder(std::atan2(c.y - b.y, c.x - b.x) -
          std::atan2(b.y - a.y, b.x - a.x), 2.0 * M_PI)), M_PI / 4.0);
      }
    }
  }
}

TEST(NavfnTest, TranslationHeadingsFollowPathAndPreserveGoalPolicy)
{
  for (bool astar : {false, true}) {
    for (bool final_approach : {false, true}) {
      OpenMapPlanner planner(astar, final_approach);
      for (double direction : {0.0, 1.5707963267948966, 3.141592653589793, -1.2}) {
        geometry_msgs::msg::PoseStamped start, goal;
        start.header.frame_id = goal.header.frame_id = "map";
        start.pose.orientation.w = 1.0;
        goal.pose.position.x = 2.0 * std::cos(direction);
        goal.pose.position.y = 2.0 * std::sin(direction);
        goal.pose.orientation.z = std::sin(0.6);
        goal.pose.orientation.w = std::cos(0.6);
        const auto path = planner.createPlan(start, goal, [] {return false;});
        ASSERT_GT(path.poses.size(), 2U);
        double last_tangent = 0.0;
        for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
          const auto & a = path.poses[i].pose;
          const auto & b = path.poses[i + 1].pose;
          const double dx = b.position.x - a.position.x;
          const double dy = b.position.y - a.position.y;
          if (std::hypot(dx, dy) < 1e-8) {continue;}
          last_tangent = std::atan2(dy, dx);
          const double yaw = 2.0 * std::atan2(a.orientation.z, a.orientation.w);
          ASSERT_LE(std::abs(std::remainder(yaw - last_tangent, 2.0 * M_PI)),
            M_PI / 4.0 + 1e-6);
          if (i + 2 < path.poses.size()) {
            const double end_yaw = 2.0 * std::atan2(b.orientation.z, b.orientation.w);
            ASSERT_LE(std::abs(std::remainder(end_yaw - last_tangent, 2.0 * M_PI)),
              M_PI / 4.0 + 1e-6);
          }
        }
        const auto & end = path.poses.back().pose;
        EXPECT_NEAR(end.position.x, goal.pose.position.x, 1e-9);
        EXPECT_NEAR(end.position.y, goal.pose.position.y, 1e-9);
        const double goal_yaw = 2.0 * std::atan2(end.orientation.z, end.orientation.w);
        EXPECT_NEAR(std::remainder(goal_yaw - (final_approach ? last_tangent : 1.2),
          2.0 * M_PI), 0.0, 1e-6);
      }
    }
  }
}

TEST(NavfnTest, testDynamicParameter)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("Navfntest");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto planner =
    std::make_unique<rbs_navfn_planner::NavfnPlanner>();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  planner->configure(node, "test", tf, costmap);
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.tolerance", 1.0),
      rclcpp::Parameter("test.use_astar", true),
      rclcpp::Parameter("test.allow_unknown", true),
      rclcpp::Parameter("test.use_final_approach_orientation", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.tolerance").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.use_astar").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.allow_unknown").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.use_final_approach_orientation").as_bool(), true);
}
