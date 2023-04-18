/*
Created by clemens on 02.03.22.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <arti_planner_hierarchy/local_planner_hierarchy.h>
#include <arti_ros_param/arti_ros_param.h>
#include <arti_ros_param/collections.h>
#include <pluginlib/class_list_macros.h>

namespace arti_planner_hierarchy
{
LocalPlannerHierarchy::LocalPlannerHierarchy()
  : plugin_loader_("arti_nav_core", "arti_nav_core::BaseLocalPlanner")
{
}

void LocalPlannerHierarchy::initialize(
  std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros)
{
  ros::NodeHandle private_nh = ros::NodeHandle("~/" + name);

  std::vector<XmlRpc::XmlRpcValue> planners_raw = arti_ros_param::getRequiredParam<std::vector<XmlRpc::XmlRpcValue>>(
    private_nh, "planners");

  ROS_INFO_STREAM("init local planners");
  for (const auto& planner_info_raw: planners_raw)
  {
    std::string plugin_type = arti_ros_param::getRequiredParam<std::string>(planner_info_raw, "plugin_type");
    std::string plugin_name = name + "/" + arti_ros_param::getRequiredParam<std::string>(planner_info_raw, "plugin_name");

    ROS_INFO_STREAM("plugin_type: " << plugin_type << " plugin_name: " << plugin_name);

    try
    {
      plugins_.emplace_back(plugin_loader_.createUniqueInstance(plugin_type));
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      throw std::invalid_argument("failed to load the '" + plugin_type + "' plugin: " + ex.what());
    }

    if (!plugins_.back())
    {
      throw std::logic_error("got nullptr when loading the '" + plugin_type + "' plugin");
    }

    try
    {
      plugins_.back()->initialize(plugin_name, transformer, costmap_ros);
    }
    catch (const std::exception& ex)
    {
      throw std::invalid_argument("failed to initialize the '" + plugin_type + "' plugin: " + ex.what());
    }
  }
}

bool LocalPlannerHierarchy::setPlan(const arti_nav_core_msgs::Path2DWithLimits& plan)
{
  for (const auto& plugin: plugins_)
  {
    if (!plugin->setPlan(plan))
    {
      ROS_ERROR("can not set the plan");
      return false;
    }
  }

  return true;
}

bool LocalPlannerHierarchy::setFinalVelocityConstraints(const arti_nav_core_msgs::Twist2DWithLimits& final_twist)
{
  for (const auto& plugin: plugins_)
  {
    if (!plugin->setFinalVelocityConstraints(final_twist))
    {
      ROS_ERROR("can not set the final velocity constraint");
      return false;
    }
  }

  return true;
}

arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum LocalPlannerHierarchy::makeTrajectory(
  arti_nav_core_msgs::Trajectory2DWithLimits& trajectory)
{
  arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum last_result = arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::NO_TRAJECTORY_POSSIBLE;
  for (const auto& plugin: plugins_)
  {
    ROS_INFO_STREAM("try plugin to find a trajectory");

    arti_nav_core_msgs::Trajectory2DWithLimits tmp_trajectory;
    arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum tmp_result = plugin->makeTrajectory(tmp_trajectory);
    if (tmp_result == arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum::TRAJECTORY_FOUND)
    {
      ROS_INFO_STREAM("found trajectory");

      trajectory = tmp_trajectory;
      return tmp_result;
    }

    last_result = tmp_result;
  }

  ROS_ERROR("no plan can be found with any of the planners");

  return last_result;
}

bool LocalPlannerHierarchy::isGoalReached()
{
  for (const auto& plugin: plugins_)
  {
    if (plugin->isGoalReached())
    {
      return true;
    }
  }

  return false;
}
}

PLUGINLIB_EXPORT_CLASS(arti_planner_hierarchy::LocalPlannerHierarchy, arti_nav_core::BaseLocalPlanner)
