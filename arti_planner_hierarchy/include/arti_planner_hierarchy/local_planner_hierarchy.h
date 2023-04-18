/*
Created by clemens on 02.03.22.
This file is part of the software provided by ARTI
Copyright (c) 2022, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARTI_PLANNER_HIERARCHY_LOCAL_PLANNER_HIERARCHY_H
#define ARTI_PLANNER_HIERARCHY_LOCAL_PLANNER_HIERARCHY_H

#include <arti_nav_core/base_local_planner.h>
#include <memory>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/server.h>

namespace arti_planner_hierarchy
{
class LocalPlannerHierarchy : public arti_nav_core::BaseLocalPlanner
{
public:
  LocalPlannerHierarchy();

  void initialize(
    std::string name, arti_nav_core::Transformer* transformer, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool setPlan(const arti_nav_core_msgs::Path2DWithLimits& plan) override;

  bool setFinalVelocityConstraints(const arti_nav_core_msgs::Twist2DWithLimits& final_twist) override;

  arti_nav_core::BaseLocalPlanner::BaseLocalPlannerErrorEnum makeTrajectory(
    arti_nav_core_msgs::Trajectory2DWithLimits& trajectory) override;

  bool isGoalReached() override;

private:
  pluginlib::ClassLoader<arti_nav_core::BaseLocalPlanner> plugin_loader_;
  std::vector<pluginlib::UniquePtr<arti_nav_core::BaseLocalPlanner>> plugins_;
};
}

#endif //ARTI_PLANNER_HIERARCHY_LOCAL_PLANNER_HIERARCHY_H
