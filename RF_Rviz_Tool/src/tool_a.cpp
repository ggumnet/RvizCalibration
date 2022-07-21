/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "tool_a.h"

namespace rf_rviz_tool
{
ToolA::ToolA()
{
  shortcut_key_ = Qt::Key_A;
  keyPub = nh.advertise<std_msgs::Int32>("rf_tool_a", 1);
}
ToolA::~ToolA()
{
}
void ToolA::onInitialize()
{
}
void ToolA::activate()
{
  ROS_INFO_STREAM("tool a activated\n");
  std_msgs::Int32 msg;
  msg.data = 0;
  keyPub.publish(msg);

  deactivate();
}
void ToolA::deactivate()
{
  ;
}
int ToolA::processMouseEvent(rviz::ViewportMouseEvent &event)
{
  return Finished;
}
void ToolA::save(rviz::Config config) const
{
  rviz::Tool::save(config);
}
void ToolA::load(const rviz::Config &config)
{
  rviz::Tool::load(config);
}

int ToolA::processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel)
{
}
} // end namespace rf_rviz_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rf_rviz_tool::ToolA, rviz::Tool)
// END_TUTORIAL
