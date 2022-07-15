/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
#ifndef TOOL_A_CLICK_H
#define TOOL_A_CLICK_H

#include <ros/ros.h>
#include <rviz/tool.h>
#include <QCursor>
#include <QObject>
namespace Ogre
{
class SceneNode;
class Vector3;
} // namespace Ogre

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
} // namespace rviz

namespace rf_rviz_tool
{

class ToolAClick : public rviz::Tool
{
  Q_OBJECT
public:
  ToolAClick();
  ~ToolAClick();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processKeyEvent(QKeyEvent *event, rviz::RenderPanel *panel);
  virtual int processMouseEvent(rviz::ViewportMouseEvent &event);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

private:
  ros::NodeHandle nh;
  ros::Publisher keyPub;
};

} // end namespace rf_rviz_tool

#endif // TOOL_CLICK_H
