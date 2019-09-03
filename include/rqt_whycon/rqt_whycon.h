/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RQT_WHYCON__RQT_WHYCON_H
#define RQT_WHYCON__RQT_WHYCON_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_rqt_whycon.h>

#include <ros/package.h>
#include <ros/macros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>

#include <QSet>
#include <QAction>
#include <QString>
#include <QWidget>

#include <vector>

#include "whycon_ros/SelectMarker.h"
#include "whycon_ros/SetCalibMethod.h"
#include "whycon_ros/SetCalibPath.h"
#include "whycon_ros/SetCoords.h"
#include "whycon_ros/SetDrawing.h"

namespace rqt_whycon
{

class RqtWhycon : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  RqtWhycon();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:

  virtual void onNodeChanged(int index);

  virtual void updateNodeList();

  virtual void onZoom1(bool checked);

  virtual void onRotateLeft();

  virtual void onRotateRight();

  virtual void onCoordinateChanged(int index);

  virtual void saveImage();

  virtual void loadCalib();

  virtual void saveCalib();

  virtual void onDrawChanged();

  virtual void onCalibMethod();

  virtual void onMouseLeft(int x, int y);

  virtual void onHideToolbarChanged(bool hide);

protected:

  virtual void createCoordinateList();

  virtual QSet<QString> getTopics(const QSet<QString>& message_types);

  virtual void selectNode(const QString& node);

  virtual void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  Ui::RqtWhyconWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber img_sub_;

  cv::Mat conversion_mat_;

private:

  enum RotateState
  {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };

  enum CoordState
  {
    COORD_CAMERA = 0,
    COORD_2D = 1,
    COORD_3D = 2,

    COORD_STATE_COUNT
  };

  enum CalibState
  {
    CALIB_AUTO = 0,
    CALIB_MAN = 1,

    CALIB_STATE_COUNT
  };

  const std::vector<QString> coord_states_str_
  {
    "camera coordinates",
    "2D coordinates",
    "3D coordinates"
  };

  void syncRotateLabel();
    
  ros::ServiceClient drawing_client_;
  ros::ServiceClient coord_system_client_;
  ros::ServiceClient calib_method_client_;
  ros::ServiceClient calib_path_client_;
  ros::ServiceClient select_marker_client_;

  QAction* hide_toolbar_action_;

  RotateState rotate_state_;
  CoordState coord_state_;
  CalibState calib_state_;
};

}

#endif  // RQT_WHYCON__RQT_WHYCON_H
