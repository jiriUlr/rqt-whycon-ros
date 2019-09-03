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

#include <rqt_whycon_ros/rqt_whycon_ros.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QStandardPaths>

namespace rqt_whycon_ros
{

RqtWhyconRos::RqtWhyconRos()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , rotate_state_(ROTATE_0)
  , coord_state_(COORD_CAMERA)
  , calib_state_(CALIB_AUTO)
{
  setObjectName("RqtWhyconRos");
}

void RqtWhyconRos::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // init widget
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  // if more instances then display number in window title
  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // build up nodes list and fill combo box
  updateNodeList();
  ui_.nodes_combo_box->setCurrentIndex(ui_.nodes_combo_box->findText(""));
  connect(ui_.nodes_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onNodeChanged(int)));
  connect(ui_.refresh_nodes_push_button, SIGNAL(pressed()), this, SLOT(updateNodeList()));

  // buttons to control quility and dimensions of image
  connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));
  connect(ui_.smooth_image_push_button, SIGNAL(toggled(bool)), ui_.image_frame, SLOT(onSmoothImageChanged(bool)));

  // set up rotation control
  connect(ui_.rotate_left_push_button, SIGNAL(clicked(bool)), this, SLOT(onRotateLeft()));
  connect(ui_.rotate_right_push_button, SIGNAL(clicked(bool)), this, SLOT(onRotateRight()));

  // Make sure we have enough space for "XXX °"
  ui_.rotate_label->setMinimumWidth(ui_.rotate_label->fontMetrics().width("XXX°"));

  // build coordinate systems and fill combo box
  createCoordinateList();
  ui_.coordinates_combo_box->setCurrentIndex(ui_.coordinates_combo_box->findText(coord_states_str_[coord_state_].c_str()));
  connect(ui_.coordinates_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onCoordinateChanged(int)));

  // set up calibration stack
  connect(ui_.load_calib_push_button, SIGNAL(pressed()), this, SLOT(loadCalib()));
  connect(ui_.save_calib_push_button, SIGNAL(pressed()), this, SLOT(saveCalib()));
  connect(ui_.save_as_image_push_button, SIGNAL(pressed()), this, SLOT(saveImage()));

  // set up buttons to control displayed image
  connect(ui_.draw_segments_push_button, SIGNAL(toggled(bool)), this, SLOT(onDrawChanged()));
  connect(ui_.draw_coords_push_button, SIGNAL(toggled(bool)), this, SLOT(onDrawChanged()));

  // set up calibration button group
  ui_.calib_button_group->setId(ui_.auto_calib_push_button, CALIB_AUTO);
  ui_.calib_button_group->setId(ui_.man_calib_push_button, CALIB_MAN);
  connect(ui_.calib_button_group, SIGNAL(buttonClicked(int)), this, SLOT(onCalibMethod()));
  connect(ui_.recalib_push_button, SIGNAL(clicked(bool)), this, SLOT(onCalibMethod()));

  // set up displaying image widget
  ui_.image_frame->setOuterLayout(ui_.image_layout);
  connect(ui_.image_frame, SIGNAL(mouseLeft(int, int)), this, SLOT(onMouseLeft(int, int)));

  // set up action to hide toolbad area
  hide_toolbar_action_ = new QAction(tr("Hide toolbar"), this);
  hide_toolbar_action_->setCheckable(true);
  ui_.image_frame->addAction(hide_toolbar_action_);
  connect(hide_toolbar_action_, SIGNAL(toggled(bool)), this, SLOT(onHideToolbarChanged(bool)));
}

void RqtWhyconRos::shutdownPlugin()
{
  // disconnect from the node
  img_sub_.shutdown();
  drawing_client_.shutdown();
  coord_system_client_.shutdown();
  calib_method_client_.shutdown();
  calib_path_client_.shutdown();
  select_marker_client_.shutdown();
}

void RqtWhyconRos::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  instance_settings.setValue("topic", ui_.nodes_combo_box->currentText());
  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  instance_settings.setValue("smooth_image", ui_.smooth_image_push_button->isChecked());
  instance_settings.setValue("rotate", rotate_state_);
  instance_settings.setValue("draw_segments", ui_.draw_segments_push_button->isChecked());
  instance_settings.setValue("draw_coords", ui_.draw_coords_push_button->isChecked());
  instance_settings.setValue("toolbar_hidden", hide_toolbar_action_->isChecked());
}

void RqtWhyconRos::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  QString topic = instance_settings.value("topic", "").toString();
  selectNode(topic);

  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
  ui_.zoom_1_push_button->setChecked(zoom1_checked);

  bool smooth_image_checked = instance_settings.value("smooth_image", false).toBool();
  ui_.smooth_image_push_button->setChecked(smooth_image_checked);

  rotate_state_ = static_cast<RotateState>(instance_settings.value("rotate", 0).toInt());
  if(rotate_state_ >= ROTATE_STATE_COUNT)
    rotate_state_ = ROTATE_0;
  syncRotateLabel();

  bool draw_segments_checked = instance_settings.value("draw_segments", false).toBool();
  ui_.draw_segments_push_button->setChecked(draw_segments_checked);

  bool draw_coords_checked = instance_settings.value("draw_coords", true).toBool();
  ui_.draw_coords_push_button->setChecked(draw_coords_checked);

  bool toolbar_hidden = instance_settings.value("toolbar_hidden", false).toBool();
  hide_toolbar_action_->setChecked(toolbar_hidden);
}

void RqtWhyconRos::createCoordinateList()
{
  // fill combo box
  ui_.coordinates_combo_box->clear();
  ui_.coordinates_combo_box->addItem(QString(coord_states_str_[COORD_CAMERA].c_str()), QVariant(COORD_CAMERA));
  ui_.coordinates_combo_box->addItem(QString(coord_states_str_[COORD_2D].c_str()), QVariant(COORD_2D));
  ui_.coordinates_combo_box->addItem(QString(coord_states_str_[COORD_3D].c_str()), QVariant(COORD_3D));
}

void RqtWhyconRos::updateNodeList()
{
  // msg types to look for
  QSet<QString> message_types;
  message_types.insert("whycon_ros/MarkerArray");

  // previously selected node
  QString selected = ui_.nodes_combo_box->currentText();

  // get current topics on rosmaster that matche the type
  QList<QString> topics = getTopics(message_types).values();

  // extract node names from topics
  QList<QString> nodes;
  for(QString& str : topics)
  {
    str.chop(str.size() - str.lastIndexOf("/"));
    nodes.append(str);
  }
  nodes.append("");
  qSort(nodes);

  // fill combo box
  ui_.nodes_combo_box->clear();
  for (const QString& node : nodes)
  {
    ui_.nodes_combo_box->addItem(node, QVariant(node));
  }

  // restore previous selection
  selectNode(selected);
}

QSet<QString> RqtWhyconRos::getTopics(const QSet<QString>& message_types)
{
  ros::master::V_TopicInfo topics_info;
  ros::master::getTopics(topics_info);

  QSet<QString> topics;
  for (const ros::master::TopicInfo& topic : topics_info)
  {
    if (message_types.contains(topic.datatype.c_str()))
    {
      topics.insert(topic.name.c_str());
      //qDebug() << "RqtWhyconRos::getTopics() topic " << topic;
    }
  }
  return topics;
}

void RqtWhyconRos::selectNode(const QString& node)
{
  int index = ui_.nodes_combo_box->findText(node);
  if (index == -1)
  {
    // add node name to list if not yet in
    // mainly for restoring last used configuration
    ui_.nodes_combo_box->addItem(node, QVariant(node));
    index = ui_.nodes_combo_box->findText(node);
    //qDebug() << "RqtWhyconRos::selectNode() adding node " << node;
  }
  ui_.nodes_combo_box->setCurrentIndex(index);
}

void RqtWhyconRos::onNodeChanged(int index)
{
  // disconnect from previous node
  img_sub_.shutdown();
  drawing_client_.shutdown();
  coord_system_client_.shutdown();
  calib_method_client_.shutdown();
  calib_path_client_.shutdown();
  select_marker_client_.shutdown();

  // reset image on topic change
  ui_.image_frame->setImage(QImage());

  // selected new node
  QString node = ui_.nodes_combo_box->itemData(index).toString();

  // connnect to new node if valid
  if (!node.isEmpty())
  {
    std::string node_str = node.toStdString();
    image_transport::ImageTransport it(getNodeHandle());
    img_sub_ = it.subscribe(node_str + "/processed_image", 1, &RqtWhyconRos::imageCallback, this);
    drawing_client_ = getNodeHandle().serviceClient<whycon_ros::SetDrawing>(node_str + "/set_drawing");
    coord_system_client_ = getNodeHandle().serviceClient<whycon_ros::SetCoords>(node_str + "/set_coords");
    calib_method_client_ = getNodeHandle().serviceClient<whycon_ros::SetCalibMethod>(node_str + "/set_calib_method");
    calib_path_client_ = getNodeHandle().serviceClient<whycon_ros::SetCalibPath>(node_str + "/set_calib_path");
    select_marker_client_ = getNodeHandle().serviceClient<whycon_ros::SelectMarker>(node_str + "/select_marker");
    //qDebug() << "RqtWhyconRos::onNodeChanged() to node " << node;
  }
}

void RqtWhyconRos::onCoordinateChanged(int index)
{
  if(coord_system_client_.exists())
  {
    coord_state_ = static_cast<CoordState>(ui_.coordinates_combo_box->itemData(index).toInt());

    whycon_ros::SetCoords set_coords;
    set_coords.request.coords = coord_state_;
    coord_system_client_.call(set_coords);

    // decide if coordinate change was successful before enabling bottons
    if(set_coords.response.success)
    {
      switch(coord_state_)
      {
        default:
        case COORD_CAMERA:
          {
            // disable calibration options because it's not needed
            if(ui_.auto_calib_push_button->isEnabled())
            {
              ui_.auto_calib_push_button->setEnabled(false);
            }
            if(ui_.man_calib_push_button->isEnabled())
            {
              ui_.man_calib_push_button->setEnabled(false);
            }
            if(ui_.recalib_push_button->isEnabled())
            {
              ui_.recalib_push_button->setEnabled(false);
            }
            break;
          }
        case COORD_2D:
        case COORD_3D:
          {
            // enable calibration options
            if(!ui_.auto_calib_push_button->isEnabled())
            {
              ui_.auto_calib_push_button->setEnabled(true);
            }
            if(!ui_.man_calib_push_button->isEnabled())
            {
              ui_.man_calib_push_button->setEnabled(true);
            }
            if(!ui_.recalib_push_button->isEnabled())
            {
              ui_.recalib_push_button->setEnabled(true);
            }
            break;
          }
      }
    }
    else
    {
      QMessageBox::warning(widget_, tr("Coordinate system fail"), set_coords.response.msg.c_str());

      // fallback to camera coordinates
      coord_state_ = COORD_CAMERA;
      if(ui_.auto_calib_push_button->isEnabled())
      {
        ui_.auto_calib_push_button->setEnabled(false);
      }
      if(ui_.man_calib_push_button->isEnabled())
      {
        ui_.man_calib_push_button->setEnabled(false);
      }
      if(ui_.recalib_push_button->isEnabled())
      {
        ui_.recalib_push_button->setEnabled(false);
      }
    }
  }
  else
  {
    //qDebug() << "RqtWhyconRos::onCoordinateChanged() client" << coord_system_client_.getService() << ".exists() returned false";
  }
}

void RqtWhyconRos::onDrawChanged()
{
  if(drawing_client_.exists())
  {
    whycon_ros::SetDrawing set_drawing;
    set_drawing.request.draw_coords = ui_.draw_coords_push_button->isChecked();
    set_drawing.request.draw_segments = ui_.draw_segments_push_button->isChecked();
    drawing_client_.call(set_drawing);
  }
  else
  {
    //qDebug() << "RqtWhyconRos::onDrawChanged() client" << drawing_client_.getService() << ".exists() returned false";
  }
}

void RqtWhyconRos::onCalibMethod()
{
  if(calib_method_client_.exists())
  {
    whycon_ros::SetCalibMethod calib_method;
    calib_method.request.method = ui_.calib_button_group->checkedId();
    calib_method_client_.call(calib_method);

    if(!calib_method.response.success)
    {
      QMessageBox::critical(widget_, tr("Failed to recalibrate"), calib_method.response.msg.c_str());
    }
  }
  else
  {
    //qDebug() << "RqtWhyconRos::onCalibMethod() client" << calib_method_client_.getService() << ".exists() returned false";
  }
}

void RqtWhyconRos::onZoom1(bool checked)
{
  if (checked)
  {
    if (ui_.image_frame->getImage().isNull())
    {
      return;
    }
    ui_.image_frame->setInnerFrameFixedSize(ui_.image_frame->getImage().size());
  } else {
    ui_.image_frame->setInnerFrameMinimumSize(QSize(80, 60));
    ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    widget_->setMinimumSize(QSize(80, 60));
    widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
  }
}

void RqtWhyconRos::loadCalib()
{
  QString file_name = QFileDialog::getOpenFileName(widget_, tr("Open calibration file"), QStandardPaths::writableLocation(QStandardPaths::HomeLocation), tr("YAML file (*.yaml)"));
  if(file_name.isEmpty())
  {
    return;
  }

  if(calib_path_client_.exists())
  {
    whycon_ros::SetCalibPath calib_path;
    calib_path.request.action = "load";
    calib_path.request.path = file_name.toStdString();
    calib_path_client_.call(calib_path);
    
    if(!calib_path.response.success)
    {
      QMessageBox::critical(widget_, tr("Failed to load calibration"), calib_path.response.msg.c_str());
    }
  }
  else
  {
    //qDebug() << "RqtWhyconRos::loadCalib() client" << calib_path_client_.getService() << ".exists() returned false";
  }
}

void RqtWhyconRos::saveCalib()
{
  QString file_name = QFileDialog::getSaveFileName(widget_, tr("Save calibration file"), QStandardPaths::writableLocation(QStandardPaths::HomeLocation), tr("YAML file (*.yaml)"));
  if(file_name.isEmpty())
  {
    return;
  }

  if(calib_path_client_.exists())
  {
    whycon_ros::SetCalibPath calib_path;
    calib_path.request.action = "save";
    calib_path.request.path = file_name.toStdString();
    calib_path_client_.call(calib_path);

    if(!calib_path.response.success)
    {
      QMessageBox::critical(widget_, tr("Failed to save calibration"), calib_path.response.msg.c_str());
    }
  }
  else
  {
    //qDebug() << "RqtWhyconRos::saveCalib() client" << calib_path_client_.getService() << ".exists() returned false";
  }
}

void RqtWhyconRos::saveImage()
{
  // take a snapshot before asking for the filename
  QImage img = ui_.image_frame->getImageCopy();

  QString file_name = QFileDialog::getSaveFileName(widget_, tr("Save as image"), "image.png", tr("Image (*.bmp *.jpg *.png *.tiff)"));
  if (file_name.isEmpty())
  {
    return;
  }

  img.save(file_name);
}

void RqtWhyconRos::onMouseLeft(int x, int y)
{
  if(select_marker_client_.exists())
  {
    if(!ui_.image_frame->getImage().isNull())
    {
      geometry_msgs::Point clickCanvasLocation;

      // create point in pixel coordinates
      clickCanvasLocation.x = round((double)x/(double)ui_.image_frame->width()*(double)ui_.image_frame->getImage().width());
      clickCanvasLocation.y = round((double)y/(double)ui_.image_frame->height()*(double)ui_.image_frame->getImage().height());
      clickCanvasLocation.z = 0;

      geometry_msgs::Point clickLocation = clickCanvasLocation;

      // rotate point based on rotation of the image
      switch(rotate_state_)
      {
        case ROTATE_90:
          clickLocation.x = clickCanvasLocation.y;
          clickLocation.y = ui_.image_frame->getImage().width() - clickCanvasLocation.x;
          break;
        case ROTATE_180:
          clickLocation.x = ui_.image_frame->getImage().width() - clickCanvasLocation.x;
          clickLocation.y = ui_.image_frame->getImage().height() - clickCanvasLocation.y;
          break;
        case ROTATE_270:
          clickLocation.x = ui_.image_frame->getImage().height() - clickCanvasLocation.y;
          clickLocation.y = clickCanvasLocation.x;
          break;
        default:
          break;
      }

      whycon_ros::SelectMarker marker;
      marker.request.point = clickLocation;
      select_marker_client_.call(marker);
    }
  }
  else
  {
    //qDebug() << "RqtWhyconRos::onMouseLeft() client" << select_marker_client_.getService() << ".exists() returned false";
  }
}

void RqtWhyconRos::onHideToolbarChanged(bool hide)
{
  ui_.toolbar_widget->setVisible(!hide);
}

void RqtWhyconRos::onRotateLeft()
{
  int m = rotate_state_ - 1;
  if(m < 0)
    m = ROTATE_STATE_COUNT-1;

  rotate_state_ = static_cast<RotateState>(m);
  syncRotateLabel();
}

void RqtWhyconRos::onRotateRight()
{
  rotate_state_ = static_cast<RotateState>((rotate_state_ + 1) % ROTATE_STATE_COUNT);
  syncRotateLabel();
}

void RqtWhyconRos::syncRotateLabel()
{
  switch(rotate_state_)
  {
    default:
    case ROTATE_0:   ui_.rotate_label->setText("0°"); break;
    case ROTATE_90:  ui_.rotate_label->setText("90°"); break;
    case ROTATE_180: ui_.rotate_label->setText("180°"); break;
    case ROTATE_270: ui_.rotate_label->setText("270°"); break;
  }
}

void RqtWhyconRos::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else {
        qWarning("RqtWhyconRos.imageCallback() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        ui_.image_frame->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("RqtWhyconRos.imageCallback() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      ui_.image_frame->setImage(QImage());
      return;
    }
  }

  // Handle rotation
  switch(rotate_state_)
  {
    case ROTATE_90:
    {
      cv::Mat tmp;
      cv::transpose(conversion_mat_, tmp);
      cv::flip(tmp, conversion_mat_, 1);
      break;
    }
    case ROTATE_180:
    {
      cv::Mat tmp;
      cv::flip(conversion_mat_, tmp, -1);
      conversion_mat_ = tmp;
      break;
    }
    case ROTATE_270:
    {
      cv::Mat tmp;
      cv::transpose(conversion_mat_, tmp);
      cv::flip(tmp, conversion_mat_, 0);
      break;
    }
    default:
      break;
  }

  // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
  ui_.image_frame->setImage(image);

  if (!ui_.zoom_1_push_button->isEnabled())
  {
    ui_.zoom_1_push_button->setEnabled(true);
  }
  // Need to update the zoom 1 every new image in case the image aspect ratio changed,
  // though could check and see if the aspect ratio changed or not.
  onZoom1(ui_.zoom_1_push_button->isChecked());
}

}

PLUGINLIB_EXPORT_CLASS(rqt_whycon_ros::RqtWhyconRos, rqt_gui_cpp::Plugin)
