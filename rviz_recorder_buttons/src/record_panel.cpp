#include "record_panel.h"
#include "record_widget.h"
#include <QVBoxLayout>
#include <ros/console.h>

namespace rviz_recorder_buttons {

  RecordPanel::RecordPanel( QWidget* parent)
  : rviz::Panel(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new RecordWidget();
  layout->addWidget(widget_);
  setLayout(layout);

  nh_.param<std::string>("/robot_recorder_ns", node_ns, "/robot_recorder");

  clients = {
    { start, nh_.serviceClient<std_srvs::Trigger>(node_ns + "/" + start)},
    { pause, nh_.serviceClient<std_srvs::Trigger>(node_ns + "/" + pause)},
    { disc,  nh_.serviceClient<std_srvs::Trigger>(node_ns + "/" + disc)},
    { save,  nh_.serviceClient<std_srvs::Trigger>(node_ns + "/" + save)}
  };

  connect(widget_, SIGNAL(BtnClicked(std::string)),  this, SLOT(call_srv(std::string)));
  }

  void RecordPanel::call_srv(std::string action)
  {   
    if(!clients[action].call(trigger))
    {
      ROS_ERROR_STREAM("Recorder with name " << node_ns << " is not started or ready.");
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_recorder_buttons::RecordPanel, rviz::Panel )
