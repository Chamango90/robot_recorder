#include "record_panel.h"
#include "record_widget.h"

#include <QVBoxLayout>
#include <std_srvs/Trigger.h>
#include <ros/ros.h>


namespace rviz_recorder_buttons {

RecordPanel::RecordPanel( QWidget* parent)
: rviz::Panel(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new RecordWidget();
  layout->addWidget(widget_);
  setLayout(layout);

  parent_ns = "/robot_recorder";
  connect(widget_, SIGNAL(RecordClicked()), this, SLOT(record_clicked()));
  connect(widget_, SIGNAL(PauseClicked()), this, SLOT(pause_clicked()));
  connect(widget_, SIGNAL(StopClicked()), this, SLOT(stop_clicked()));
}

void RecordPanel::record_clicked()
{
	std::string service = parent_ns + "/start";
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(service);
    std_srvs::Trigger trigger;
    client.call(trigger);
}

void RecordPanel::pause_clicked()
{
	std::string service = parent_ns + "/pause";
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(service);
    std_srvs::Trigger trigger;
    client.call(trigger);
}

void RecordPanel::stop_clicked()
{
	std::string service = parent_ns + "/stop";
    ros::ServiceClient client = nh_.serviceClient<std_srvs::Trigger>(service);
    std_srvs::Trigger trigger;
    client.call(trigger);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_recorder_buttons::RecordPanel, rviz::Panel )
