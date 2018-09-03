#ifndef RECORD_PANEL_H
#define RECORD_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Trigger.h>


namespace rviz_recorder_buttons {

  class RecordWidget;

  class RecordPanel: public rviz::Panel
  {
    Q_OBJECT
    public:
      RecordPanel( QWidget* parent = 0);
    public Q_SLOTS:
      void call_srv(std::string action);
    private:
      RecordWidget *widget_;
      std::string node_ns;
      std::string start = "start", pause = "pause", disc = "discard", save = "save";
      std_srvs::Trigger trigger;
      std::map<std::string, ros::ServiceClient> clients;
    protected:
      ros::NodeHandle nh_;
  };
}

#endif // RECORD_PANEL_H
