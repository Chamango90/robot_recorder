#ifndef RECORD_PANEL_H
#define RECORD_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>


namespace rviz_recorder_buttons {

class RecordWidget;

class RecordPanel: public rviz::Panel
{
    Q_OBJECT
public:
    RecordPanel( QWidget* parent = 0);
	std::string parent_ns;
public Q_SLOTS:
    void record_clicked();
    void pause_clicked();
    void stop_clicked();
private:
    RecordWidget *widget_;
protected:
    ros::NodeHandle nh_;
};
}

#endif // RECORD_PANEL_H
