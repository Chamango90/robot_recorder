#include "record_widget.h"
#include "ui_record_widget.h"

#include <std_srvs/Trigger.h>
#include <ros/ros.h>

namespace rviz_recorder_buttons
{

RecordWidget::RecordWidget(QWidget *parent) :
    QWidget(parent), ui_(new Ui::RecordWidget)
{
    ui_->setupUi(this);

    ui_->RecordButton->setIcon(style()->standardIcon(QStyle::QStyle::SP_MediaPlay));
    ui_->PauseButton->setIcon(style()->standardIcon(QStyle::QStyle::SP_MediaPause));
    ui_->StopButton->setIcon(style()->standardIcon(QStyle::QStyle::SP_DialogSaveButton));

    connect(ui_->RecordButton, SIGNAL(clicked()), this, SLOT(Record_Clicked()));
    connect(ui_->PauseButton, SIGNAL(clicked()), this, SLOT(Pause_Clicked()));
    connect(ui_->StopButton, SIGNAL(clicked()), this, SLOT(Stop_Clicked()));
}

RecordWidget::~RecordWidget() = default;

void RecordWidget::Record_Clicked()
{
  ui_->RecordButton->setText("Recording");
  ui_->PauseButton->setText("Pause");
  ui_->RecordButton->setEnabled(false);
  ui_->PauseButton->setEnabled(true);
  ui_->StopButton->setEnabled(true);
  Q_EMIT RecordClicked();
}

void RecordWidget::Pause_Clicked()
{
  if(ui_->PauseButton->text() == "Pause")
  {
    ui_->PauseButton->setText("Unpause");
  }
  else
  {
    ui_->PauseButton->setText("Pause");
  }
  Q_EMIT PauseClicked();
}

void RecordWidget::Stop_Clicked()
{
  ui_->RecordButton->setText("Record");
  ui_->PauseButton->setText("Pause");
  ui_->RecordButton->setEnabled(true);
  ui_->PauseButton->setEnabled(false);
  ui_->StopButton->setEnabled(false);
  Q_EMIT StopClicked();
}


}
