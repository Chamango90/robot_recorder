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
      ui_->DiscardButton->setIcon(style()->standardIcon(QStyle::QStyle::SP_DialogNoButton));
      ui_->SaveButton->setIcon(style()->standardIcon(QStyle::QStyle::SP_DialogSaveButton));

      connect(ui_->RecordButton, SIGNAL(clicked()), this, SLOT(Record_Clicked()));
      connect(ui_->PauseButton, SIGNAL(clicked()), this, SLOT(Pause_Clicked()));
      connect(ui_->DiscardButton, SIGNAL(clicked()), this, SLOT(Discard_Clicked()));
      connect(ui_->SaveButton, SIGNAL(clicked()), this, SLOT(Save_Clicked()));
  }

  RecordWidget::~RecordWidget() = default;

  void RecordWidget::Record_Clicked()
  {
    ui_->RecordButton->setText("Recording");
    ui_->PauseButton->setText("Pause");
    ui_->RecordButton->setEnabled(false);
    ui_->PauseButton->setEnabled(true);
    ui_->DiscardButton->setEnabled(true);
    ui_->SaveButton->setEnabled(true);
    Q_EMIT BtnClicked("start");
  }

  void RecordWidget::Pause_Clicked()
  {
    if(pause)
    {
      ui_->PauseButton->setText("Unpause");
    }
    else
    {
      ui_->PauseButton->setText("Pause");
    }
    Q_EMIT BtnClicked("pause");

    pause = !pause;
  }

  void RecordWidget::Reset()
  {
    ui_->RecordButton->setText("Record");
    ui_->PauseButton->setText("Pause");
    ui_->RecordButton->setEnabled(true);
    ui_->PauseButton->setEnabled(false);
    ui_->DiscardButton->setEnabled(false);
    ui_->SaveButton->setEnabled(false);
  }

  void RecordWidget::Save_Clicked()
  {
    Reset();
    Q_EMIT BtnClicked("save");
  }

  void RecordWidget::Discard_Clicked()
  {
    Reset();
    Q_EMIT BtnClicked("discard");
  }
    

}
