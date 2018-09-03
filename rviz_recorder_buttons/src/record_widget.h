#ifndef RECORD_WIDGET_H
#define RECORD_WIDGET_H

#include <QWidget>
#include <memory>

namespace Ui 
{
  class RecordWidget;
}

namespace rviz_recorder_buttons
  {

  class RecordWidget : public QWidget
  {
    Q_OBJECT
  public:
    RecordWidget(QWidget* parent = 0);
    virtual ~RecordWidget();
  public Q_SLOTS:
    void Record_Clicked();
    void Pause_Clicked();
    void Discard_Clicked();
    void Save_Clicked();
  Q_SIGNALS:
    void BtnClicked(std::string action);
  private:
    bool pause = false;
    void Reset();
    std::unique_ptr<Ui::RecordWidget> ui_;

  };
}

#endif // RECORD_WIDGET_H
