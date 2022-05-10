#ifndef FRANCORWIDGET_H_
#define FRANCORWIDGET_H_


#include <QWidget>
#include <QApplication>
#include <QPushButton>
#include <QGridLayout>
#include <QProcess>
#include <QProgressBar>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>


namespace francor{

class FrancorWidget : public QObject
{
  Q_OBJECT

public:
  FrancorWidget();
  virtual ~FrancorWidget() = default;

  QGridLayout* getLayout()
  {
    return _grid_layout;
  }
  
  void setCo2Value(int value);

private:
  // QWidget*      _widget;
  QGridLayout*  _grid_layout;
  QProgressBar* _co2bar_fast;
  QProgressBar* _co2bar_slow;
  QLineEdit*    _info_text_a;
  QLineEdit*    _info_text_b;
  QLineEdit*    _info_text_c;


};

} //namespace francor


#endif  //FRANCORWIDGET_H_
