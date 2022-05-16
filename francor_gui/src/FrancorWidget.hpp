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

class MainWindow;

namespace francor{


class FrancorWidget : public QObject
{
  Q_OBJECT

public:
  FrancorWidget(MainWindow* parent);
  virtual ~FrancorWidget() = default;

  QGridLayout* getLayout()
  {
    return _grid_layout;
  }
  
  void setCo2Value(int value);

  void setInfoA(const std::string& info)
  {
    _info_text_a->setText(info.c_str());
  }
  void setInfoB(const std::string& info)
  {
    _info_text_b->setText(info.c_str());
  }
  void setInfoC(const std::string& info)
  {
    _info_text_c->setText(info.c_str());
  }

  void setTempFR(const float temp)
  {
    _lbl_temp_front_right->setText(QString::number(temp));
  }

  void setTempFL(const float temp)
  {
    _lbl_temp_front_left->setText(QString::number(temp));
  }

  void setTempBL(const float temp)
  {
    _lbl_temp_back_left->setText(QString::number(temp));
  }

  void setTempBR(const float temp)
  {
    _lbl_temp_back_right->setText(QString::number(temp));
  }

private:
  // QWidget*      _widget;
  QGridLayout*  _grid_layout;
  QProgressBar* _co2bar_fast;
  QProgressBar* _co2bar_slow;
  QLineEdit*    _info_text_a;
  QLineEdit*    _info_text_b;
  QLineEdit*    _info_text_c;

  //btn
  QPushButton* _btn_vid_disable;
  QPushButton* _btn_vid_manip;
  QPushButton* _btn_vid_haz;

  //label
  QLabel* _lbl_temp_front_right;
  QLabel* _lbl_temp_front_left;
  QLabel* _lbl_temp_back_right;
  QLabel* _lbl_temp_back_left;


  MainWindow* _parent;
};

} //namespace francor


#endif  //FRANCORWIDGET_H_
