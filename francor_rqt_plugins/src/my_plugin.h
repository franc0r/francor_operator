#ifndef my_namespace__my_plugin_H
#define my_namespace__my_plugin_H
 
// #include <rqt_gui_cpp/plugin.h>
// #include <ui_my_plugin.h>
#include <QWidget>
 
namespace my_namespace
{ 

class MyPlugin : public QObject
{ 
  Q_OBJECT

public: 
  MyPlugin(void);
  virtual ~MyPlugin(void) = default;

public slots:
  void stop(void);

private: 
  // Ui::MyPluginWidget ui_; 
  QWidget* widget_; 
};

} // namespace

#endif // my_namespace__my_plugin_H