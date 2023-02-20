#ifndef EC_GUI_START_H
#define EC_GUI_START_H

#include <QtUiTools>
#include <QWidget>
#include <QLineEdit>

class EcGuiStart : public QWidget
{
    Q_OBJECT

public:

    explicit EcGuiStart(QWidget *parent = nullptr);
    void startGUI();
    void onStarGUICmdReleased();
    void onIgnoreCmdReleased();
    void onLowLevelFileCmdReleased();
    QFileInfoList search_file(QDir dir,QString ch_dir,QString targetStr);

private:
  QVBoxLayout *_l;
  QLineEdit *_lowlevelpath_line;
  std::string _low_level_file="",_joint_map_file="",_robot_file="";
  QString _start_dir="/home";
};

#endif // EC_GUI_START_H
