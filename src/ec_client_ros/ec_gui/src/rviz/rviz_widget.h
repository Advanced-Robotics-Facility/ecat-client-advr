#ifndef RVIZ_WIDGET_H
#define RVIZ_WIDGET_H

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>

#include <yaml-cpp/yaml.h>

class RvizWidget : public QWidget
{

    Q_OBJECT

public:

    RvizWidget(QWidget * parent = nullptr);

    //bool init(Args&);
    void update();
    QString name();

    //~RvizWidget();

private:

    rviz::VisualizationManager* _manager;
    rviz::RenderPanel* _render_panel;
    rviz::Display* _robot_model;
    rviz::Display* _robot_marker_array;


    // QWidget interface
protected:
    void contextMenuEvent(QContextMenuEvent* event) override;

    // CustomQtWidget interface
public:
    bool loadConfig(const YAML::Node& cfg);
    bool saveConfig(YAML::Node& cfg);
    bool usesOpenGl();
};

#endif // RVIZ_WIDGET_H
