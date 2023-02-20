#include "rviz_widget.h"

#include <QHBoxLayout>
#include <QContextMenuEvent>
#include <QMenu>
#include <QInputDialog>
#include <QLabel>
#include <tf/transform_listener.h>
#include <fmt/format.h>

RvizWidget::RvizWidget(QWidget* parent)
{
    _render_panel = new rviz::RenderPanel(this);

    auto label = new QLabel;
    label->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,
                                     QSizePolicy::Fixed));
    label->setText("Right click to show menu");

    auto l = new QVBoxLayout;
    l->addWidget(label);
    l->addWidget(_render_panel);
    setLayout(l);

    _manager = new rviz::VisualizationManager(_render_panel);
    _render_panel->initialize(_manager->getSceneManager(),
                              _manager);
    _manager->initialize();
    _manager->startUpdate();
    _manager->setFixedFrame("base_link");

    //_robot_model = _manager->createDisplay("rviz/RobotModel",
    //                                       "EC Client Robot",
    //                                       true);

    //_robot_model->subProp("Robot Description")->
    //    setValue("/ec_client/robot_description");

    _robot_marker_array= _manager->createDisplay("rviz/MarkerArray",
                                                   "EC Client Robot",
                                                   true);

    _robot_marker_array->subProp("Marker Topic")->
                          setValue("/ec_client/robot");

    Ogre::ColourValue colour(1, 1, 1, 0.9);
    _render_panel->setBackgroundColor(colour);

    _manager->createDisplay("rviz/Grid",
                            "Grid",
                            true);

}


//bool RvizWidget::init(Args& args)
//{
//    return CustomQtWidget::init(args);
//}

void RvizWidget::update()
{
}

QString RvizWidget::name()
{
    return "RViz";
}

//RvizWidget::~RvizWidget()
//{
//    fmt::print("{}\n", __func__);
//}


void RvizWidget::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);

    // set fixed frame
    auto * set_fixed_frame = new QAction("Set fixed frame", this);

    // get list of frames
    std::vector<std::string> frames_std;
    _manager->getTF2BufferPtr()->_getFrameStrings(frames_std);

    // convert to qt
    QStringList frames;
    std::transform(frames_std.begin(), frames_std.end(),
                   std::back_inserter(frames),
                   [](auto stdstr)
                   {
                       return QString::fromStdString(stdstr);
                   });

    // find current index
    int current_idx = frames.indexOf(_manager->getFixedFrame());

    connect(set_fixed_frame, &QAction::triggered,
            [this, frames, current_idx]()
            {
                bool ok;
                QString text = QInputDialog::getItem(
                    this,
                    "RViz fixed frame selection",
                    "Fixed frame:",
                    frames,
                    current_idx,
                    false,
                    &ok);

                if(ok && !text.isEmpty())
                {
                    _manager->setFixedFrame(text);
                }
            });

    menu.addAction(set_fixed_frame);

    // set tf prefix
    auto * set_tf_prefix = new QAction("Set tf prefix", this);

    connect(set_tf_prefix, &QAction::triggered,
            [this, event]()
            {
                bool ok;
                auto tfp = _robot_model->subProp("TF Prefix")->getValue().toString();
                QString text = QInputDialog::getText(
                    this,
                    "Robot model TF prefix selection",
                    "TF prefix:",
                    QLineEdit::Normal,
                    tfp,
                    &ok);

                if(ok)
                {
                    _robot_model->subProp("TF Prefix")->setValue(text);
                }
            });

    menu.addAction(set_tf_prefix);

    // set param name
    auto * set_param_name = new QAction("Set robot description", this);

    connect(set_param_name, &QAction::triggered,
            [this, event]()
            {
                bool ok;
                auto rd = _robot_model->subProp("Robot Description")->
                           getValue().toString();
                QString text = QInputDialog::getText(
                    this,
                    "Robot model description selection",
                    "Robot description:",
                    QLineEdit::Normal,
                    rd,
                    &ok);

                if(ok && !text.isEmpty())
                {
                    _robot_model->subProp("Robot Description")->
                        setValue(text);
                }
            });

    menu.addAction(set_param_name);

    menu.exec(event->globalPos());
}


bool RvizWidget::loadConfig(const YAML::Node& cfg)
{
    if(auto ff = cfg["fixed_frame"])
    {
        _manager->setFixedFrame(QString::fromStdString(ff.as<std::string>()));
    }

    if(auto tp = cfg["tf_prefix"])
    {
        auto text = QString::fromStdString(tp.as<std::string>());
        _robot_model->subProp("TF Prefix")->
            setValue(text);
    }

    if(auto rd = cfg["robot_description"])
    {
        auto text = QString::fromStdString(rd.as<std::string>());
        _robot_model->subProp("Robot Description")->
            setValue(text);
    }

    return true;
}

bool RvizWidget::saveConfig(YAML::Node& cfg)
{
    cfg["fixed_frame"] = _manager->getFixedFrame().toStdString();
    cfg["tf_prefix"] = _robot_model->subProp("TF Prefix")->getValue().toString().toStdString();
    cfg["robot_description"] =  _robot_model->subProp("Robot Description")->
                               getValue().toString().toStdString();
    return true;
}

bool RvizWidget::usesOpenGl()
{
    return true;
}
