#include "sliders_widget.h"
#include <iostream>


inline void initSlidersResource()
{
    Q_INIT_RESOURCE(slave_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    
    initSlidersResource();
    
    QUiLoader loader;

    QFile file(":/sliders_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}
}

SlidersWidget::SlidersWidget (std::vector<std::string>  joint_names,
                              std::vector<ec_msgs::SlaveDescription> slaves_descr_vector,
                             std::string slider_name,
                             QWidget *parent) :
    QWidget(parent),
    _joint_names(joint_names)
{

    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    layout->setSpacing(0);
    layout->setMargin(0);

    setLayout(layout);

    std::vector<std::string>  posvel_gains={"P","I","D"};
    std::vector<std::string>  imp_gains={"P","D","Tau_p","Tau_d","Tau_fc"};
    std::vector<std::string>  limits={"Min_Pos","Max_Pos","Max_Vel","Max_Torq"};
    std::vector<std::string>  sine={"Freq [Hz]","Ampl [rad]","Theta [rad]","Secs [s]"};
    std::vector<std::string>  smooth={"x1_smooth [s]","y1_smooth [rad]",
                                      "x2_smooth [s]","y2_smooth [rad]",
                                      "x3_smooth [s]","y3_smooth [rad]",
                                      "x4_smooth [s]","y4_smooth [rad]"};

    std::vector<double> init_value;
    std::vector<std::string>  sliders_type;
    int slider_numb;


    if((slider_name=="Pos_Gains")||(slider_name=="Vel_Gains"))
    {
        slider_numb=3;
        sliders_type.resize(slider_numb);
        sliders_type=posvel_gains;
    }
    else if(slider_name=="Imp_Gains")
    {
        slider_numb=5;
        sliders_type.resize(5);
        sliders_type=imp_gains;
    }
    else if(slider_name=="Max_Current")
    {
        slider_numb=1;
        sliders_type.resize(1);
        sliders_type[0]="[Max A]";
    }
    else if(slider_name=="Current")
    {
        slider_numb=1;
        sliders_type.resize(1);
        sliders_type[0]="[A]";
    }
    else if(slider_name=="Limits")
    {
        slider_numb=4;
        sliders_type.resize(4);
        sliders_type=limits;
    }
    else if(slider_name=="Homing")
    {
        slider_numb=1;
        sliders_type.resize(2);
        sliders_type[0]="x1_home [s]";
        sliders_type[1]="x2_home [s]";
    }

    else if(slider_name=="Sine")
    {
        slider_numb=4;
        sliders_type.resize(4);
        sliders_type=sine;
    }
    else if(slider_name=="Smooth")
    {
        slider_numb=8;
        sliders_type.resize(8);
        sliders_type=smooth;
    }
    else if((slider_name=="Position")||(slider_name=="Homing_Position"))
    {
        slider_numb=1;
        sliders_type.resize(1);
        sliders_type[0]="[rad]";
    }
    else if(slider_name=="Velocity")
    {
        slider_numb=1;
        sliders_type.resize(1);
        sliders_type[0]="[rad/s]";
    }
    else if(slider_name=="Torque")
    {
        slider_numb=1;
        sliders_type.resize(1);
        sliders_type[0]="[Nm]";
    }
    else if((slider_name=="LED")||(slider_name=="FAN"))
    {
        slider_numb=1;
        sliders_type.resize(1);
        sliders_type[0]="[OFF/ON]";
    }
    else
    {
        throw std::runtime_error("Error sliders Name");
        return;
    }

    _sliders_leftlayout  = findChild<QVBoxLayout *>("SlidersLeftLayout");
    _sliders_rightlayout = findChild<QVBoxLayout *>("SlidersRightLayout");

    bool left_slider_turn=true;

    _qwid= new QTreeWidget();

    _qwid->setHeaderLabel("Slave Selection");

    for(int i=0; i<_joint_names.size();i++)
    {
        for(int j=0;j<slaves_descr_vector.size();j++)
        {
          ec_msgs::SlaveDescription slave_descr=slaves_descr_vector[j];
          if(slave_descr.slave_name==_joint_names[i])
          {

              if(!init_value.empty())
                init_value.clear();

              if(slider_name=="Pos_Gains")
              {
                init_value.resize(slave_descr.position_gain.size());
                for(int index=0;index < init_value.size();index++)
                    init_value[index]=slave_descr.position_gain[index];
              }
              else if(slider_name=="Vel_Gains")
              {
                  init_value.resize(slave_descr.velocity_gain.size());
                  for(int index=0;index < init_value.size();index++)
                      init_value[index]=slave_descr.velocity_gain[index];
              }
              else if(slider_name=="Imp_Gains")
              {
                  init_value.resize(slave_descr.impedance_gain.size());
                  for(int index=0;index < init_value.size();index++)
                      init_value[index]=slave_descr.impedance_gain[index];
              }
              else if(slider_name=="Max_Current")
              {
                  init_value.push_back(slave_descr.max_current);
              }
              else if(slider_name=="Homing_Position")
                  init_value.push_back(slave_descr.home_position);

              else if((slider_name=="Limits")||(slider_name=="Sine"))
              {
                  init_value.resize(4);
                  for(int index=0;index < init_value.size();index++)
                      init_value[index]=0.0;
              }
              else if(slider_name=="Smooth")
              {
                  init_value.resize(8);
                  for(int index=0;index < init_value.size();index++)
                      init_value[index]=0.0;
              }
              else if((slider_name=="Homing")||
                      (slider_name=="Position")||
                      (slider_name=="Velocity")||
                      (slider_name=="Torque")||
                      (slider_name=="Current")||
                      (slider_name=="LED")||
                      (slider_name=="FAN"))
              {
                  init_value.push_back(0.0);
              }
          }
       }

       QTreeWidgetItem * topLevel = new QTreeWidgetItem();
       QTreeWidgetItem *childItem = new QTreeWidgetItem();

       topLevel->setText(0,QString::fromStdString(_joint_names[i]));

       QString jnames=QString::fromStdString(_joint_names[i]);
       auto wid = new SliderWidget(jnames,init_value,sliders_type,this);

       topLevel->addChild(childItem);


       _qwid->addTopLevelItem(topLevel);
       _qwid->setItemWidget(childItem,0,wid);

       if(_slider_wid_map.count(_joint_names[i])==0)
           _slider_wid_map[_joint_names[i]]=wid;

       if(left_slider_turn)
       {
           left_slider_turn=false;
           _sliders_leftlayout->addWidget(wid,0, Qt::AlignTop);
       }
       else
       {
           left_slider_turn=true;
           _sliders_rightlayout->addWidget(wid,0, Qt::AlignTop);
       }

    }

}
SlidersWidget::~SlidersWidget()
{

}

std::map<std::string, SliderWidget *> SlidersWidget::get_slider_map()
{
    return _slider_wid_map;
}
