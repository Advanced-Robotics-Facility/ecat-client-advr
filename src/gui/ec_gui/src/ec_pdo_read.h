#ifndef EC_PDO_READ_H
#define EC_PDO_READ_H

#include <QtUiTools>
#include <QWidget>

#include "Utils.h"
#include "ec_utils.h"
#include "qcustomplot.h"


class EcPDORead
{
public:
      EcPDORead(EcIface::Ptr client,
                QTreeWidget *tree_wid) :
      _client(client),
      _tree_wid(tree_wid)
      {
        _receive_timer= new QElapsedTimer();

        _custom_plot = new QCustomPlot();
        
        // make left and bottom axes transfer their ranges to right and top axes:
        QCustomPlot::connect(_custom_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), _custom_plot->xAxis2, SLOT(setRange(QCPRange)));
        QCustomPlot::connect(_custom_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), _custom_plot->yAxis2, SLOT(setRange(QCPRange)));

        // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
        _custom_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);


        QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
        timeTicker->setTimeFormat("%s");
        _custom_plot->xAxis->setTicker(timeTicker);
        _custom_plot->xAxis->setLabel("Time [s]");
        _custom_plot->axisRect()->setupFullAxesBox();
        _custom_plot->yAxis->setRange(-100, 100);
        
        _update_plot=false;

#ifdef TEST
        for(int i=11; i<37; i++)
        {
            _internal_motor_status_map[i] = std::make_tuple(10,10,0,0,100,25,25,0,0,0,0,0);
        }
#endif
      };
      
      ~EcPDORead(){};
      
      void restart_receive_timer()
      {
        _receive_timer->restart();
      }

      QCustomPlot* get_custom_plot()
      {
         return _custom_plot;
      }
      
      void update_plot()
      {
        if(_update_plot)
        {
            _custom_plot->legend->setVisible(true);
            qint64 ms_receive_time= _receive_timer->elapsed();
            double s_receive_time=(double) ms_receive_time/1000;
            // make key axis range scroll with the data (at a constant range size of 8):
            _custom_plot->xAxis->setRange(s_receive_time, 8, Qt::AlignRight);
            _custom_plot->replot();
            _update_plot=false;
        }
      }

      QTreeWidgetItem * search_slave_into_treewid(std::string esc_id_name);
      QTreeWidgetItem * initial_setup(std::string esc_id_name,QList<QString> pdo_fields);
      void fill_data(std::string esc_id_name,QTreeWidgetItem * topLevel,QList<QString> pdo_fields,std::vector<float> pdo);
      
      void read_motor_status();
      void read_ft6_status();
      void read_pow_status();
      
private:
      EcIface::Ptr _client;
      MotorStatusMap _internal_motor_status_map;
      QTreeWidget *_tree_wid;
      QElapsedTimer *_receive_timer;
      QCustomPlot *_custom_plot;
      std::map<std::string,QCPGraph *> _graph_pdo_map;
      bool _update_plot;
      
      QList<QString> _motor_pdo_fields= {"Link Position",
                                         "Motor Position",
                                         "Link Velocity",
                                         "Motor Velocity",
                                         "Torque",
                                         "Motor Temperature",
                                         "Board Temperature",
                                         "fault",
                                         "rtt",
                                         "op_idx_ack",
                                         "aux",          
                                         "Brake_Sts",
                                         "LED_Sts"};
                                        
       QList<QString> _ft6_pdo_fields=  {"force_x",
                                         "force_y",
                                         "force_z",
                                         "torque_x",
                                         "torque_y",
                                         "torque_z"};
                                         
       QList<QString> _pow_pdo_fields=  {"v_batt",
                                         "v_load",
                                         "i_load",
                                         "temp_pcb",
                                         "temp_heatsink",
                                         "temp_batt"};

};

/************************************* SEARCH SLAVE INTO TREE WID ***************************************/
inline QTreeWidgetItem * EcPDORead::search_slave_into_treewid(std::string esc_id_name)
{
    QTreeWidgetItem * topLevel_read=nullptr;
    for(int i=0;i<_tree_wid->topLevelItemCount();i++)
    {
      topLevel_read =_tree_wid->topLevelItem(i);
      if(esc_id_name==topLevel_read->text(1).toStdString())
      {
          topLevel_read =_tree_wid->topLevelItem(i);
          return(topLevel_read);
      }
    }
    return(nullptr);
}
/************************************* SEARCH SLAVE INTO TREE WID ***************************************/

/************************************* INITIAL SETUP ***************************************/
inline QTreeWidgetItem * EcPDORead::initial_setup(std::string esc_id_name,QList<QString> pdo_fields)
{
      QTreeWidgetItem * topLevelrtn = new QTreeWidgetItem();
      topLevelrtn->setText(1,QString::fromStdString(esc_id_name));
      topLevelrtn->setText(2,"");
      topLevelrtn->setText(3,"Rx");

      for(int index=0; index<pdo_fields.size(); index++)
      {
          QTreeWidgetItem * item = new QTreeWidgetItem();
          item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
          item->setCheckState(1,Qt::Unchecked);
          item->setText(1,pdo_fields.at(index));
          topLevelrtn->addChild(item);

          auto graph_pdo = _custom_plot->addGraph();
          //graph_pdo->setPen(QPen(QColor(40, 110, 255)));
          std::string esc_id_pdo = esc_id_name + "_" + pdo_fields.at(index).toStdString();
          graph_pdo->setName(QString::fromStdString(esc_id_pdo));
          _graph_pdo_map[esc_id_pdo]=graph_pdo;
      }

      _tree_wid->addTopLevelItem(topLevelrtn);
      return(topLevelrtn);
}
/************************************* INITIAL SETUP ***************************************/

/************************************* FILL DATA ***************************************/
inline void EcPDORead::fill_data(std::string esc_id_name,QTreeWidgetItem * topLevel,QList<QString> pdo_fields,std::vector<float> pdo)
{
    /************************************* TIME ************************************************/
    qint64 ms_receive_time= _receive_timer->elapsed();
    double s_receive_time=(double) ms_receive_time/1000;
    topLevel->setText(0,QString::number(s_receive_time, 'f', 3));
    /************************************* TIME ***********************************************/

    for(int k=0; k < pdo.size();k++)
    {
        try{
            /************************************* DATA ***********************************************/
            QString raw_data="";
            for(int k=0; k<pdo_fields.size(); k++)
            {
                QTreeWidgetItem * item = topLevel->child(k);
                QString data=QString::number(pdo[k], 'f', 6);
                item->setText(2,data);
                raw_data=raw_data+data+ " ";
                
                std::string esc_id_pdo = esc_id_name + "_" + pdo_fields.at(k).toStdString();
                auto graph_pdo = _graph_pdo_map[esc_id_pdo];
                if(item->checkState(1)==Qt::Checked)
                {
                    // add data to lines:
                    graph_pdo->addData(s_receive_time,data.toDouble());
                    _update_plot |= true; //update plot
                }
                else
                {
                    graph_pdo->removeFromLegend();
                }
            }
            /************************************* DATA ************************************************/

            /************************************* RAW DATA ********************************************/
            topLevel->setText(4,raw_data);
            /************************************* RAW DATA ********************************************/
            
        }catch (std::out_of_range oor) {}
    }
}

inline void EcPDORead::read_motor_status()
{
    auto motors_status_map= _client->get_motors_status();
    if(motors_status_map.empty())
    {
        motors_status_map=_internal_motor_status_map;
    }

    if(!motors_status_map.empty())
    {
        for ( const auto &[esc_id, motor_status] : motors_status_map)
        {
            std::string esc_id_name="joint_id"+std::to_string(esc_id);
            
            QTreeWidgetItem *topLevel=nullptr;
            topLevel= search_slave_into_treewid(esc_id_name);

            /************************************* INITIAL SETUP ***************************************/
            if(!topLevel)
            {
                topLevel= initial_setup(esc_id_name,_motor_pdo_fields);
            }

            /************************************* INITIAL SETUP ***************************************/

            /************************************* TIME ************************************************/
            qint64 ms_receive_time= _receive_timer->elapsed();
            double s_receive_time=(double) ms_receive_time/1000;
            topLevel->setText(0,QString::number(s_receive_time, 'f', 3));
            /************************************* TIME ***********************************************/

            /************************************* DATA ***********************************************/
            try{
                QString raw_data="";
                for(size_t k=0; k<_motor_pdo_fields.size(); k++)
                {
                    QTreeWidgetItem * item = topLevel->child(k);
                    QString data;
                    if(k < 7 || _motor_pdo_fields[k]=="aux")
                    {
                        float value=boost::get<float>(dynamic_get(k,motor_status));
                        data=QString::number(value, 'f', 6);
                    }
                    else
                    {
                        if(_motor_pdo_fields[k]=="Brake_Sts")
                        {
                            uint32_t cmd_aux_sts=boost::get<uint32_t>(dynamic_get(11,motor_status));
                                      
                            uint32_t brake_sts = cmd_aux_sts & 3; //00 unknown
                                                                  //01 release brake 
                                                                  //10 enganged brake
                                                                 //11 error
                            data=QString::number(brake_sts, 'd', 0);
                        }
                        else if(_motor_pdo_fields[k]=="LED_Sts")
                        {
                            uint32_t cmd_aux_sts=boost::get<uint32_t>(dynamic_get(11,motor_status));
                            uint32_t led_sts= (cmd_aux_sts & 4)/4; // 1 or 0 LED  ON/OFF
                            data=QString::number(led_sts, 'd', 0);
                        }
                        else
                        {
                            uint32_t value=boost::get<uint32_t>(dynamic_get(k,motor_status));
                            data=QString::number(value, 'd', 1);
                        }
                    }
                    
                    item->setText(2,data);

                    raw_data=raw_data+data+ " ";
                    
                    std::string esc_id_pdo = esc_id_name + "_" + _motor_pdo_fields.at(k).toStdString();
                    auto graph_pdo = _graph_pdo_map[esc_id_pdo];
                    
                    if(item->checkState(1)==Qt::Checked)
                    {
                        graph_pdo->addToLegend();
                        // add data to lines:
                        std::string esc_id_pdo = esc_id_name + "_" + _motor_pdo_fields.at(k).toStdString();
                        graph_pdo->addData(s_receive_time,data.toDouble());
                        _update_plot |= true; //update plot
                    }
                    else
                    {
                        graph_pdo->removeFromLegend();
                    }
                }
                
                /************************************* DATA ************************************************/

                /************************************* RAW DATA ********************************************/
                topLevel->setText(4,raw_data);
                /************************************* RAW DATA ********************************************/
            }catch (std::out_of_range oor) {}
        }
    }
}

inline void EcPDORead::read_ft6_status()
{
    auto ft6_status_map= _client->get_ft6_status();
    /*************************************FT*****************************************************************/
     if(!ft6_status_map.empty())
     {
        for ( const auto &[esc_id, ft6_status] : ft6_status_map)
        {
            std::string esc_id_name="joint_id"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel)
            {
                topLevel= initial_setup(esc_id_name,_ft6_pdo_fields);
            }
            
            fill_data(esc_id_name,topLevel,_ft6_pdo_fields,ft6_status);
        }
     }
    /*************************************FT*****************************************************************/
}

inline void EcPDORead::read_pow_status()
{
    auto pow_status_map= _client->get_pow_status();
    /*************************************Power Board*****************************************************************/
     if(!pow_status_map.empty())
     {
        for ( const auto &[esc_id, pow_status] : pow_status_map)
        {
            std::string esc_id_name="joint_id"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel)
            {
                topLevel= initial_setup(esc_id_name,_pow_pdo_fields);
            }

            fill_data(esc_id_name,topLevel,_pow_pdo_fields,pow_status);
        }
     }
    /*************************************Power Board*****************************************************************/
}

#endif // EC_PDO_READ_H


