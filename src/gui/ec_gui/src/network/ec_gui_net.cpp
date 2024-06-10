#include "ec_gui_net.h"

#define HOSTNAME_COL 1
#define HOSTIP_COL 2
#define HOSTPORT_COL 3

EcGuiNet::EcGuiNet(QWidget *parent) :
    QWidget(parent)
{
    _net_tree_wid = parent->findChild<QTreeWidget *>("NetworkSetup");
    _net_tree_wid->installEventFilter(this);
    connect(_net_tree_wid, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),this, SLOT(OnMouseDoubleClicked(QTreeWidgetItem*, int)));
    
    _net_item = nullptr;
    _net_column=-1;
    
    _server_hostname=_net_tree_wid->topLevelItem(0)->child(1)->text(1);
    
    
    if(_net_tree_wid->topLevelItem(0)->child(1)->text(2)=="localhost")
    {
        _server_ip="127.0.0.1";
    }
    else
    {
        _server_ip=_net_tree_wid->topLevelItem(0)->child(1)->text(2);
    }
    _server_port=_net_tree_wid->topLevelItem(0)->child(1)->text(3);
    
    _password = parent->findChild<QLineEdit *>("Password");
    connect(_password, &QLineEdit::textChanged, this, &EcGuiNet::OnPasswordChanged);
    connect(_password, &QLineEdit::returnPressed, this, &EcGuiNet::OnPasswordEntered);
    
    _server_pwd= _password->text();
    
    connect(_net_tree_wid, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),this, SLOT(OnMouseClicked(QTreeWidgetItem*, int)));


    _ec_master_process = new QProcess(this);
    _ec_master_process->setReadChannel(QProcess::StandardOutput);
    _ec_master_process->setProcessChannelMode(QProcess::MergedChannels);
    _ec_master_process->setCurrentReadChannel(QProcess::StandardOutput);
    
    connect(_ec_master_process, &QProcess::readyReadStandardOutput,
            this, &EcGuiNet::on_ec_process_readyReadStandardOutput);
    
    _server_process = new QProcess(this);
    _server_process->setReadChannel(QProcess::StandardOutput);
    _server_process->setProcessChannelMode(QProcess::MergedChannels);
    _server_process->setCurrentReadChannel(QProcess::StandardOutput);
    
    connect(_server_process, &QProcess::readyReadStandardOutput,
            this, &EcGuiNet::on_server_process_readyReadStandardOutput);
    
    /*protocl */

    _protocol_combobox = parent->findChild<QComboBox *>("Protocol");
    _protocol_combobox->setCurrentIndex(0);
    _server_protocol = _protocol_combobox->currentText();
    
    /* connection of frequency function */
    connect(_protocol_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(OnProtocolChanged())
    );
    
    _ec_master_terminal=std::make_shared<EcGuiTerminal>();
    _ec_master_terminal->setWindowTitle("EtherCAT Master terminal");
    _ec_master_terminal->setAttribute( Qt::WA_QuitOnClose, false );
    
    _server_terminal=std::make_shared<EcGuiTerminal>();
    _server_terminal->setWindowTitle("Server terminal");
    _server_terminal->setAttribute( Qt::WA_QuitOnClose, false );
}

void EcGuiNet::OnPasswordEntered()
{
    _password->setEchoMode(QLineEdit::Password);
}

void EcGuiNet::OnPasswordChanged()
{
    _password->setEchoMode(QLineEdit::PasswordEchoOnEdit);
    _server_pwd= _password->text();
}

void EcGuiNet::OnProtocolChanged()
{
    _server_protocol = _protocol_combobox->currentText();
}

void EcGuiNet::set_ec_network()
{
    _server_hostname=_net_tree_wid->topLevelItem(0)->child(1)->text(1);
    _net_tree_wid->topLevelItem(0)->child(2)->setText(HOSTNAME_COL,_server_hostname);
    
    _net_tree_wid->topLevelItem(0)->child(2)->setText(HOSTIP_COL,_net_tree_wid->topLevelItem(0)->child(1)->text(2));
    if(_net_tree_wid->topLevelItem(0)->child(1)->text(2)=="localhost")
    {
        _server_ip="127.0.0.1";
    }
    else
    {
        _server_ip=_net_tree_wid->topLevelItem(0)->child(1)->text(2);
    }
    
    _server_port=_net_tree_wid->topLevelItem(0)->child(1)->text(3);
    _net_tree_wid->topLevelItem(0)->child(0)->setText(HOSTPORT_COL,_server_port);
    
    _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    _net_item = nullptr;
    _net_column=-1;
}

bool EcGuiNet::eventFilter( QObject* o, QEvent* e )
{
    if( o == _net_tree_wid && e->type() == QEvent::KeyRelease)
    {
        QKeyEvent *qkey = static_cast<QKeyEvent*>(e);
        if(qkey->key() == Qt::Key_Return)
        {
            set_ec_network();
        }
    }
    return false;
}

void EcGuiNet::OnMouseDoubleClicked(QTreeWidgetItem* item, int column)
{
    if(column == 4 && item->text(0)=="Server")
    {
        _server_terminal->show();
        _server_terminal->setWindowState(Qt::WindowActive);
    }
    else if(column == 4 && item->text(0)=="EtherCAT Master")
    {
        _ec_master_terminal->show();
        _ec_master_terminal->setWindowState(Qt::WindowActive);
    }
}

void EcGuiNet::OnMouseClicked(QTreeWidgetItem* item, int column)
{
    if(_net_item != nullptr)
    {
        _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    }

    _net_item = item;
    _net_column=column;

    if((item->text(0)=="Server") &&
       ((column == HOSTNAME_COL) || 
        (column == HOSTIP_COL)   ||
        (column == HOSTPORT_COL)))    
    {
        _net_tree_wid->openPersistentEditor(item,column);
    }
    else
    {
        _net_tree_wid->closePersistentEditor(item,column);
    } 
}


void EcGuiNet::on_ec_process_readyReadStandardOutput()
{
   _ec_master_stoud.clear();
   while(_ec_master_process->canReadLine()){
       _ec_master_stoud = _ec_master_process->readLine();
       _ec_master_terminal->setText(_ec_master_stoud);
  }
}

void EcGuiNet::on_server_process_readyReadStandardOutput()
{
   _server_stdout.clear();
   while(_server_process->canReadLine()){
       _server_stdout = _server_process->readLine();
       _server_terminal->setText(_server_stdout);
  }
}

QString EcGuiNet::find_running_process(QProcess * process,QString bin_name,QString& stdout)
{
    QStringList cmd;
    QString bin_pid="";
    
    cmd = _ssh_command;
    cmd.append("'pgrep'"); // remember comment out: .bashrc all line of # If not running interactively, don't do anything
    cmd.append(bin_name);
    
    stdout.clear();
    process->start("sshpass", cmd);
    if(process->waitForFinished())
    {
       bin_pid = stdout;
    }
    bin_pid = bin_pid.remove(QChar('\n'));

    return bin_pid;
}


QString EcGuiNet::find_process(QProcess * process,QString bin_name,QString& stdout)
{
    QStringList cmd;
    QString bin_file_path="";
    
    cmd = _ssh_command;
    cmd.append("'which'"); // remember comment out: .bashrc all line of # If not running interactively, don't do anything
    cmd.append(bin_name);
    
    stdout.clear();
    process->start("sshpass", cmd);
    if(process->waitForFinished())
    {
       bin_file_path = stdout;
    }
    
    bin_file_path = bin_file_path.remove(QChar('\n'));

    return bin_file_path;
}

void EcGuiNet::kill_process(QProcess *process,QString bin_name,QString& stdout)
{
    QString pid=find_running_process(process,bin_name,stdout);
    if(pid!="")
    {
        QStringList cmd;
        
        cmd=_ssh_command;
        cmd.append("'killall'");
        cmd.append(bin_name);
        
        process->start("sshpass", cmd);
        process->waitForFinished();
    }
}

void EcGuiNet::start_process(QProcess *process,QString bin_file_path,QString option)
{
    QStringList cmd;
    
    cmd=_ssh_command;  
    cmd.append(bin_file_path);
    if(option!="")
    {
        cmd.append(option);
    }
    process->start("sshpass", cmd);
}

bool EcGuiNet::create_ssh_cmd(QProcess *process)
{
    _ssh_command.clear();
    _ssh_command.append("-p");

    _ssh_command.append(_server_pwd);
    _ssh_command.append("ssh");
    _ssh_command.append("-o StrictHostKeyChecking=no");
    _ssh_command.append(_server_hostname+"@"+_server_ip);

    QStringList cmd = _ssh_command;
    cmd.append("'whoami'");
    process->start("sshpass", cmd);
    process->waitForFinished();
    
    auto host_name = _server_stdout;
    host_name = host_name.remove(QChar('\n'));
    
    if(host_name != _server_hostname)
    {
        QMessageBox msgBox;
        msgBox.setText("Problem on the ssh command, please verify the EtherCAT system setup");
        msgBox.exec();
        return false;
    }

    return true;
}

bool EcGuiNet::start_network()
{
    if(!create_ssh_cmd(_server_process)){
        return false;
    }

    QString bin_file_name = "'repl'";
    kill_process(_ec_master_process,bin_file_name,_ec_master_stoud);
    auto bin_file_path = find_process(_ec_master_process,bin_file_name,_ec_master_stoud);

    /******************************START EtherCAT Master ************************************************/
    if(!bin_file_path.isEmpty()){
        QString option="";
        if(_server_protocol=="udp"){
            option="-f ~/.ecat_master/configs/zipc_config.yaml";  
        }
        
        start_process(_ec_master_process,bin_file_path,option);
    }
    
    /******************************START SEVER ************************************************/
    if(_server_protocol=="udp") {
        bin_file_name = "'udp_server'";
        kill_process(_server_process,bin_file_name,_server_stdout);
        
        bin_file_path.clear();
        bin_file_path = find_process(_server_process,bin_file_name,_server_stdout);

        if(!bin_file_path.isEmpty()){
            start_process(_server_process,bin_file_path,"");
        }
    }
    
    return true;
}

void EcGuiNet::stop_network()
{
    QString bin_file_name;
    /******************************STOP Server ************************************************/
    if(_server_protocol=="udp")
    {
        bin_file_name = "'udp_server'";
        _server_process->close();
        kill_process(_server_process,bin_file_name,_server_stdout);
    }
    
    /******************************STOP EtherCAT Master ************************************************/
    _ec_master_process->close();
    bin_file_name = "'repl'";
    kill_process(_ec_master_process,bin_file_name,_ec_master_stoud);
}

bool EcGuiNet::check_network()
{
    bool ret= false;
    if(_server_protocol=="udp"){
        if(!create_ssh_cmd(_server_process))
        {
            return ret;
        }
        
        QString pid=find_running_process(_server_process,"'udp_server'",_server_stdout);
        if(pid!="")
        {
            ret=true;
        }
    }
    else{
        ret=true;
    }
    
    return ret;
}

EcGuiNet::ec_net_info_t EcGuiNet::get_net_setup()
{
    ec_net_info_t ec_net_info;

    ec_net_info.protocol=_server_protocol.toStdString();
    ec_net_info.host_name=_server_ip.toStdString();
    ec_net_info.host_port=_server_port.toUInt();
    
    return ec_net_info;
}

EcGuiNet::~EcGuiNet()
{
    if(_server_protocol=="udp"){
        _server_process->kill();
    }
    
    _ec_master_process->kill();
}
