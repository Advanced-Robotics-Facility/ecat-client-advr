#include "rt_thread.h"
#include <esc_pipe_iface.h>

#include <functional>
#include <numeric>
#include "../command/ec_client_cmd.h"
#include <boost/asio.hpp>
#include <yaml-cpp/yaml.h>
#include "data_cb_handler.h"

const std::map<uint32_t,const std::string> esc_type_map = {

    { 0x12, "Motor"},
    { 0x15, "Motor"},
    { 0x16, "Heri3"},
    { 0x17, "Motor"},
    { 0x24, "Ft"},
    { 0x25, "Ain"},
    { 0x32, "PowBoard"},
    
    { 0x80000001, "Test"},
    { 0x80000002, "Test"},
    { 0x80000003, "Test"},
    { 0x80000004, "Test"},
    { 0x80000005, "Test"},

};

static DataHandlerTableType cbTable = {
    { iit::ecat::HYQ_IOV5,          Default_raw_handle<iit::ecat::HyQ_IOv5EscPdoTypes> },
    //
    { iit::ecat::CENT_AC,           Default_pb_handle },
    { iit::ecat::FT6_MSP432,        Default_pb_handle },
    { iit::ecat::POW_F28M36_BOARD,  Default_pb_handle },
    //
    { iit::ecat::MSP432_TEST,       Default_pb_handle },
};


////////////////////////////////////////////////////
// 
// 
//
////////////////////////////////////////////////////
    
uint32_t RT_motor_thread::pbSet(uint32_t idx)
{
    pb_repl_cmd.set_type(iit::advr::CmdType::CTRL_CMD);
    set_pbHeader(pb_repl_cmd.mutable_header(), name, idx);
    return pb_repl_cmd.ByteSize();        
}


RT_motor_thread::RT_motor_thread(std::string _name,
                                 std::string rt_thread_start,
                                 std::string rd,
                                 std::string wr,
                                 uint32_t th_period_us ) :
rt_thread_start(rt_thread_start),
rd_pipe(rd),
wr_pipe(wr)
{
    name = _name;
    // periodic
    struct timespec ts;
    iit::ecat::us2ts(&ts, th_period_us);
    // period.period is a timeval ... tv_usec 
    period.period = { ts.tv_sec, ts.tv_nsec / 1000 };
#ifdef __COBALT__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max ( schedpolicy ) / 2;
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
    
    // zmq setup
    _zmq_uri = "tcp://" + boost::asio::ip::host_name() + ":5555";
    _timeout_ms = 1000;  // 1 secs
}

RT_motor_thread::~RT_motor_thread()
{ 
    for (auto const &[id,esc_iface] : escs_iface )  {
        esc_iface->write_quit();
    }

    for ( auto& [id, iface] : escs_iface ) {
        iface.reset();
    }
    escs_iface.clear();
    
    iit::ecat::print_stat ( s_loop );
}

void RT_motor_thread::th_init ( void * )
{
    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;
    loop_cnt = 0;
    
    pbSet(0);
    pb_repl_cmd.Clear();
    
    DPRINTF ("trying to perform: get_slaves_description\n");

    std::string slave_descr_info;

    std::map<std::string,std::string> args;
    
    EC_Client_CMD ec_client_cmd(_zmq_uri,_timeout_ms);
    ec_client_cmd.Ecat_Master_cmd(iit::advr::Ecat_Master_cmd_Type::Ecat_Master_cmd_Type_GET_SLAVES_DESCR,
                                  args,
                                  slave_descr_info);
    
    auto robot_name = "NoNe";

    if(!slave_descr_info.empty())
    {
        // load slave descr as yaml
        YAML::Node slaves_info = YAML::Load(slave_descr_info);

        // parse it into a map slave id -> slave info map
        auto slave_map = slaves_info.as<std::map<int, std::map<std::string, int>>>();
        for ( auto const item : slave_map ) {
            auto esc_id = item.first;
            auto esc_info = item.second;
            slave_descr.push_back(std::make_tuple(esc_id,esc_info["esc_type"],esc_info["position"]));
        }
    }
    
    for ( auto &[id, type, pos] : slave_descr ) {
        DPRINTF("Esc id %d pos %d 0x%04X\n", id, pos, type);
    }
    
    std::vector<float> gains = {200.0, 10.0};
   
    for ( auto &[id, esc_type, pos] : slave_descr ) {
        try { 
            auto esc_name = esc_type_map.at(esc_type);
            if ( esc_name.find("Motor") || esc_name.find("Test")) {
                EC_Client_CMD start_stop_cmd(_zmq_uri,_timeout_ms);
                std::string msg="";
                if(rt_thread_start!="idle")
                {
                    start_stop_cmd.Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_START,
                                            id,
                                            iit::advr::Gains_Type_POSITION,
                                            gains,
                                            msg);
                }
                else
                {
                    start_stop_cmd.Ctrl_cmd(iit::advr::Ctrl_cmd_Type::Ctrl_cmd_Type_CTRL_CMD_STOP,
                                            id,
                                            0.0, // ignored
                                            {},  // ignored
                                            msg);
                }
            
                // iface_factory will populate escs_iface map
                try { escs_iface[id] = iface_factory(id,esc_type,pos,robot_name); }
                catch ( const EscPipeIfaceError &e) {
                    printf("%s\n", e.what());
                }
            } else {
                DPRINTF("NOT handled Esc id 0x%04X %s\n", id, esc_name.c_str() );
            }
    
        } catch(std::out_of_range) {
                DPRINTF("NOT mapped Esc id 0x%04X pos %d\n", id, pos );
        }
    }
    
    for (auto const &[id,esc_iface] : escs_iface )  {
        try { cbTable.at(esc_iface->get_type())(id, esc_iface,true); }
        catch ( std::out_of_range ) { printf("Azz\n"); };   
    }
}

void RT_motor_thread::th_loop ( void * )
{
    int rd_bytes_from_nrt, rd_bytes_from_rt, wr_bytes_to_rt;
    //int msg_size;
    static FSM_type FSM;
    int read_cnt;
    
    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;
    
    loop_cnt++;
    
    for (auto const &[id,esc_iface] : escs_iface )  {
        try { cbTable.at(esc_iface->get_type())(id, esc_iface,false); }
        catch ( std::out_of_range ) { printf("Azz\n"); };   
    }
    
}

    

