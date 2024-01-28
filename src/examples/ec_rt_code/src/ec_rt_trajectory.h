#ifndef __EC_RT_TRAJECTORY__
#define __EC_RT_TRAJECTORY__

#include "utils/ec_utils.h"
#include <chrono>

class EcRtTrajectory : public Thread_hook {

private:
    
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;
    
    EcUtils::EC_CONFIG _ec_client_cfg;
    EcIface::Ptr _client;
    
    MST _motors_start = {};
    PAC _brake_cmds = {};
    SSI _slave_info;
    std::vector<int> _slave_id_vector;
    
    const int _max_pdo_aux_cmd_attemps=3;
    int _pdo_aux_cmd_attemps=0; 

    bool _send_ref=false;
    bool _stop_motors=false;
    
    
    std::chrono::steady_clock _start_time;
    std::chrono::steady_clock _time;
    
    std::chrono::milliseconds _hm_time_ms;
    std::chrono::milliseconds _trj_time_ms;
    std::chrono::milliseconds _time_to_engage_brakes;
    
    bool _first_Rx=false;
    
    std::string _STM_sts;
    std::map<int,double> _q_set_trj;
    std::map<int,double> _q_ref,_q_start,_qdot;

public:

    EcRtTrajectory(int period_ms,
                   EcUtils::EC_CONFIG ec_client_cfg,
                   EcIface::Ptr client);
    
    ~EcRtTrajectory();

    virtual void th_init ( void * );    
    virtual void th_loop ( void * );
};

#endif
