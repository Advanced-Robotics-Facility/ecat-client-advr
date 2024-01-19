#ifndef __EC_RT_TRAJECTORY__
#define __EC_RT_TRAJECTORY__

#include "utils/ec_utils.h"

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
    
public:

    EcRtTrajectory(int period_ms,
                   EcUtils::EC_CONFIG ec_client_cfg,
                   EcIface::Ptr client);
    
    ~EcRtTrajectory();

    virtual void th_init ( void * );    
    virtual void th_loop ( void * );
};

#endif
