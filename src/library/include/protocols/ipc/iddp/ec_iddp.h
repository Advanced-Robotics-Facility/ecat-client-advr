#ifndef EC_IDDP_H
#define EC_IDDP_H

#include <thread_util.h>
#include "protocols/common/ec_cmd.h"
#include "protocols/common/ec_pdo.h"

class EcIDDP : public EcCmd,Thread_hook
{
public:

    EcIDDP(std::string host_address,uint32_t host_port);
    ~EcIDDP();

    void start_client(uint32_t period_ms,bool logging) final;
    void stop_client() final ;
    void set_loop_time(uint32_t period_ms) final;
    bool pdo_aux_cmd_sts(const PAC & pac) final;

public:
    
    virtual void th_init ( void * );    
    virtual void th_loop ( void * );
    
private:
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;
    
private:
    std::shared_ptr<EcPdo<EcPipePdo>> _ec_pdo;
    
};

#endif // EC_IDDP_H
