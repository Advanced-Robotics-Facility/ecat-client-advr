#ifndef EC_ZIPC_H
#define EC_ZIPC_H

#include <thread_util.h>
#include "protocols/common/ec_pdo.h"

class EcZipc : public EcPdo<EcZmqPdo>,Thread_hook
{
public:

    EcZipc(std::string host_address,uint32_t host_port);
    ~EcZipc();

    void start_client(uint32_t period_ms,bool logging) final;
    void stop_client() final ;
    void set_loop_time(uint32_t period_ms) final;
public:
    
    virtual void th_init ( void * );    
    virtual void th_loop ( void * );
    
private:
    iit::ecat::stat_t   s_loop;
    uint64_t            start_time, tNow, tPre;
    uint64_t            loop_cnt;
    bool _create_thread;
};

#endif
