#ifndef EC_TCP_H
#define EC_TCP_H

#include "utils/ec_thread.h"
#include "mechanism/zmq/ec_zmq_cmd.h"
#include "mechanism/common/ec_pdo.h"

class EcTCP : public EcZmqCmd,EcPdo<EcZmqPdo>,EcThread
{
public:

    EcTCP(std::string host_address,uint32_t host_port);
    ~EcTCP();

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
};

#endif // EC_TCP_H
