#ifndef EC_BOOST_PDO_H
#define EC_BOOST_PDO_H

#include "ec_iface.h"
#include "ec_boost.h"

class EcBoostPdo : virtual public EcBoost,virtual public EcIface
{
public:
    void esc_factory(SSI slave_descr);
    
protected:
    ServerStatus _actual_server_status;
private:  
    // MSG HANDLERS
    void server_status_handler(char *buf, size_t size);
    void motor_status_handler(char*buf, size_t size);
    void ft6_status_handler(char*buf, size_t size);
    void pwr_status_handler(char*buf, size_t size);
};

#endif
