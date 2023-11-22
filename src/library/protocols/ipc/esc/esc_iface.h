#ifndef __ESC_PIPE_IFACE__
#define __ESC_PIPE_IFACE__

#include <memory>

#include <pb_utils.h>

class esc_pipe_iface;
class motor_iface;
using SH_IDP = std::shared_ptr<IDDP_pipe>;


////////////////////////////////////////////////////
//
////////////////////////////////////////////////////
enum class EscPipeIfaceErrorNum
{
    NO_ERROR = 0,        // 
    CONNECT_FAIL,
    BIND_FAIL,
    INVALID_TYPE,
    RUNTIME_ERROR,          
};
class EscPipeIfaceError : public std::runtime_error {
using errno_t = std::underlying_type<EscPipeIfaceErrorNum>::type;  
public:
    EscPipeIfaceError (EscPipeIfaceErrorNum err_no, const std::string & what_arg) :
        std::runtime_error(make_what(what_arg,static_cast<errno_t>(err_no))),
        error_num(static_cast<errno_t>(err_no))
    {}

    int get_error_num(void) { return error_num; }

private:
    std::string make_what(std::string what_arg, int err_no) {
        std::ostringstream msg;
        msg << "EscPipeIfaceError " << err_no << " : " << what_arg;
        return msg.str();
    }
    int error_num;
};

////////////////////////////////////////////////////
//
////////////////////////////////////////////////////
class esc_pipe_iface {

protected:

    bool                    initialized;
    std::string             name;
    int32_t                 id;
    uint32_t                type;    
    std::string             rd_pp_name, wr_pp_name;
    IDDP_pipe               rd_iddp, wr_iddp;
    iit::advr::Ec_slave_pdo pb_rx_pdos,  pb_tx_pdos;    
    uint8_t                 pb_buf[MAX_PB_SIZE];

    virtual void get_from_pb(void) = 0;
    virtual void set_to_pb(void) = 0;

public:

    esc_pipe_iface( int32_t id, uint32_t type, const std::string);
    
    esc_pipe_iface( int32_t id, const std::string esc_name, const std::string);

    esc_pipe_iface( int32_t id, uint32_t type,
                    std::string rd_pp_name,
                    std::string wr_pp_name );
    /*                
    esc_pipe_iface( const esc_pipe_iface& rhs ):
        name(rhs.name),id(rhs.id),
        rd_pp_name(rhs.rd_pp_name),
        wr_pp_name(rhs.wr_pp_name) {};
    */

    virtual void init(void);

    iit::advr::Ec_slave_pdo* pb_rx() { return &pb_rx_pdos; }
    iit::advr::Ec_slave_pdo* pb_tx() { return &pb_tx_pdos; }

    const std::string get_name()  { return name; }
    const uint32_t get_type()     { return type; }

    SH_IDP get_rd_iddp() { return std::make_shared<IDDP_pipe>(rd_iddp); }
    SH_IDP get_wr_iddp() { return std::make_shared<IDDP_pipe>(wr_iddp); }

    int read(void);
    int write(void);
    int write_dummy(void);
    int write_connect(void);
    int write_quit(void);
};
#endif
