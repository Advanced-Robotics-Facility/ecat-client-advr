#ifndef __EC_PIPE_PDO__
#define __EC_PIPE_PDO__

#include <memory>

#include <pb_utils.h>

class EcPipePdo;
using SH_IDP = std::shared_ptr<IDDP_pipe>;


////////////////////////////////////////////////////
//
////////////////////////////////////////////////////
enum class EcPipePdoErrorNum
{
    NO_ERROR = 0,        // 
    CONNECT_FAIL,
    BIND_FAIL,
    INVALID_TYPE,
    RUNTIME_ERROR,          
};
class EcPipePdoError : public std::runtime_error {
using errno_t = std::underlying_type<EcPipePdoErrorNum>::type;  
public:
    EcPipePdoError (EcPipePdoErrorNum err_no, const std::string & what_arg) :
        std::runtime_error(make_what(what_arg,static_cast<errno_t>(err_no))),
        error_num(static_cast<errno_t>(err_no))
    {}

    int get_error_num(void) { return error_num; }

private:
    std::string make_what(std::string what_arg, int err_no) {
        std::ostringstream msg;
        msg << "EcPipePdoError " << err_no << " : " << what_arg;
        return msg.str();
    }
    int error_num;
};

////////////////////////////////////////////////////
//
////////////////////////////////////////////////////
class EcPipePdo {

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

    EcPipePdo( int32_t id, uint32_t type, const std::string);
    
    EcPipePdo( int32_t id, const std::string esc_name, const std::string);

    EcPipePdo( int32_t id, uint32_t type,
                    std::string rd_pp_name,
                    std::string wr_pp_name );
    /*                
    EcPipePdo( const EcPipePdo& rhs ):
        name(rhs.name),id(rhs.id),
        rd_pp_name(rhs.rd_pp_name),
        wr_pp_name(rhs.wr_pp_name) {};
    */

    virtual ~EcPipePdo () {
    }
    
    virtual void init(void);

    iit::advr::Ec_slave_pdo* pb_rx() { return &pb_rx_pdos; }
    iit::advr::Ec_slave_pdo* pb_tx() { return &pb_tx_pdos; }

    std::string get_name() const    { return name; }
    uint32_t get_type() const       { return type; }

    SH_IDP get_rd_iddp() { return std::make_shared<IDDP_pipe>(rd_iddp); }
    SH_IDP get_wr_iddp() { return std::make_shared<IDDP_pipe>(wr_iddp); }

    int read(void);
    int write(void);
    int write_dummy(void);
    int write_connect(void);
    int write_quit(void);
};
#endif
