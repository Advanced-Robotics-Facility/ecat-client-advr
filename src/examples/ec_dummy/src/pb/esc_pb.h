#ifndef __ESC_PB__
#define __ESC_PB__

class EscPb{

public:
   
    virtual ~EscPb () {
    }
    
    
    virtual void pbDeserialize(iit::advr::Ec_slave_pdo pb) = 0;
    
    virtual void pbSerialize(iit::advr::Ec_slave_pdo& pb)   = 0;
};

#endif
