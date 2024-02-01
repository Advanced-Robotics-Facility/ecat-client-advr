#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include <atomic>

#include "utils/ec_utils.h"
#include <test_common.h>

using namespace std::chrono;

static bool run_loop = true;
// /* signal handler*/
static void sig_handler(int sig) 
{
    printf("got signal %d\n", sig);
    switch (sig)
    {
    case SIGALRM:
        run_loop = false;
        break;
    
    default:
        run_loop = false;
        break;
    }
}

// IDLE-->Connected->Autodetection->Motor_Started->Motor_Ctrl->Engage_Motor_Brake->Motor_Stopping->Exit
                                                                    
int main(int argc, char * const argv[])
{    
    EcUtils::Ptr ec_client_utils;
    EcUtils::EC_CONFIG ec_client_cfg;

    try{
        ec_client_utils=std::make_shared<EcUtils>();
        ec_client_cfg = ec_client_utils->get_ec_cfg();
    }catch(std::exception &ex){
        DPRINTF("Error on ec client config file\n");
        DPRINTF("%s\n",ex.what());
    return 1;
    }
    
        
    std::vector<int> slave_id_vector;
    for ( auto &[id, pos] : ec_client_cfg.homing_position ) {
        slave_id_vector.push_back(id);
    }
    
    if(slave_id_vector.empty()){
        DPRINTF("Got an homing position map\n");
        return 0;
    }
    std::string STM_sts="IDLE";
    // *************** START CLIENT  *************** //
    EcIface::Ptr client=ec_client_utils->make_ec_iface();
    STM_sts="Connected";
    // *************** START CLIENT  *************** //
                
    
    // *************** AUTODETECTION *************** //
    
    MST motors_start = {};
    PAC release_brake_cmds = {};
    PAC engage_brake_cmds = {};
    
    SSI slave_info;
    
    if(client->retrieve_slaves_info(slave_info))
    {   
        if(!slave_info.empty())
        {
            DPRINTF("AUTODETECTION\n");
            for(int motor_id_index=0;motor_id_index<slave_id_vector.size();motor_id_index++)
            {
                bool motor_found=false;
                for ( auto &[id, type, pos] : slave_info ) {
                    if((type==CENT_AC) || (type==LO_PWR_DC_MC))//HP or LP motor
                    {
                        if(id == ec_client_cfg.motor_id[motor_id_index])
                        {
                            motor_found=true;
                            break;
                        }
                    }
                }
            
                if(!motor_found)
                {
                    throw std::runtime_error("ID: " + std::to_string(ec_client_cfg.motor_id[motor_id_index]) + "not found");
                }
            }

            STM_sts="Autodetection";
        }
    }
    
    for(int i=0; i< slave_id_vector.size();i++)
    {
        int id = slave_id_vector[i];
        motors_start.push_back(std::make_tuple(id,ec_client_cfg.motor_config_map[id].control_mode_type,ec_client_cfg.motor_config_map[id].gains));
        
        if(ec_client_cfg.motor_config_map[id].brake_present){
            // queue release/engage brake commands for all motors 
            release_brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_RELEASE)));
            engage_brake_cmds.push_back(std::make_tuple(id,to_underlying(PdoAuxCmdType::BRAKE_ENGAGE)));
        }
    }
        
    // *************** AUTODETECTION *************** //

    bool stop_motors=false;
    bool motor_started=false;
    const int max_pdo_aux_cmd_attemps=3; // 3 times to release/engage or LED ON/OFF command
    int pdo_aux_cmd_attemps=0; // 3 times to release/engage or LED ON/OFF command

    if(!motors_start.empty())
    {
        // ************************* START Motors ***********************************//
        DPRINTF("START ALL MOTORS\n");
        motor_started=client->start_motors(motors_start);

        if(motor_started)
        {
            if(!release_brake_cmds.empty()){
                STM_sts="Motor_Started";
                // ************************* RELEASE BRAKES ***********************************//
                while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
                {
                    pdo_aux_cmd_attemps++;
                    if(!client->pdo_aux_cmd(release_brake_cmds))
                    {
                        DPRINTF("Cannot perform the release brake command of the motors\n");
                        pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                    }
                    else
                    {
                        std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                        if(client->pdo_aux_cmd_sts(release_brake_cmds))
                        {
                            pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                            STM_sts="Motor_Ctrl";
                        }
                    }
                }
            }
            else{
                STM_sts="Motor_Ctrl";
            }
            // ************************* RELEASE BRAKES ***********************************//
        }
        else
        {
            DPRINTF("Motors not started\n");
        }
        // ************************* START Motors ***********************************//
    }
    else
    {
        DPRINTF("NO MOTORS STARTED\n");
    }

#ifdef TEST_EXAMPLES
        if(STM_sts=="Connected")
        {   
            STM_sts="Motor_Ctrl";
        }
#endif
            
    if(STM_sts=="Motor_Ctrl")
    {
        struct timespec ts= { 0, ec_client_cfg.period_ms*1000000}; //sample time
        
        uint64_t start_time_ns = iit::ecat::get_time_ns();
        uint64_t time_ns=start_time_ns;
        
        double time_elapsed_ms;
        double incrementat_freq_ns=0;
    
        bool first_Rx=false;
        
        std::map<int,double> q_set_trj=ec_client_cfg.homing_position;
        std::map<int,double> q_ref,qdot;
        
        // Power Board
        PwrStatusMap pow_status_map;
        float v_batt,v_load,i_load,temp_pcb,temp_heatsink,temp_batt;
        
        // Motor
        float  link_pos,motor_pos,link_vel,motor_vel,torque,aux;
        float  motor_temp, board_temp;
        uint32_t fault,rtt,op_idx_ack;
        uint32_t cmd_aux_sts,brake_sts,led_sts;
        MotorStatusMap motors_status_map;
        std::vector<MR> motors_ref;
        int motor_ref_index=0;
        
        
        // memory allocation
        client->get_pow_status(pow_status_map);
        client->get_motors_status(motors_status_map);
        
        for ( const auto &[esc_id, motor_status] : motors_status_map){
            q_ref[esc_id]= std::get<1>(motor_status); //motor pos
            qdot[esc_id] = std::get<3>(motor_status); //motor vel
        }
        
#ifdef TEST_EXAMPLES
        if(!first_Rx){
            for(int i=0; i<slave_id_vector.size();i++){
                int id=slave_id_vector[i];
                q_ref[id]= 0.0;
                qdot[id]=  0.0;
            }
        }
#endif
        if(q_ref.size() == q_set_trj.size()){
            //Open Loop SENSE
            first_Rx=true;
        }
        else{
            throw std::runtime_error("fatal error: different size of initial position and trajectory vectors");
        }
        
        
        for ( const auto &[esc_id, pos_ref] : q_ref){
           motors_ref.push_back(std::make_tuple(esc_id, //bId
                                                ec_client_cfg.motor_config_map[esc_id].control_mode_type, //ctrl_type
                                                pos_ref, //pos_ref
                                                0.0, //vel_ref
                                                0.0, //tor_ref
                                                ec_client_cfg.motor_config_map[esc_id].gains[0], //gain_1
                                                ec_client_cfg.motor_config_map[esc_id].gains[1], //gain_2
                                                ec_client_cfg.motor_config_map[esc_id].gains[2], //gain_3
                                                ec_client_cfg.motor_config_map[esc_id].gains[3], //gain_4
                                                ec_client_cfg.motor_config_map[esc_id].gains[4], //gain_5
                                                1, // op means NO_OP
                                                0, // idx
                                                0  // aux
                                            ));
        }
        
        if(motors_ref.empty()){
            throw std::runtime_error("fatal error: motors references structure empty!");
        }
        
        if(motors_ref.size() != q_set_trj.size()){
            throw std::runtime_error("fatal error: different size of reference and trajectory vectors");
        }
        // memory allocation
    
        if(ec_client_cfg.protocol=="iddp"){
            DPRINTF("Real-time process....\n");
            // add SIGALRM
            signal ( SIGALRM, sig_handler );
            main_common (&argc, (char*const**)&argv, sig_handler);
            assert(set_main_sched_policy(10) >= 0);
        }
        else{
            struct sigaction sa;
            sa.sa_handler = sig_handler;
            sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
            sigaction(SIGINT,&sa, nullptr);
        }

        while (run_loop && client->is_client_alive())
        {
            time_elapsed_ms= (time_ns-start_time_ns)/1000000;
            //DPRINTF("Time [%f]\n",time_elapsed_ms);
            
            // Rx "SENSE"
            //******************* Power Board Telemetry ********
            client->get_pow_status(pow_status_map);
            for ( const auto &[esc_id, pow_status] : pow_status_map){
                v_batt =        pow_status[0];
                v_load =        pow_status[1];
                i_load =        pow_status[2];
                temp_pcb =      pow_status[3];
                temp_heatsink=  pow_status[4];
                temp_batt=      pow_status[5];
            }
            //******************* Power Board Telemetry ********
            
            //******************* Motor Telemetry **************
            client->get_motors_status(motors_status_map);
            for ( const auto &[esc_id, motor_status] : motors_status_map){
                try{
                    std::tie(link_pos,motor_pos,link_vel,motor_vel,torque,motor_temp,board_temp,fault,rtt,op_idx_ack,aux,cmd_aux_sts) = motor_status;
                    
                    // PRINT OUT Brakes and LED get_motors_status @ NOTE To be tested.         
                    brake_sts = cmd_aux_sts & 3; //00 unknown
                                                //01 release brake 
                                                //10 enganged brake  
                                                //11 error
                    led_sts= (cmd_aux_sts & 4)/4; // 1 or 0 LED  ON/OFF
                    
                    //Closed Loop SENSE for motor velocity
                    qdot[esc_id] = motor_vel;
                    
                    
                    //Closed Loop SENSE for motor position and velocity
                    
                    if(!first_Rx)
                    {
                        q_ref[esc_id]=motor_pos;
                    }
                    
                } catch (std::out_of_range oor) {}
            }
            
            //******************* Motor Telemetry **************

            // ************************* SEND ALWAYS REFERENCES***********************************//
            motor_ref_index=0;
            for ( const auto &[esc_id, pos_ref] : q_ref){
                std::get<2>(motors_ref[motor_ref_index]) = pos_ref;
                motor_ref_index++;
            }
            
            // Tx "MOVE"  @NOTE: motors_ref done when the state machine switch between homing and trajectory after motor_ref will remain equal to old references 
            if(!motors_ref.empty())
            {
                client->set_motors_references(MotorRefFlags::FLAG_MULTI_REF, motors_ref);
            }
            else
            {
                throw std::runtime_error("fatal error: motors references structure empty!");
            }
            // ************************* SEND ALWAYS REFERENCES***********************************//

            
            // get period ns
            time_ns = iit::ecat::get_time_ns();
            
            if(time_elapsed_ms>=1000) //1s
            {
                incrementat_freq_ns=incrementat_freq_ns+100000; //(100 us) every 1s
                start_time_ns=time_ns;
            }
            
            ts.tv_nsec=ts.tv_nsec+incrementat_freq_ns;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL); 
        }
            
    }
    
    // ************************* ENGAGE BRAKES ***********************************//
    if(!engage_brake_cmds.empty()){
        pdo_aux_cmd_attemps=0;
        while(pdo_aux_cmd_attemps<max_pdo_aux_cmd_attemps)
        {
            pdo_aux_cmd_attemps++;
            if(!client->pdo_aux_cmd(engage_brake_cmds))
            {
                DPRINTF("Cannot perform the engage brake command of the motors\n");
                pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
            }
            else
            {
                std::this_thread::sleep_for(1000ms); //wait 1s to check if the brakes are released
                if(client->pdo_aux_cmd_sts(engage_brake_cmds))
                {
                    pdo_aux_cmd_attemps=max_pdo_aux_cmd_attemps;
                    STM_sts="Motor_Stopping";
                }
            }
        }
    }
    else{
        STM_sts="Motor_Stopping";
    }
    // ************************* ENGAGE BRAKES ***********************************//


    // ************************* STOP Motors ***********************************//
    if(STM_sts =="Motor_Stopping")
    {
        if(!client->stop_motors())
        {
            DPRINTF("Not all motors are stopped\n");
        }
        else
        {
            DPRINTF("All Motors stopped\n");
        }
        STM_sts = "Exit";
    }
    // ************************* STOP Motors ***********************************//
    
    // STOP CLIENT
    if(client->is_client_alive())
    {
        client->stop_client();
    }
    return 0;
}
