#ifndef __MOTOR_SETUP__
#define __MOTOR_SETUP__

#include <yaml-cpp/yaml.h>


class Motor_Setup
{
public:
    
    typedef std::shared_ptr<Motor_Setup> Ptr;
    
    /**
    * @brief Constructor of Motor Setup Class
    * It needs of motor configuration file, low level id, and slave name
    * 
    * 
    * @param _motor_cfg p__motor_cfg: Motor configuration file for that specific motor.
    * @param id_low_level p_id_low_level: low level id.
    * @param name p_name: Motor Name.
    */
    Motor_Setup(const YAML::Node & _motor_cfg,int id_low_level,std::string name);
    
    /**
    * @brief Destructor of Motor Setup Class
    * 
    */
    ~Motor_Setup();
    
    /**
    * @brief Structure for the trajectory, Every motor can one homing, periodic and smooth trajectory.
    * 
    */
    struct trajectoy_t{
        
    std::string trj_name;     
   
    std::vector<float> homing_x;
    
    float period_freq;
    float period_ampl;
    float period_teta;
    float period_secs;
        
    
    std::vector<float> smooth_x;
    std::vector<float> smooth_y;
    
    };

    /******* CALIBRATION *///////////////////

    /**
    * @brief Set maximum current.
    * 
    * @param max_current_A p_max_current_A: Maximum current
    */
    void set_max_current_A(float max_current_A);
    
    /**
    * @brief Set minimum position.
    * 
    * @param min_pos p_min_pos: Minimum position.
    */
    void set_min_pos(float min_pos);
    
    /**
    * @brief Set maximum position.
    * 
    * @param max_pos p_max_pos: Maximum position.
    */
    void set_max_pos(float max_pos);
    
    /**
    * @brief Set maximum velocity.
    * 
    * @param max_vel p_max_vel: Maximum velocity.
    */
    void set_max_vel(float max_vel);
    
    /**
    * @brief Set maximum torque.
    * 
    * @param max_torq p_max_torq: Maximum torque.
    */
    void set_max_torq(float max_torq);
    
    /**
    * @brief Set c28 filename and password.
    * 
    * @param c28_firm p_c28_firm:...
    * @param c28_firm_password p_c28_firm_password:...
    */
    void set_c28_firm_info(std::string c28_firm,std::string c28_firm_password);
    
    /**
    * @brief Set m3 filename and password.
    * 
    * @param m3_firm p_m3_firm:...
    * @param m3_firm_password p_m3_firm_password:...
    */
    void set_m3_firm_info(std::string m3_firm,std::string m3_firm_password);

    /******* COMMAND *///////////////////
    
    /**
    * @brief Set control mode, (position,velocity and torque ).
    * 
    * @param control_mode p_control_mode: control mode.
    */
    void set_control_mode(std::string control_mode);
    
    /**
    * @brief Set position gains.
    * 
    * @param position_gains p_position_gains: Position gains.
    */
    void set_position_gains(std::vector<float> position_gains);   // NOTE: USED FOR CALIBRATION TOO.
    
    /**
    * @brief Set velocity gains.
    * 
    * @param velocity_gains p_velocity_gains: Velocity gains.
    */
    void set_velocity_gains(std::vector<float> velocity_gains);   // NOTE: USED FOR CALIBRATION TOO.
    
    /**
    * @brief Set impedance gains.
    * 
    * @param impedance_gains p_impedance_gains: Impedance Gains.
    */
    void set_impedance_gains(std::vector<float> impedance_gains); // NOTE: USED FOR CALIBRATION TOO.
    
    /**
    * @brief Set data object type, PDO or SDO.
    * 
    * @param data_object_type p_data_object_type:...
    */
    void set_data_object_type(std::string data_object_type);  // NOTE: USED to SWITCH PDO/SDO.
    
    /**
    * @brief Set actual position.
    * 
    * @param actual_position p_actual_position: Actual position.
    */
    void set_actual_position(float actual_position);
    
    /**
    * @brief Set actual velocity.
    * 
    * @param actual_velocity p_actual_velocity: Actual velocity.
    */
    void set_actual_velocity(float actual_velocity);
    
    /**
    * @brief Set actual torque.
    * 
    * @param actual_torque p_actual_torque: Actual torque.
    */
    void set_actual_torque(float actual_torque);
    
    /**
    * @brief Set actual amperage
    * 
    * @param actual_amperage p_actual_amperage: Actual amperage.
    */
    void set_actual_amperage(float actual_amperage);
    
    /**
    * @brief Set homing position.
    * 
    * @param homing_position p_homing_position: Homing position.
    */
    void set_homing_position(float homing_position);
    
    /**
    * @brief Set fan ON/OFF.
    * 
    * @param fan_onoff p_fan_onoff: FAN ON/OFF
    */
    void set_fan_onoff(bool fan_onoff);
    
    /**
    * @brief Set led ON/OFF.
    * 
    * @param led_onoff p_led_onoff: LED ON/OFF
    */
    void set_led_onoff(bool led_onoff);
    
    /******* TRAJECTORY *///////////////////
    
    /**
    * @brief Set trajectory structure.
    * 
    * @param trajectoy_s p_trajectoy_s: Trajectory structure.
    */
    void set_trajectory_structure(trajectoy_t trajectoy_s); 
    
    /******* GENERAL INFO *///////////////////
    /**
    * @brief Get motor id low level.
    * 
    * @return int
    */
    int get_id_low_level();
    
    /**
    * @brief Get motor sign.
    * 
    * @return int
    */
    int get_sign();   
    
    /**
    * @brief Get motor offset.
    * 
    * @return float
    */
    float get_pos_offset();
    
    /**
    * @brief Get motor name.
    * 
    * @return std::__cxx11::string
    */
    std::string get_name();
    
    /******* CALIBRATION *///////////////////
    /**
    * @brief Get maximum current of the motor.
    * 
    * @return float
    */
    float get_max_current_A();
    
    /**
    * @brief Get minimum position of the motor.
    * 
    * @return float
    */
    float get_min_pos();
    
    /**
    * @brief Get maximum position of the motor.
    * 
    * @return float
    */
    float get_max_pos();
    
    /**
    * @brief Get maximum velocity of the motor.
    * 
    * @return float
    */
    float get_max_vel();
    
    /**
    * @brief Get maximum torque of the motor.
    * 
    * @return float
    */
    float get_max_torq();
    
    /**
    * @brief Get c28 file name and password.
    * 
    * @param c28_firm p_c28_firm: c28 filename.
    * @param c28_firm_password p_c28_firm_password: c28 password.
    */
    void get_c28_firm(std::string &c28_firm,std::string &c28_firm_password);
    
    /**
    * @brief Get m3 file name and password.
    * 
    * @param m3_firm p_m3_firm: m3 filename.
    * @param m3_firm_password p_m3_firm_password: m3 password.
    */
    void get_m3_firm(std::string &m3_firm,std::string &m3_firm_password);
    
    /******* COMMAMD *///////////////////
    
    /**
    * @brief Get control mode (position, velocity and torque mode).
    * 
    * @return std::__cxx11::string
    */
    std::string get_control_mode();
    
    /**
    * @brief Get brake prensence.
    * 
    * @return bool
    */
    bool get_brake_present();

    /**
    * @brief Get position gains.
    * 
    * @return std::vector< float >
    */
    std::vector<float> get_position_gains();    // NOTE: USED FOR CALIBRATION TOO.
    
    /**
    * @brief Get velocity gains.
    * 
    * @return std::vector< float >
    */
    std::vector<float> get_velocity_gains();    // NOTE: USED FOR CALIBRATION TOO.
    
    /**
    * @brief Get impedance gains.
    * 
    * @return std::vector< float >
    */
    std::vector<float> get_impedance_gains();   // NOTE: USED FOR CALIBRATION TOO.
     
    /**
    * @brief Get data object type, PDO/SDO
    * 
    * @return std::__cxx11::string
    */
    std::string get_data_object_type();      // NOTE: USED to SWITCH PDO/SDO.
    
    /**
    * @brief  Get actual position.
    * 
    * @return float
    */
    float get_actual_position();
    
    /**
    * @brief Get actual velocity.
    * 
    * @return float
    */
    float get_actual_velocity();
    
    /**
    * @brief Get actual torque.
    * 
    * @return float
    */
    float get_actual_torque();
    
    /**
    * @brief Get actual amperage.
    * 
    * @return float
    */
    float get_actual_amperage();
    
    /**
    * @brief Get homing position.
    * 
    * @return float
    */
    float get_homing_position();
    
    /**
    * @brief Get fan ON/OFF 
    * 
    * @return bool
    */
    bool get_fan_onoff();
    
    /**
    * @brief Get led ON/OFF
    * 
    * @return bool
    */
    bool get_led_onoff();
    
    /******* TRAJECTORY *///////////////////
    
    /**
    * @brief Get trajectory structure-
    * 
    * @return Motor_Setup::trajectoy_t
    */
    trajectoy_t get_trajectory_structure(); 
    
private:
    
  trajectoy_t _trajectoy_s;
    
  int _id_low_level; 
  std::string _name;
  std::string _control_mode;
  bool _brake_present;
  int _sign;
  float _max_current_A,_pos_offset,_actual_amperage;
  float _min_pos,_max_pos,_max_vel,_max_torq;
  std::vector<float> _position_gains,_impedance_gains,_velocity_gains;
  std::string _c28_firm,_c28_firm_password,_m3_firm,_m3_firm_password;
  std::string _data_object_type;
  
  float _actual_position,_actual_velocity,_actual_torque;
  float _homing_position;
  
  bool _led_onoff, _fan_onoff;
  
};

#endif
