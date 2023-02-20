#include "motor_setup.h"

using namespace std;

Motor_Setup::Motor_Setup(const YAML::Node & _motor_cfg,int id_low_level,string name):
_id_low_level(id_low_level),
_name(name)
{   
    if(_motor_cfg["sign"])
        _sign = _motor_cfg["sign"].as<int>();
    else
        _sign = 0;
    
    if(_motor_cfg["pos_offset"])
        _pos_offset = _motor_cfg["pos_offset"].as<float>();
    else
        _pos_offset = 0.0;
    
    if ( _motor_cfg["max_current_A"] )
        _max_current_A = _motor_cfg["max_current_A"].as<float>();
    else
        _max_current_A = 0.0;
    
    if ( _motor_cfg["brake_present"] ) {
        _brake_present = _motor_cfg["brake_present"].as<bool>();
    }
    else {
        _brake_present = false;
    }
    
    if ( _motor_cfg["control_mode"] )
        _control_mode= _motor_cfg["control_mode"].as<std::string>();
    else
        _control_mode="";
    
    if(_motor_cfg["pid"] && 
        _motor_cfg["pid"]["position"]
        )
        _position_gains = _motor_cfg["pid"]["position"].as<std::vector<float>>();
    
    if((_position_gains.size()!=3)&&(!_position_gains.empty()))
        throw std::runtime_error("Wrong dimension of position gains of slave: "+ name+", the size has to be 3");     
    
    if(_motor_cfg["pid"] && _motor_cfg["pid"]["velocity"])
        _velocity_gains = _motor_cfg["pid"]["velocity"].as<std::vector<float>>();
    
    if((_velocity_gains.size()!=3)&&(!_velocity_gains.empty()))
        throw std::runtime_error("Wrong dimension of velocity gains of slave: "+ name+", the size has to be 3");  

    
    if(_motor_cfg["pid"] && _motor_cfg["pid"]["impedance"])
        _impedance_gains = _motor_cfg["pid"]["impedance"].as<std::vector<float>>();
    
    if((_impedance_gains.size()!=5)&&(!_impedance_gains.empty()))
        throw std::runtime_error("Wrong dimension of impedance gains of slave: "+ name+", the size has to be 5");  
    
    _min_pos=_max_pos=_max_vel=_max_torq=0.0;
    _c28_firm=_m3_firm=_c28_firm_password=_m3_firm_password="";
    _data_object_type="sdo";
    
    
    _actual_position=_actual_velocity=_actual_torque=_actual_amperage = 0.0;
    
    _homing_position=0.0;
    
    _led_onoff=_fan_onoff=false;

};

void Motor_Setup::set_data_object_type(std::string data_object_type)
{
  _data_object_type=data_object_type;  
}

void Motor_Setup::set_control_mode(std::string control_mode)
{
    _control_mode=control_mode;
}
void Motor_Setup::set_position_gains(std::vector<float> position_gains)
{
    if(!position_gains.empty())
        _position_gains.clear();
    
    _position_gains=position_gains;
}
void Motor_Setup::set_velocity_gains(std::vector<float> velocity_gains)
{
    if(!_velocity_gains.empty())
        _velocity_gains.clear();
    
    _velocity_gains=velocity_gains;
}
void Motor_Setup::set_impedance_gains(std::vector<float> impedance_gains)
{
    if(!_impedance_gains.empty())
        _impedance_gains.clear();
        
    _impedance_gains=impedance_gains;
}

void Motor_Setup::set_max_current_A(float max_current_A)
{
    _max_current_A=max_current_A;
}

void Motor_Setup::set_actual_position(float actual_position)
{
    _actual_position=actual_position;
}

void Motor_Setup::set_actual_velocity(float actual_velocity)
{
    _actual_velocity=actual_velocity;
}

void Motor_Setup::set_actual_torque(float actual_torque)
{
    _actual_torque=actual_torque;
}

void Motor_Setup::set_actual_amperage(float actual_amperage)
{
    _actual_amperage=actual_amperage;
}

void Motor_Setup::set_homing_position(float homing_position)
{
    _homing_position=homing_position;
}

void Motor_Setup::set_led_onoff(bool led_onoff)
{
    _led_onoff=led_onoff;
}

void Motor_Setup::set_fan_onoff(bool fan_onoff)
{
    _fan_onoff=fan_onoff;
}

void Motor_Setup::set_min_pos(float min_pos)
{
    _min_pos=min_pos;
}
void Motor_Setup::set_max_pos(float max_pos)
{
    _max_pos=max_pos;
}
void Motor_Setup::set_max_vel(float max_vel)
{
    _max_vel=max_vel;
}
void Motor_Setup::set_max_torq(float max_torq)
{
    _max_torq=max_torq;
}
void Motor_Setup::set_c28_firm_info(std::string c28_firm,std::string c28_firm_password)
{
    _c28_firm=c28_firm;
    _c28_firm_password=c28_firm_password;
}
void Motor_Setup::set_m3_firm_info(std::string m3_firm,std::string m3_firm_password)
{
    _m3_firm=m3_firm;
    _m3_firm_password=m3_firm_password;
}

void Motor_Setup::set_trajectory_structure(trajectoy_t trajectoy_s)
{
    _trajectoy_s=trajectoy_s;
}


std::string Motor_Setup::get_data_object_type()
{
    return _data_object_type;
}

int Motor_Setup::get_id_low_level()
{
    return _id_low_level;
}


string Motor_Setup::get_control_mode()
{
    return _control_mode;
};

bool Motor_Setup::get_brake_present()
{
    return _brake_present;
};

int Motor_Setup::get_sign()
{
    return _sign;
};
float Motor_Setup::get_pos_offset()
{
    return _pos_offset;
};
float Motor_Setup::get_max_current_A()
{
    return _max_current_A;
};

float Motor_Setup::get_actual_position()
{
    return _actual_position;
};    
    
float Motor_Setup::get_actual_velocity()
{
    return _actual_velocity;
};    
    
float Motor_Setup::get_actual_torque()
{
    return _actual_torque;
};    
    
float Motor_Setup::get_actual_amperage()
{
    return _actual_amperage;
};

float Motor_Setup::get_homing_position()
{
    return _homing_position;
}

bool Motor_Setup::get_led_onoff()
{
    return _led_onoff;
};    
    
bool Motor_Setup::get_fan_onoff()
{
    return _fan_onoff;
};


vector<float> Motor_Setup::get_position_gains()
{
    return _position_gains;
};
vector<float> Motor_Setup::get_velocity_gains()
{
    return _velocity_gains;
};
vector<float> Motor_Setup::get_impedance_gains()
{
    return _impedance_gains;
};

float Motor_Setup::get_min_pos()
{
    return _min_pos;
}
float Motor_Setup::get_max_pos()
{
    return _max_pos;
}
float Motor_Setup::get_max_vel()
{
    return _max_vel;
}
float Motor_Setup::get_max_torq()
{
    return _max_torq;
}

void Motor_Setup::get_c28_firm(std::string &c28_firm,std::string &c28_firm_password)
{
    c28_firm=_c28_firm;
    c28_firm_password=_c28_firm_password;
}

void Motor_Setup::get_m3_firm(std::string &m3_firm,std::string &m3_firm_password)
{
    m3_firm=_m3_firm;
    m3_firm_password=_m3_firm_password;
}

string Motor_Setup::get_name()
{
    return _name;
}

Motor_Setup::trajectoy_t Motor_Setup::get_trajectory_structure()
{
    return _trajectoy_s;
}

Motor_Setup::~Motor_Setup()
{
};

