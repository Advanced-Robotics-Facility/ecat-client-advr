#ifndef __EC_ZMQ_CONTEXT__
#define __EC_ZMQ_CONTEXT__

#include <protobuf/repl_cmd.pb.h>
#include <protobuf/ecat_pdo.pb.h>

#include "zmq.hpp"
#include <zmq_addon.hpp>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace EcZmqPdoContext{
    extern std::unique_ptr<zmq::context_t> pdo_context;
    inline static void start_context(){
        if(pdo_context==nullptr){
            try{
                pdo_context=std::make_unique<zmq::context_t>(1);
                //std::cout << "starting of zmq pdo context" << std::endl;
            }catch (const zmq::error_t &err){
                //std::cout << "fatal error on starting zmq pdo context: " << err.what() << std::endl;
            }
        }
    }
    inline static void stop_context(){
        if(pdo_context!=nullptr){
            try{
                pdo_context->shutdown();
                //std::cout << "shutdowning of zmq pdo context" << std::endl;
                pdo_context.reset();
            }catch (const zmq::error_t &err){
                //std::cout << "fatal error on shutdowning zmq pdo context: " << err.what() << std::endl;
            }
        }
    }
}

namespace EcZmqCmdContext{
    extern std::unique_ptr<zmq::context_t> cmd_context;
    inline static void start_context(){
        if(cmd_context==nullptr){
            try{
                cmd_context=std::make_unique<zmq::context_t>(1);
                //std::cout << "starting of zmq command context" << std::endl;
            }catch (const zmq::error_t &err){
                //std::cout << "fatal error on starting zmq command context: " << err.what() << std::endl;
            }
        }
    }
    inline static void stop_context(){
        if(cmd_context!=nullptr){
            try{
                cmd_context->shutdown();
                //std::cout << "shutdowning of zmq command context" << std::endl;
                cmd_context.reset();
            }catch (const zmq::error_t &err){
                //std::cout << "fatal error on shutdowning zmq command context: " << err.what() << std::endl;
            }
        }
    }
}
#endif


/******************** SINGLETON TEST */
/*
class EcZmqPdoContext {
public:
    EcZmqPdoContext &operator=(EcZmqPdoContext &&) = default;
    static EcZmqPdoContext& getInstance()
    {
        static EcZmqPdoContext instance;
        if(_pdo_context==nullptr){
            _pdo_context=std::make_unique<zmq::context_t>(1);
            std::cout << "new context" << std::endl;
        }
        return instance;
    }

    static zmq::context_t* context(){
        return _pdo_context.get();
    }

    static void clear_context() {
        getInstance() = EcZmqPdoContext{};
        if(_pdo_context!=nullptr){
            _pdo_context->shutdown();
            std::cout << "shutdown context" << std::endl;
        }
        _pdo_context.reset();
    }

    EcZmqPdoContext(EcZmqPdoContext &&) = delete;
    EcZmqPdoContext(const EcZmqPdoContext&) = delete;
    EcZmqPdoContext& operator=(const EcZmqPdoContext&) = delete;

private:
    EcZmqPdoContext()
    {
    }

    ~EcZmqPdoContext()
    {
    }

    static std::unique_ptr<zmq::context_t> _pdo_context;
};
*/