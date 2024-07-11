#ifndef __EC_THREAD_H__
#define __EC_THREAD_H__

#include <pthread.h>
#include <linux/sched.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <bits/local_lim.h>
#include <exception>
#include <typeinfo>
#include <iostream>
#include <sys/timerfd.h>
#include <map>
#include <memory>

#include <utils.h>

#define handle_error_en(en, msg) \
    do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

typedef struct {
    struct timeval task_time;
    struct timeval period;
} ec_period_t;


class EcThread;

typedef EcThread* EcThread_Ptr;
typedef std::map<std::string, EcThread_Ptr> EcThreadsMap; 

void * periodic_thread ( EcThread_Ptr );
void * non_periodic_thread ( EcThread_Ptr );
void * nrt_thread ( EcThread_Ptr );

extern pthread_barrier_t threads_barrier;

class EcThread {

public:

    virtual ~EcThread();

    void create ( int rt, int cpu_nr );
    void stop ( void );
    void join ( void );

    int is_non_periodic();

    virtual void th_init ( void * ) = 0;
    virtual void th_loop ( void * ) = 0;

    static void * th_helper ( void * );

protected:

    int _run_loop;

    std::string     name;
    ec_period_t     period;

    pthread_t       thread_id;
    // pthread attribute
    int             schedpolicy;
    int             priority;
    int             stacksize;

    int             fd_timer;
    
    friend void * periodic_thread ( EcThread_Ptr );
    friend void * non_periodic_thread ( EcThread_Ptr );

};

inline EcThread::~EcThread() {

    close(fd_timer);
    std::cout << "~" << typeid ( this ).name() << " " << std::string(name) <<std::endl;
}

inline int EcThread::is_non_periodic() {

    return ( period.period.tv_sec == 0 && period.period.tv_usec == 1 );
}

inline void * EcThread::th_helper ( void *kls )  {

        if ( ( ( EcThread_Ptr ) kls )->is_non_periodic() ) {
            return non_periodic_thread ( ( EcThread_Ptr ) kls );
        }
        return periodic_thread ( ( EcThread_Ptr ) kls );

}


inline void EcThread::stop() {
    // exit loop of function in wrap_thread.cpp
    _run_loop = 0;
}

inline void EcThread::join() {
    //pthread_cancel(thread_id);
    pthread_join ( thread_id, 0 );
}

inline void EcThread::create ( int rt=true, int cpu_nr=-1 ) {

    int ret;
    pthread_attr_t      attr;
    struct sched_param  schedparam;
    cpu_set_t           cpu_set;
    size_t 		dflt_stacksize;
    _run_loop = 1;

    CPU_ZERO ( &cpu_set );
    CPU_SET ( cpu_nr,&cpu_set );

    ret = pthread_attr_init(&attr);
    if (ret != 0) handle_error_en(ret, "pthread_attr_init");
    
    ret = pthread_attr_setinheritsched ( &attr, PTHREAD_EXPLICIT_SCHED );
    if (ret != 0) handle_error_en(ret, "pthread_attr_setinheritsched");

    ret = pthread_attr_setschedpolicy ( &attr, schedpolicy );
    if (ret != 0) handle_error_en(ret, "pthread_attr_setschedpolicy");

    schedparam.sched_priority = priority;
    ret = pthread_attr_setschedparam ( &attr, &schedparam );
    if (ret != 0) handle_error_en(ret, "pthread_attr_setschedparam");

    pthread_attr_getstacksize ( &attr, &dflt_stacksize );
    //DPRINTF ( "default stack size %ld\n", dflt_stacksize );
    if ( stacksize > 0 ) {
        ret = pthread_attr_setstacksize ( &attr, stacksize );
        if (ret != 0) handle_error_en(ret, "pthread_attr_setstacksize");
    
    }
    
    ret = pthread_attr_setdetachstate ( &attr, PTHREAD_CREATE_JOINABLE );
    if (ret != 0) handle_error_en(ret, "pthread_attr_setdetachstate");

    if ( cpu_nr >= 0 ) {
        ret = pthread_attr_setaffinity_np ( &attr, sizeof ( cpu_set ), &cpu_set );
        if (ret != 0) handle_error_en(ret, "pthread_attr_setaffinity_np");
    }
    
    ret = pthread_create ( &thread_id, &attr, &th_helper, this );

    pthread_attr_destroy ( &attr );

    if ( ret != 0 ) {
        DPRINTF ( "%s %d %s", __FILE__, __LINE__, name.c_str() );
        handle_error_en(ret, "pthread_create");
    }

}


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

inline void * periodic_thread ( EcThread_Ptr th_hook ) {
    
    int                 ret = 0;
    uint64_t            time_ns=0,start_time_ns=0,sleep_ns=0;
    uint64_t            min_sleep_ns=10000;
    struct timespec     periodtp;
    int overruns = 0;
        

    // thread specific initialization
    th_hook->th_init ( 0 );

    DPRINTF ( "%s %s period {%ld,%ld} tv\n",
              __FUNCTION__, th_hook->name.c_str(),
              th_hook->period.period.tv_sec,
              th_hook->period.period.tv_usec );

    ret = pthread_setname_np ( pthread_self(), th_hook->name.c_str() );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_name_np() return code %d\n",
                  th_hook->name.c_str(), ret );
        exit ( 1 );
    }


#ifdef __COBALT__
    // PTHREAD_WARNSW, when set, cause the signal SIGXCPU to be sent to the
    // current thread, whenever it involontary switches to secondary mode;
    if(th_hook->sched_policy == SCHED_FIFO){
        ret = pthread_setmode_np ( 0, PTHREAD_WARNSW, 0 );
        if ( ret != 0 ) {
            DPRINTF ( "%s : pthread_set_mode_np() return code %d\n",
                    th_hook->name.c_str(), ret );
            exit ( 1 );
        }
    }
#endif
    
    periodtp.tv_sec  = th_hook->period.period.tv_sec;
    periodtp.tv_nsec = th_hook->period.period.tv_usec * 1000ULL;
    start_time_ns=iit::ecat::get_time_ns(CLOCK_MONOTONIC);
    time_ns=start_time_ns;

    DPRINTF ( "%s %s : Start looping ...\n",
              __FUNCTION__, th_hook->name.c_str() );


    while ( th_hook->_run_loop ) {

        float time_elapsed_ms= (static_cast<float>((time_ns-start_time_ns))/1000000);   
        time_ns += th_hook->period.period.tv_usec * 1000ULL;
        float sample_time_ms= (static_cast<float>(sleep_ns)/1000000);

        // thread specific loop
        th_hook->th_loop ( 0 );

        sleep_ns = time_ns- iit::ecat::get_time_ns(CLOCK_MONOTONIC);


        #if defined(PREEMPT_RT) || defined(__COBALT__)
                // if less than threshold, print warning (only on rt threads)
                if(sleep_ns < min_sleep_ns && th_hook->sched_policy == SCHED_FIFO){
                    ++overruns;
                    DPRINTF( "Thread: %s overruns detected: %d\n",th_hook->name.c_str(),n_overruns);
                }
        #endif

        sleep_ns = std::max(min_sleep_ns, sleep_ns);
        periodtp.tv_nsec=sleep_ns;
        while(clock_nanosleep(CLOCK_MONOTONIC, 0, &periodtp,NULL) == -1 && errno == EINTR)
        {}
    } // end while

    DPRINTF ( "%s %s : exit thread ...\n",
              __FUNCTION__, th_hook->name.c_str() );

    return 0;
}


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

inline void * non_periodic_thread ( EcThread_Ptr th_hook ) {
    int ret = 0;

    // thread specific initialization
    th_hook->th_init ( 0 );

    DPRINTF ( "%s %s, period %ld us\n",
              __FUNCTION__, th_hook->name.c_str(),
              th_hook->period.period.tv_usec );

    ret = pthread_setname_np ( pthread_self(), th_hook->name.c_str() );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_setname_np() return code %d\n",
                  th_hook->name.c_str(), ret );
        exit ( 1 );
    }

#ifdef __COBALT__
    // PTHREAD_WARNSW, when set, cause the signal SIGXCPU to be sent to the
    // current thread, whenever it involontary switches to secondary mode;
    if(th_hook->sched_policy == SCHED_FIFO){
        ret = pthread_setmode_np ( 0, PTHREAD_WARNSW, 0 );
        if ( ret != 0 ) {
            DPRINTF ( "%s : pthread_set_mode_np() return code %d\n",
                    th_hook->name.c_str(), ret );
            exit ( 1 );
        }
    }
#endif

    DPRINTF ( "%s %s : Start looping ...\n",
              __FUNCTION__, th_hook->name.c_str() );


    while ( th_hook->_run_loop ) {

        // thread specific loop
        th_hook->th_loop ( 0 );

    } // end while

    DPRINTF ( "%s %s : exit thread ...\n",
              __FUNCTION__, th_hook->name.c_str() );
    
    return 0;
}

#endif

