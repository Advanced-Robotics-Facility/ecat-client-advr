#ifndef __EC_GUI_UTILS_H__
#define __EC_GUI_UTILS_H__


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <QLabel>
#include <functional>

#include <stdexcept>
#include <boost/variant.hpp>

template <typename SignalType>
class SecondOrderFilter {

public:

    typedef std::shared_ptr<SecondOrderFilter<SignalType>> Ptr;

    SecondOrderFilter():
        _omega(1.0),
        _eps(0.8),
        _ts(0.01),
        _reset_has_been_called(false)
    {
        computeCoeff();
    }

    SecondOrderFilter(double omega, double eps, double ts, const SignalType& initial_state):
        _omega(omega),
        _eps(eps),
        _ts(ts),
        _reset_has_been_called(false)
    {
        computeCoeff();
        reset(initial_state);
    }

    void reset(const SignalType& initial_state){
        _reset_has_been_called = true;
        _u = initial_state;
        _y = initial_state;
        _yd = initial_state;
        _ydd = initial_state;
        _udd = initial_state;
        _ud = initial_state;
    }

    const SignalType& process(const SignalType& input){

        if(!_reset_has_been_called) reset(input*0);


        _ydd = _yd;
        _yd = _y;
        _udd = _ud;
        _ud = _u;


        _u = input;
        _y = 1.0/_a0 * ( _u + _b1*_ud + _b2*_udd - _a1*_yd - _a2*_ydd );

        return _y;
    }

    const SignalType& getOutput() const {
        return _y;
    }

    void setOmega(double omega){
        _omega = omega;
        computeCoeff();
    }

    double getOmega()
    {
        return _omega;
    }

    void setDamping(double eps){
        _eps = eps;
        computeCoeff();
    }

    double getDamping()
    {
        return _eps;
    }

    void setTimeStep(double ts){
        _ts = ts;
        computeCoeff();
    }

    double getTimeStep()
    {
        return _ts;
    }

private:

    void computeCoeff(){
        _b1 = 2.0;
        _b2 = 1.0;

        _a0 = 1.0 + 4.0*_eps/(_omega*_ts) + 4.0/std::pow(_omega*_ts, 2.0);
        _a1 = 2 - 8.0/std::pow(_omega*_ts, 2.0);
        _a2 = 1.0 + 4.0/std::pow(_omega*_ts, 2.0) - 4.0*_eps/(_omega*_ts);

    }

    double _omega;
    double _eps;
    double _ts;

    double _b1, _b2;
    double _a0, _a1, _a2;

    bool _reset_has_been_called;

    SignalType _y, _yd, _ydd, _u, _ud, _udd;

};

template <size_t n, typename... T>
boost::variant<T...> dynamic_get_impl(size_t i, const std::tuple<T...>& tpl)
{
    if (i == n)
        return std::get<n>(tpl);
    else if (n == sizeof...(T) - 1)
        throw std::out_of_range("Tuple element out of range.");
    else
        return dynamic_get_impl<(n < sizeof...(T)-1 ? n+1 : 0)>(i, tpl);
}

template <typename... T>
boost::variant<T...> dynamic_get(size_t i, const std::tuple<T...>& tpl)
{
    return dynamic_get_impl<0>(i, tpl);
}

#endif // EC_GUI_UTILS_H
