#ifndef LOWPASSFILTER_HPP_
#define LOWPASSFILTER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Sum.hpp>

using namespace eeros::control;

template <typename T = double>
class LowPassFilter : public Block
{
public:
    LowPassFilter(T K, T Tc)
        : K(K),
          Tcinv(1 / Tc)
    {
        sum.getIn(0).connect(this->K.getOut());
        sum.negateInput(1);
        sum.getIn(1).connect(i.getOut());
        Tcinv.getIn().connect(sum.getOut());
        i.getIn().connect(Tcinv.getOut());
        i.enable();
        i.setInitCondition(0.0);
    }

    Input<T> &getIn() { return K.getIn(); }
    Output<T> &getOut() { return i.getOut(); }

    virtual void run()
    {
        K.run();
        sum.run();
        Tcinv.run();
        i.run();
    }

protected:
    Sum<2, T> sum;
    Gain<T> K, Tcinv;
    I<T> i;
};

#endif //LOWPASSFILTER_HPP_
