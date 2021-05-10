#ifndef PDCONTROLLER_HPP_
#define PDCONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Sum.hpp>

using namespace eeros::control;

template <typename T = double>
class PDController : public Block
{
public:
    PDController(double Kp, double Kd, double M)
        : Kp(Kp), Kd(Kd), M(M)
    {
        connectInputs();
    }

    PDController(double f_task, double D, double s, double M)
        : Kp(f_task/s/D * f_task/s/D), Kd(2.0 * f_task/s), M(M)
    {
        connectInputs();
    }

    virtual Input<T> &getIn(int index)
    {
        return e.getIn(index);
    }

    virtual Output<T> &getOut()
    {
        return M.getOut();
    }

    virtual void run()
    {
        e.run();
        Kp.run();
        d.run();
        Kd.run();
        qdd.run();
        M.run();
    }

protected:
    Sum<2, T> e, qdd;
    Gain<T> Kp, Kd, M;
    D<T> d;

private:
    void connectInputs(void)
    {
        e.negateInput(1);
        Kp.getIn().connect(e.getOut());
        d.getIn().connect(e.getOut());
        Kd.getIn().connect(d.getOut());
        qdd.getIn(0).connect(Kp.getOut());
        qdd.getIn(1).connect(Kd.getOut());
        M.getIn().connect(qdd.getOut());
    }
};

#endif //PDCONTROLLER_HPP_
