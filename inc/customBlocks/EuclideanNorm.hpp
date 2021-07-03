#ifndef EUCLIDEANNORM_HPP_
#define EUCLIDEANNORM_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::control;

template <uint8_t NrIn = 2>
class EuclideanNorm : public Block1i1o<eeros::math::Vector<NrIn, double>, double>
{
public:
    EuclideanNorm() {}

    virtual void run()
    {
        double norm = 0.0;
        for (uint8_t i = 0; i < NrIn; i++)
            norm += square(this->in.getSignal().getValue()[i]);
        norm = sqrt(norm);
        this->out.getSignal().setValue(norm);
        this->out.getSignal().setTimestamp(this->in.getSignal().getTimestamp());
    }

protected:
    double square(double x) { return x * x; }
};

#endif //EUCLIDEANNORM_HPP_
