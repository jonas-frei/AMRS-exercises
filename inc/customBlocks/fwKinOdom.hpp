#ifndef FWKINODOM_HPP_
#define FWKINODOM_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>

using namespace eeros::control;

template <typename T = double>
class FwKinOdom : public Block
{
public:
    FwKinOdom() 
    {
        // Connect subblocks, initialize variables, ...
    }
    
    // Implement getter functions for the inputs and outputs

    virtual void run()
    {
        // Calculate output values, set timestamps and 
        // call the run method of the subblocks
    }

protected:
    Gain<T> RJW;
    I<T> GrR, phi;
};

#endif //FWKINODOM_HPP_
