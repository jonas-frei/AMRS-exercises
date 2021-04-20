#ifndef CUSTOMBLOCKTEMPLATE_HPP_
#define CUSTOMBLOCKTEMPLATE_HPP_

#include <eeros/control/Block.hpp>

using namespace eeros::control;

template <typename T = double>
class CustomBlockName : public Block
{
public:
    CustomBlockName() {}

    virtual void run() {}

protected:
};

#endif //CUSTOMBLOCKTEMPLATE_HPP_
