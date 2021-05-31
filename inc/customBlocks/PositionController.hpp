#ifndef POSITIONCONTROLLER_HPP_
#define POSITIONCONTROLLER_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <cmath>
#include <mutex>

using namespace eeros::control;

class PositionController : public Block
{
public:
    PositionController(double k1, double k2, double k3,
                       double tolPos = 5e-3, double tolRot = M_PI / 1000.0,
                       double xT = 0.0, double yT = 0.0, double phiT = 0.0)
        : GrR(this),
          phi(this),
          xT(xT),
          yT(yT),
          phiT(phiT),
          k1(k1),
          k2(k2),
          k3(k3),
          tolPos(tolPos),
          tolRot(tolRot),
          targetReached(true),
          enabled(false),
          RvRx(this),
          omegaR(this)
    {
        // Name all signals
        this->RvRx.getSignal().setName("Robot translational velocity setpoint"
                                       " in the robot frame [m/s]");
        this->omegaR.getSignal().setName("Robot angular velocity setpoint [rad/s]");
    }

    // Input getter functions
    Input<eeros::math::Vector2> &getInGrR() { return GrR; }
    Input<> &getInphi() { return phi; }

    // Output getter functions
    Output<> &getOutRvRx() { return RvRx; }
    Output<> &getOutomegaR() { return omegaR; }

    virtual void run()
    {
        // Check if the controller is enabled and the target position is not reached
        // If true: run the algorithm
        // Else: set the translational and angular velocities to 0
        if (enabled && !targetReached)
        {
            std::lock_guard<std::mutex> lock(mtx);
            // Calculate the distance to the target
            rho = sqrt(square(xT - GrR.getSignal().getValue()[0]) +
                       square(yT - GrR.getSignal().getValue()[1]));
            // Check if the target is reached
            // If true: set the translational and angular velocities to 0
            // Else: calculate the translational and angular velocities
            if (rho <= tolPos)
            {
                targetReached = true;
                RvRx.getSignal().setValue(0.0);
                omegaR.getSignal().setValue(0.0);
            }
            else
            {
                // Calculate the orientation to the target
                gamma = constrainAngle(atan2(yT - GrR.getSignal().getValue()[1],
                                             xT - GrR.getSignal().getValue()[0]) -
                                       phi.getSignal().getValue());
                delta = constrainAngle(gamma + phi.getSignal().getValue() - phiT);
                // Check if orientation is already correct
                // If true: the term sin(gamma)/gamma is approximately 1
                // Else: use the formula as it was shown in the lecture
                if (fabs(gamma) <= tolRot)
                {
                    omegaR.getSignal().setValue(k2 * gamma +
                                                k1 * cos(gamma) * (gamma + k3 * delta));
                }
                else
                {
                    omegaR.getSignal().setValue(k2 * gamma +
                                                k1 * sin(gamma) * cos(gamma) * (gamma + k3 * delta) / gamma);
                }
                // Set the translational velocity
                RvRx.getSignal().setValue(k1 * rho * cos(gamma));
            }
        }
        else
        {
            RvRx.getSignal().setValue(0.0);
            omegaR.getSignal().setValue(0.0);
        }
        // Set timestamps of the output signals
        RvRx.getSignal().setTimestamp(GrR.getSignal().getTimestamp());
        omegaR.getSignal().setTimestamp(GrR.getSignal().getTimestamp());
    }

    // Enable the controller
    void enable(void) { enabled = true; }
    // Disable the controller
    void disable(void) { enabled = false; }

    // Set new target position and orientation
    void setTarget(double xT, double yT, double phiT)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->xT = xT;
        this->yT = yT;
        this->phiT = phiT;
        this->targetReached = false;
    }

    // Get the controller status
    bool getStatus(void) { return targetReached; }

protected:
    std::mutex mtx;
    Input<eeros::math::Vector2> GrR;
    Input<> phi;
    double rho, gamma, delta, xT, yT, phiT, k1, k2, k3, tolPos, tolRot;
    bool targetReached, enabled;
    Output<> RvRx, omegaR;

private:
    double square(double x) { return x * x; }
    double constrainAngle(double x)
    {
        x = fmod(x + M_PI, 2.0 * M_PI);
        if (x < 0.0)
            x += 2.0 * M_PI;
        return x - M_PI;
    }
};

#endif //POSITIONCONTROLLER_HPP_
