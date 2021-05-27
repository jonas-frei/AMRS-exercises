#ifndef AMRSCONSTANTS_HPP_
#define AMRSCONSTANTS_HPP_

namespace AMRSC
{
    namespace MOT
    {
        const double QMAX = 2.5;                    // [N]
        const double qdMAX = 0.848;                 // [m/s]
        const double i = 3441.0 / 104.0 / 0.04;     // [1/m]
        const double R = 8.0;                       // [hm]
        const double KM = 8.44e-3;                  // [Nm/A]
    }
    namespace CONT
    {
        const double D = 0.7;                       // [-]
        const double s = 2.2;                       // [-]
        const double M = 0.046;                     // [kg]
        const double ILIMIT = 0.1;                  // [m]
        const double K1 = 0.5;
        const double K2 = 1.0;
        const double K3 = 1.0;
        const double posTol = 1e-3;                 // [m]
        const double rotTol = 1e-3;                 // [rad]
    }
    namespace ROB
    {
        const double B = 0.15;                      // [m]
    }
}

#endif // AMRSCONSTANTS_HPP_