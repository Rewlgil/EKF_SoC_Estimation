#ifndef _ORDER2_SOC_ESTIMATION_HPP_
#define _ORDER2_SOC_ESTIMATION_HPP_

#include <vector>
#include "matrix.hpp"

struct const_param
{
    double 
        Qcell,      // cell Capacity [Ah]
        R0,         // R0 parameter
        R1,         // R1 parameter
        R2,         // R2 parameter
        C1,         // C1 parameter
        C2,         // C2 parameter
        var_R0,     // Variance of R0
        var_R1,     // Variance of R1
        var_R2,     // Variance of R2
        var_C1,     // Variance of C1
        var_C2,     // Variance of C2
        var_I,      // Variance of current measurement
        var_V,      // Variance of voltage measurement
        dt;         // Calculation period
};

class order2_soc_estimation
{
public:
    order2_soc_estimation(const_param *my_param);
    double soc_ekf_compute(double V, double I);
    
private:
    /* function */
    double voc_soc_slope_lut(double soc);
    double voc_lut(double soc);

    /* variable */
    const_param param;
    matrix
        X,          // 4x1 state var [Soc V1 V0]
        K,          // 4x1 Kalman gain
        Qp,         // 5x5 Cov of parameter
        P,          // 4x4
        S,          // 2x2
        BSB,        // 4x4 BSB'
        A,          // 4x4 State transition matrix
        B,          // 4x2 Control input matrix
        C,          // 1x4 Measurement matrix
        J,          // 4x5 linearization state/parameter
        Q,          // 4x4 process noise Cov
        tmp_m;      // 1x1
    double    
        X_min[4],
        X_max[4],
        zp,         // predict measurement value
        e1,         // RC1 expo declay
        e2,         // RC2 expo declay
        last_I,     // Current in last compute loop
        tmp;
};

#endif