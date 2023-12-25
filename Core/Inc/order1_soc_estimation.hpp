#ifndef _ORDER1_SOC_ESTIMATION_HPP_
#define _ORDER1_SOC_ESTIMATION_HPP_

#include <vector>
#include "matrix.hpp"

struct const_param
{
    double 
        Qcell,      // cell Capacity [Ah]
        R0,         // R0 parameter
        R1,         // R1 parameter
        C1,         // C1 parameter
        var_R0,     // Variance of R0
        var_R1,     // Variance of R1
        var_C1,     // Variance of C1
        var_I,      // Variance of current measurement
        var_V,      // Variance of voltage measurement
        dt;         // Calculation period
};

class order1_soc_estimation
{
public:
    order1_soc_estimation(const_param *my_param);
    double soc_ekf_compute(double V, double I);
    
private:
    /* function */
    double voc_soc_slope_lut(double soc);
    double voc_lut(double soc);

    /* variable */
    const_param param;
    matrix
        X,          // 3x1 state var [Soc V1 V0]
        K,          // 3x1 Kalman gain
        Qp,         // 3x3 Cov of parameter
        P,          // 3x3
        S,          // 2x2
        BSB,        // 3x3 BSB'
        A,          // 3x3 State transition matrix
        B,          // 3x2 Control input matrix
        C,          // 1x3 Measurement matrix
        J,          // 3x3 linearization state/parameter
        Q,          // 3x3 process noise Cov
        tmp_m;      // 1x1
    double    
        X_min[3],
        X_max[3],
        zp,         // predict measurement value
        e1,         // RC expo declay
        last_I,     // Current in last compute loop
        tmp;
};

#endif