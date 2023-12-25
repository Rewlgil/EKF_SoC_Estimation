#ifndef _RINT_SOC_ESTIMATION_HPP_
#define _RINT_SOC_ESTIMATION_HPP_

#include <vector>
#include "matrix.hpp"

struct const_param
{
    double 
        Qcell,      // cell Capacity [Ah]
        R0,         // R0 parameter
        var_R0,     // Variance of R0
        var_I,      // Variance of current measurement
        var_V,      // Variance of voltage measurement
        dt;         // Calculation period
};

class rint_soc_estimation
{
public:
    rint_soc_estimation(const_param *my_param);
    double soc_ekf_compute(double V, double I);
    
private:
    /* function */
    double voc_soc_slope_lut(double soc);
    double voc_lut(double soc);

    /* variable */
    const_param param;
    matrix
        X,          // 2x1 state var [Soc V1 V0]
        K,          // 2x1 Kalman gain
        P,          // 2x2
        S,          // 2x2
        BSB,        // 2x2 BSB'
        A,          // 2x2 State transition matrix
        B,          // 2x2 Control input matrix
        C,          // 1x2 Measurement matrix
        J,          // 2x2 linearization state/parameter
        Q,          // 2x2 process noise Cov
        tmp_m;      // 1x1
    double    
        X_min[2],
        X_max[2],
        Qp,         // Cov of parameter
        zp,         // predict measurement value
        last_I,     // Current in last compute loop
        tmp;
};

#endif