#include <vector>
#include <math.h>
#include "matrix.hpp"
#include "rint_soc_estimation.hpp"
#include "voc_soc.hpp"

using namespace std;

#define constrain(x, x_min, x_max) max(min(x, x_max), x_min)

rint_soc_estimation::rint_soc_estimation(const_param *my_param) 
    : param(*my_param)
{
    X = matrix(2, 1);
    X.mat[0][0] = 0.5;  // initial SoC
    
    X_min[0] = 0;       X_max[0] = 1;   // SoC
    X_min[1] = -0.5;    X_max[1] = 0.5; // V0

    K = matrix(2, 1);

    Qp = param.var_R0;

    P = matrix(2, 2);   // System state error
    P = P.eyes(2) * 0.25;

    S = matrix(2, 2);
    S = S.eyes(2) * param.var_I;

    A = matrix(2, 2);   // State transition matrix
    A.mat[0][0] = 1;

    B = matrix(2, 2);   // Control input matrix
    B.mat[0][0] = -param.dt/(3600*param.Qcell);
    B.mat[1][1] = param.R0;

    BSB = matrix(2, 2);
    BSB = B * (S * B.t());

    C = matrix(1, 2);   // Measurement matrix
    C.mat[0][1] = -1;

    J = matrix(2, 2);   // linearization state/parameter

    Q = matrix(2, 2);   // process noise Cov

    tmp_m = matrix(1, 1);

    last_I = 0;
}

double rint_soc_estimation::soc_ekf_compute(double V, double I)
{
    J.mat[1][0] = I;

    X = (A * X) + (B * matrix(2,1,{{last_I},{I}}));

    for (size_t i = 0; i < 2; i++) {
        X.mat[i][0] = constrain(X.mat[i][0], X_min[i], X_max[i]);
    }

    Q = (J * J.t() * Qp) + BSB;

    P = (A * (P * A.t())) + Q;

    C.mat[0][0] = voc_soc_slope_lut(X.mat[0][0]);

    // zp = Voc - V0
    zp = voc_lut(X.mat[0][0]) + 0.005 - X.mat[1][0];
    
    // K = (P * C') / (C * P * C' + R)
    tmp_m = C * (P * C.t());
    tmp = tmp_m.mat[0][0] + param.var_V;
    K = (P * C.t()) / tmp;

    X = X + (K * (V - zp));

    for (size_t i = 0; i < 2; i++) {
        X.mat[i][0] = constrain(X.mat[i][0], X_min[i], X_max[i]);
    }

    P = (P.eyes(2) - (K*C)) * P;

    last_I = I;

    return X.mat[0][0];
}

double rint_soc_estimation::voc_soc_slope_lut(double soc)
{
    return voc_soc_slope[(int)(soc*1000)];
}

double rint_soc_estimation::voc_lut(double soc)
{
    // y = slope * (x - x1) + y1
    return ((voc_soc_slope_lut(soc)*(soc-(round(soc*1000)/1000)))
         + (voc_soc[(int)(soc*1000)]));
}
