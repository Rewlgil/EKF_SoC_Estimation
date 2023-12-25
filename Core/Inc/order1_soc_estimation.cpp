#include <vector>
#include <math.h>
#include "matrix.hpp"
#include "order1_soc_estimation.hpp"
#include "voc_soc.hpp"

using namespace std;

#define constrain(x, x_min, x_max) max(min(x, x_max), x_min)

order1_soc_estimation::order1_soc_estimation(const_param *my_param) 
    : param(*my_param)
{
    X = matrix(3, 1);
    X.mat[0][0] = 0.5;  // initial SoC
    
    X_min[0] = 0;       X_max[0] = 1;   // SoC
    X_min[1] = -0.2;    X_max[1] = 0.2; // V1
    X_min[2] = -0.6;    X_max[2] = 0.6; // V0

    K = matrix(3, 1);

    Qp = matrix(3, 3);  // Cov of parameter
    Qp.mat[0][0] = param.var_R0;
    Qp.mat[1][1] = param.var_R1;
    Qp.mat[2][2] = param.var_C1;

    P = matrix(3, 3);   // System state error
    P = P.eyes(3) * 0.25;

    S = matrix(2, 2);
    S = S.eyes(2) * param.var_I;

    e1 = exp(-param.dt/(param.R1*param.C1));

    A = matrix(3, 3);   // State transition matrix
    A.mat[0][0] = 1;
    A.mat[1][1] = e1;

    B = matrix(3, 2);   // Control input matrix
    B.mat[0][0] = -param.dt/(3600*param.Qcell);
    B.mat[1][0] = param.R1*(1-e1);
    B.mat[2][1] = param.R0;

    BSB = matrix(3, 3);
    BSB = B * (S * B.t());

    C = matrix(1, 3);   // Measurement matrix
    C.mat[0][1] = -1;
    C.mat[0][2] = -1;

    J = matrix(3, 3);   // linearization state/parameter

    Q = matrix(3, 3);   // process noise Cov

    tmp_m = matrix(1, 1);

    last_I = 0;
}

double order1_soc_estimation::soc_ekf_compute(double V, double I)
{
    J.mat[1][1] = e1*(param.dt/(pow(param.R1,2.0)*param.C1))*
                 (X.mat[1][0] - param.R1*last_I) + (1-e1)*last_I;
    J.mat[1][2] = e1*(param.dt/(param.R1*pow(param.C1,2.0)))*
                 (X.mat[1][0] - param.R1*last_I);
    J.mat[2][0] = I;

    X = (A * X) + (B * matrix(2,1,{{last_I},{I}}));

    for (size_t i = 0; i < 3; i++) {
        X.mat[i][0] = constrain(X.mat[i][0], X_min[i], X_max[i]);
    }

    Q = (J * (Qp * J.t())) + BSB;

    P = (A * (P * A.t())) + Q;

    C.mat[0][0] = voc_soc_slope_lut(X.mat[0][0]);

    // zp = Voc - V1 - V0
    zp = voc_lut(X.mat[0][0]) - X.mat[1][0] - X.mat[2][0];
    
    // K = (P * C') / (C * P * C' + R)
    tmp_m = C * (P * C.t());
    tmp = tmp_m.mat[0][0] + param.var_V;
    K = (P * C.t()) / tmp;

    X = X + (K * (V - zp));

    for (size_t i = 0; i < 3; i++) {
        X.mat[i][0] = constrain(X.mat[i][0], X_min[i], X_max[i]);
    }

    P = (P.eyes(3) - (K*C)) * P;

    last_I = I;

    return X.mat[0][0];
}

double order1_soc_estimation::voc_soc_slope_lut(double soc)
{
    return voc_soc_slope[(int)(soc*1000)];
}

double order1_soc_estimation::voc_lut(double soc)
{
    // y = slope * (x - x1) + y1
    return ((voc_soc_slope_lut(soc)*(soc-(round(soc*1000)/1000)))
         + (voc_soc[(int)(soc*1000)]));
}
