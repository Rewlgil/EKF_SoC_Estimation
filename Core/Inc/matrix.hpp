#ifndef _MATRIX_HPP_
#define _MATRIX_HPP_

#include <vector>

using namespace std;

class matrix
{
private:
    int row, col;
public:
    vector<vector<double>> mat;

    matrix(int r, int c, vector<vector<double>>m);
    matrix(int r, int c);
    matrix();

    matrix operator + (matrix const &m2);
    matrix operator - (matrix const &m2);
    matrix operator * (matrix const &m2);
    matrix operator * (double m);
    matrix operator / (double d);
    matrix t();     // transpost
    void print();

    matrix eyes(int size);
};

#endif