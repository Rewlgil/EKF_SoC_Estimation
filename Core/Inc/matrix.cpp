#include <iostream>
#include <vector>
#include "matrix.hpp"

using namespace std;

static vector<vector<double>> emp{{0}};

matrix::matrix(int r, int c, vector<vector<double>> m)
    : row(r), col(c), mat(m)
{
    mat.resize(row, vector<double>(col));
}

matrix::matrix(int r, int c)
    : row(r), col(c)
{
    mat.clear();
    mat.resize(row, vector<double>(col));
}

matrix::matrix()
{}

matrix emptyMatrix(1, 1, emp);

matrix matrix::operator + (matrix const &m2)
{
    if (row != m2.row || col != m2.col) {
        cout << "matrix addition error" << endl;
        return emptyMatrix;
    }

    matrix res(row, col);

    for (size_t r = 0; r < row; r++)
    {
        for (size_t c = 0; c < col; c++)
        {
            res.mat[r][c] = mat[r][c] + m2.mat[r][c];
        }       
    }
    return res;
}

matrix matrix::operator - (matrix const &m2)
{
    if (row != m2.row || col != m2.col) {
        cout << "matrix subtraction error" << endl;
        return emptyMatrix;
    }

    matrix res(row, col);

    for (size_t r = 0; r < row; r++)
    {
        for (size_t c = 0; c < col; c++)
        {
            res.mat[r][c] = mat[r][c] - m2.mat[r][c];
        }       
    }
    return res;
}

matrix matrix::operator * (matrix const &m2)
{
    if (col != m2.row) {
        cout << "matrix multiplication error" << endl;
        return emptyMatrix;
    }

    matrix res(row, m2.col);

    for (size_t r = 0; r < row; r++)
    {
        for (size_t c = 0; c < m2.col; c++)
        {
            for (size_t i = 0; i < col; i++)
            {
                res.mat[r][c] += mat[r][i] * m2.mat[i][c];
            }
        }       
    }
    return res;
}

matrix matrix::operator * (double m)
{
    matrix res(row, col);

    for (size_t r = 0; r < row; r++)
    {
        for (size_t c = 0; c < col; c++)
        {
            res.mat[r][c] = mat[r][c] * m;
        }       
    }
    return res;
}

matrix matrix::operator / (double d)
{
    matrix res(row, col);

    for (size_t r = 0; r < row; r++)
    {
        for (size_t c = 0; c < col; c++)
        {
            res.mat[r][c] = mat[r][c] / d;
        }       
    }
    return res;
}

matrix matrix::t()
{
    matrix res(col, row);

    for (size_t r = 0; r < row; r++)
    {
        for (size_t c = 0; c < col; c++)
        {
            res.mat[c][r] = mat[r][c];
        }       
    }
    return res;
}

void matrix::print()
{
    cout << "{" << endl;
    for (vector<double> vect1D : mat) 
    {
        cout << "   "; 
        for (double x : vect1D)
        { 
            cout << x << " "; 
        }     
        cout << endl;
    }
    cout << "}" << endl;
}

matrix matrix::eyes(int size)
{
    matrix res(size, size);

    for (size_t r = 0; r < size; r++)
    {
        res.mat[r][r] = 1;
    }
    return res;
}