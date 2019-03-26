/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Matrix.hpp"
#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <stdexcept>

// --- Life cycle ---
Matrix::Matrix() { data = nullptr; }

Matrix::Matrix(int n_rows, int n_cols)
{
    rows    = n_rows;
    columns = n_cols;
    if (rows * columns <= MAX_STACK_MATRIX_SIZE)
    {
        // The class has a fixed size array allocated on the stack.
        // If its size is enough to contain all the elements it won't use
        // dynamic allocation, speeding things up considerably.
        data            = stackData;
        isHeapAllocated = false;
    }
    else
    {
        // If the stack allocated array is too small we need to allocate it in
        // the heap. The allocation is done in a single block to reduce memory
        // access.
        data            = new float[rows * columns];
        isHeapAllocated = true;
    }
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            data[i * columns + j] = 0;
        }
    }
}

Matrix::Matrix(int n_rows, int n_cols, const float values[])
{
    rows    = n_rows;
    columns = n_cols;
    if (rows * columns <= MAX_STACK_MATRIX_SIZE)
    {
        // The class has a fixed size array allocated on the stack.
        // If its size is enough to contain all the elements it won't use
        // dynamic allocation, speeding things up considerably
        data            = stackData;
        isHeapAllocated = false;
    }
    else
    {
        // If the stack allocated array is too small we need to allocate it in
        // the heap The allocation is done in a single block to reduce memory
        // access
        data            = new float[rows * columns];
        isHeapAllocated = true;
    }
    for (int i = 0; i < rows * columns; i++)
    {
        data[i] = values[i];
    }
}

Matrix::Matrix(const Matrix& M)
{
    rows    = M.rows;
    columns = M.columns;
    if (rows * columns <= MAX_STACK_MATRIX_SIZE)
    {
        // The class has a fixed size array allocated on the stack.
        // If its size is enough to contain all the elements it won't use
        // dynamic allocation, speeding things up considerably
        data            = stackData;
        isHeapAllocated = false;
    }
    else
    {
        // If the stack allocated array is too small we need to allocate it in
        // the heap The allocation is done in a single block to reduce memory
        // access
        data            = new float[rows * columns];
        isHeapAllocated = true;
    }
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            data[i * columns + j] = M.data[i * columns + j];
        }
    }
}

Matrix::~Matrix()
{
    // If the pointer was set to nullptr means that the data is now used by
    // another object and we don't need to deallocate it Also we have to check
    // if we're useing heap allocated memory or if the array on the stack was
    // enough
    if (data && isHeapAllocated)
        delete[] data;
}

Matrix::Matrix(Matrix&& M)
{
    // The mover copies the pointer from M in this.data and sets M.data to
    // nullptr. This has the purpose of avoiding the deallocation of memory that
    // will be used by other objects Example: Matrix A is built inside a
    // function and returned as a result. The result of the function is assigned
    // to Matrix B. Since Matrix A goes out of scope its contents will be
    // deallocated, including the data pointer which is used by Matrix B.
    // Setting the pointer to nullptr disables the deallocation and moves the
    // data from Matrix A to Matrix B.
    rows    = M.rows;
    columns = M.columns;
    if (M.isHeapAllocated)
    {
        data            = M.data;
        isHeapAllocated = true;
    }
    else
    {
        isHeapAllocated = false;
        data            = stackData;
        for (int i = 0; i < M.rows * M.columns; i++)
        {
            data[i] = M.data[i];
        }
    }
    M.data = nullptr;
}

Matrix& Matrix::operator=(Matrix&& M)
{
    // This code has the same functionality as the mover, check the mover
    // implementation for details. The difference is the conditional statement
    // to check if the matrix is assigned to itself ( A = A; ) In that case we
    // don't need to change anything

    // TODO: what if the dimensions do not match?!

    if (this != &M)
    {
        rows    = M.rows;
        columns = M.columns;

        // If the destination matrix has heap allocated data, we release it
        if (isHeapAllocated)
            delete[] data;

        // If the source matrix has heap allocated data, simply copy the pointer
        // Also set the heap allocated property in the destination matrix to
        // true
        if (M.isHeapAllocated)
        {
            data            = M.data;
            isHeapAllocated = true;
        }

        // Otherwise do the opposite: set the property to false
        // And copy the data element by element
        else
        {
            isHeapAllocated = false;
            for (int i = 0; i < M.rows * M.columns; i++)
            {
                data[i] = M.data[i];
            }
        }

        // Set the pointer to null to signal the destructor its job is not
        // needed
        M.data = nullptr;
    }
    return *this;
}

// --- Class methods ---
void Matrix::printMatrix(Matrix M)
{
    for (int i = 0; i < M.rows; i++)
    {
        for (int j = 0; j < M.columns; j++)
        {
            std::cout << std::setprecision(2) << std::fixed
                      << M.data[i * M.columns + j] << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

Matrix Matrix::eye(int n)
{
    Matrix eye{n, n};
    for (int i = 0; i < n; i++)
    {
        eye.data[i * eye.columns + i] = 1;
    }
    return eye;
}

Matrix Matrix::multiply(Matrix A, Matrix B)
{
    if (A.columns == 1 && A.rows == 1)
    {
        // In case this matrix is a scalar
        Matrix result{B.rows, B.columns};
        for (int i = 0; i < result.rows; i++)
        {
            for (int j = 0; j < result.columns; j++)
            {
                result.data[i * result.columns + j] =
                    B.data[i * B.columns + j] * A.data[0];
            }
        }
        return B;
    }

    else if (B.columns == 1 && B.rows == 1)
    {
        // In case the B matrix is a scalar
        Matrix result{A.rows, A.columns};
        for (int i = 0; i < A.rows; i++)
        {
            for (int j = 0; j < A.columns; j++)
            {
                result.data[i * result.columns + j] =
                    A.data[i * A.columns + j] * B.data[0];
            }
        }
        return result;
    }
    else if (A.columns != B.rows)
    {
        // If neither of the multipliers is a scalar, matrix dimensions must
        // agree
        throw std::invalid_argument("Matrix dimensions do not match");
    }
    else
    {
        Matrix result{A.rows, B.columns};
        for (int i = 0; i < result.rows; i++)
        {
            for (int j = 0; j < result.columns; j++)
            {
                for (int k = 0; k < A.columns; k++)
                {
                    result.data[i * result.columns + j] +=
                        A.data[i * A.columns + k] * B.data[k * B.columns + j];
                }
            }
        }
        return result;
    }
}

Matrix Matrix::sum(Matrix A, Matrix B)
{
    if (A.rows != B.rows || A.columns != B.columns)
    {
        throw std::invalid_argument("Matrix dimensions do not match");
    }
    Matrix result{A.rows, A.columns};
    for (int i = 0; i < A.rows; i++)
    {
        for (int j = 0; j < A.columns; j++)
        {
            result.data[i * result.columns + j] =
                A.data[i * A.columns + j] + B.data[i * B.columns + j];
        }
    }
    return result;
}

Matrix Matrix::subtract(Matrix A, Matrix B)
{
    if (A.rows != B.rows || A.columns != B.columns)
    {
        throw std::invalid_argument("Matrix dimensions do not match");
    }
    Matrix result{A.rows, A.columns};
    for (int i = 0; i < A.rows; i++)
    {
        for (int j = 0; j < A.columns; j++)
        {
            result.data[i * result.columns + j] =
                A.data[i * A.columns + j] - B.data[i * B.columns + j];
        }
    }
    return result;
}

// --- Instance methods ---
void Matrix::set(float values[])
{
    for (int i = 0; i < rows * columns; i++)
    {
        data[i] = values[i];
    }
}

void Matrix::sum(Matrix B)
{
    if (rows != B.rows || columns != B.columns)
    {
        throw std::invalid_argument("Matrix dimensions do not match");
    }
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            data[i * columns + j] =
                data[i * columns + j] + B.data[i * B.columns + j];
        }
    }
}

Matrix Matrix::transposed()
{
    Matrix new_matrix{columns, rows};
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            new_matrix.data[j * new_matrix.columns + i] = data[i * columns + j];
        }
    }
    return new_matrix;
}

void Matrix::subtract(Matrix B)
{
    if (rows != B.rows || columns != B.columns)
    {
        throw std::invalid_argument("Matrix dimensions do not match");
    }
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            data[i * columns + j] =
                data[i * columns + j] - B.data[i * B.columns + j];
        }
    }
}

Matrix Matrix::inverse()
{
    if (columns == 1 && rows == 1)
    {
        Matrix result{1, 1};
        result.data[0] = 1 / data[0];
        return result;
    }
    float det = determinant();
    if (det == 0)
    {
        throw std::invalid_argument("Matrix is singular");
    }

    Matrix det_matrix{1, 1};
    det_matrix.data[0] = 1 / det;
    return Matrix::multiply(((*this).cofactorMatrix().transposed()),
                            det_matrix);
}

float Matrix::determinant()
{
    // Decomposing the matrix ito two triangular matrices
    Matrix L{rows, columns};
    Matrix U{rows, columns};
    luDecomposition(*this, L, U);
    printMatrix(L);
    printMatrix(U);

    // The determinant of a triangular matrix is the product of the elements on
    // its diagonal
    float detL = 1;
    float detU = 1;
    for (int i = 0; i < rows; i++)
    {
        detL = detL * L.data[i * columns + i];
        detU = detU * U.data[i * columns + i];
    }

    // The determinant of the product of two matrices is the product of the
    // determinants
    return detL * detU;
}

float Matrix::minor(int a, int b)
{
    Matrix minor_matrix{rows - 1, columns - 1};
    for (int i = 0; i < minor_matrix.rows; i++)
    {
        for (int j = 0; j < minor_matrix.columns; j++)
        {
            int h, k;
            if (i < a)
                h = i;
            else
                h = i + 1;
            if (j < b)
                k = j;
            else
                k = j + 1;
            minor_matrix.data[i * minor_matrix.columns + j] =
                data[h * columns + k];
        }
    }
    return minor_matrix.determinant();
}

Matrix Matrix::cofactorMatrix()
{
    Matrix cofactor_matrix{rows, columns};
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            cofactor_matrix.data[i * cofactor_matrix.columns + j] =
                (*this).minor(i, j);
        }
    }
    return cofactor_matrix;
}

void Matrix::luDecomposition(Matrix M, Matrix& lower, Matrix& upper)
{
    int n = M.rows;

    // Decomposing matrix into Upper and Lower
    // triangular matrix
    for (int i = 0; i < n; i++)
    {

        // Upper Triangular
        for (int k = i; k < n; k++)
        {

            // Summation of L(i, j) * U(j, k)
            int sum = 0;
            for (int j = 0; j < i; j++)
                sum += (lower.data[i * n + j] * upper.data[j * n + k]);

            // Evaluating U(i, k)
            upper.data[i * n + k] = M.data[i * n + k] - sum;
        }

        // Lower Triangular
        for (int k = i; k < n; k++)
        {
            if (i == k)
                lower.data[i * n + i] = 1;  // Diagonal as 1
            else
            {

                // Summation of L(k, j) * U(j, i)
                int sum = 0;
                for (int j = 0; j < i; j++)
                    sum += (lower.data[k * n + j] * upper.data[j * n + i]);

                // Evaluating L(k, i)
                lower.data[k * n + i] =
                    (M.data[k * n + i] - sum) / upper.data[i * n + i];
            }
        }
    }
}

// --- Operator overload ---
Matrix Matrix::operator+(const Matrix& B) { return Matrix::sum(*this, B); }

Matrix Matrix::operator-(const Matrix& B) { return Matrix::subtract(*this, B); }

Matrix Matrix::operator*(const Matrix& B) { return Matrix::multiply(*this, B); }

float& Matrix::operator()(int i, int j) { return data[i * columns + j]; }

float& Matrix::operator()(int i) { return data[i]; }
