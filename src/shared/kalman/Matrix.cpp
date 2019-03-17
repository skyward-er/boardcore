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

#include "Matrix.h"
#include <iomanip>
#include <iostream>

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

Matrix::Matrix(int n_rows, int n_cols, float values[])
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
        // the heap. The allocation is done in a single block to reduce memory
        // access.
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

Matrix Matrix::operator=(const Matrix& M) 
{
    Matrix A{M.rows, M.columns};
    if (A.rows * A.columns <= MAX_STACK_MATRIX_SIZE)
    {
        // The class has a fixed size array allocated on the stack.
        // If its size is enough to contain all the elements it won't use
        // dynamic allocation, speeding things up considerably
        A.data            = A.stackData;
        A.isHeapAllocated = false;
    }
    else
    {
        // If the stack allocated array is too small we need to allocate it in
        // the heap. The allocation is done in a single block to reduce memory
        // access.
        A.data            = new float[A.rows * A.columns];
        A.isHeapAllocated = true;
    }
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            A.data[i * A.columns + j] = M.data[i * M.columns + j];
        }
    }
    return A;
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

Matrix Matrix::eye(int n)
{
    Matrix eye{n, n};
    for (int i = 0; i < n; i++)
    {
        eye.data[i * eye.columns + i] = 1;
    }
    return eye;
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

bool Matrix::multiply(Matrix A, Matrix B, Matrix& result)
{
    if (A.columns != B.rows || result.rows != A.rows || result.columns != B.columns)
    {
        return false;
    }
    else
    {
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
        return true;
    }
}

bool Matrix::multiply(Matrix A, float b, Matrix& result)
{
    if ( result.rows != A.rows || result.columns != A.columns)
    {
        return false;
    }
    else
    {
        for (int i = 0; i < result.rows; i++)
        {
            for (int j = 0; j < result.columns; j++)
            {
                result.data[i * result.columns + j] = A.data[i * A.columns + j]*b;
            }
        }
        return true;
    }
}

bool Matrix::sum(Matrix A, Matrix B, Matrix& result)
{
    if (A.rows != B.rows || A.columns != B.columns || A.rows != result.rows || A.columns != result.columns)
    {
        return false;
    }
    else
    {
        for(int i = 0; i < A.rows; i++)
        {
            for(int j = 0; j < A.columns; j++)
            {
                result.data[i * result.columns + j] = A.data[i * A.columns + j] + B.data[i * B.columns + j];
            }
        }
        return true;
    }
}

bool Matrix::subtract(Matrix A, Matrix B, Matrix& result)
{
    if (A.rows != B.rows || A.columns != B.columns || A.rows != result.rows || A.columns != result.columns)
    {
        return false;
    }
    else
    {
        for(int i = 0; i < A.rows; i++)
        {
            for(int j = 0; j < A.columns; j++)
            {
                result.data[i * result.columns + j] = A.data[i * A.columns + j] - B.data[i * B.columns + j];
            }
        }
        return true;
    }
}

bool Matrix::transpose(Matrix A, Matrix& result)
{
    if (A.rows != result.columns || A.columns != result.rows) {
        return false;
    }
    else {
        for (int i = 0; i < A.rows; i++)
        {
            for (int j = 0; j < A.columns; j++)
            {
                result.data[j * result.columns + i] = A.data[i * A.columns + j];
            }
        }
        return true;
    }
}

bool Matrix::invert(Matrix A, Matrix& result)
{
    // std:cout << "ME HERE! \n";
    float det;
    if (!determinant(A, det) || det == 0)
    {
        return false;
    }
    else
    {
        Matrix B{A.rows, A.columns};
        if (!cofactorMatrix(A, B))
            return false;
        if (!transpose(B, B))
            return false;
        if (!multiply(B, 1/det, result))
            return false;
        return true;
    }
}

bool Matrix::determinant(Matrix A, float& result)
{

    // If the matrix is not square the determinant doesn't exist
    if (A.rows != A.columns) {
        return false;
    }

    // Decomposing the matrix ito two triangular matrices
    Matrix L{A.rows, A.columns};
    Matrix U{A.rows, A.columns};
    
    if (!luDecomposition(A, L, U))
        return false;

    // The determinant of a triangular matrix is the product of the elements on
    // its diagonal
    float detL = 1.0f;
    float detU = 1.0f;
    for (int i = 0; i < A.rows; i++)
    {
        detL = detL * L.data[i * L.columns + i];
        detU = detU * U.data[i * U.columns + i];
    }

    // The determinant of the product of two matrices is the product of the
    // determinants
    result = detL * detU;
    return true;
}

bool Matrix::minor(Matrix A, int a, int b, float& result)
{
    if (A.rows != A.columns)
    {
        return false;
    }
    Matrix minor_matrix{A.rows - 1, A.columns - 1};
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
                A.data[h * A.columns + k];
        }
    }
    return determinant(minor_matrix, result);
}

bool Matrix::cofactorMatrix(Matrix A, Matrix& result)
{
    if (A.rows != result.rows || A.columns != result.columns || A.rows != A.columns) 
    {
        return false;
    }
    for (int i = 0; i < A.rows; i++)
    {
        for (int j = 0; j < A.columns; j++)
        {
            float min;
            if (!minor(A, i, j, min))
            {
                return false;
            }
            if ((i+j)%2 == 0) {
                result.data[i * result.columns + j] = min;
            }
            else
            {
                result.data[i * result.columns + j] = -min;
            }
            
        }
    }
    return true;
}

bool Matrix::luDecomposition(Matrix M, Matrix& lower, Matrix& upper)
{
    int n = M.rows;
    if (M.columns != n || lower.rows != n || lower.columns != n || upper.rows != n || upper.columns != n)
    {
        return false;
    }
    // Decomposing matrix into Upper and Lower
    // triangular matrix
    for (int i = 0; i < n; i++)
    {

        // Upper Triangular
        for (int k = i; k < n; k++)
        {

            // Summation of L(i, j) * U(j, k)
            float sum = 0;
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
                float sum = 0;
                for (int j = 0; j < i; j++)
                    sum += (lower.data[k * n + j] * upper.data[j * n + i]);

                // Evaluating L(k, i)
                lower.data[k * n + i] =
                    (M.data[k * n + i] - sum) / upper.data[i * n + i];
            }
        }
    }
    return true;
}
// --- Instance methods ---
void Matrix::set(float values[])
{
    for (int i = 0; i < rows * columns; i++)
    {
        data[i] = values[i];
    }
}

// // --- Operator overload ---
float& Matrix::operator()(int i, int j) { return data[i * columns + j]; }

float& Matrix::operator()(int i) { return data[i]; }