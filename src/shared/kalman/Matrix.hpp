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

#ifndef Matrix_hpp
#define Matrix_hpp

#include <stdio.h>

#define MAX_STACK_MATRIX_SIZE 9
/*!
 * \brief A class representing a matrix and supporting basic algebra and
 * miscellaneous functions
 *
 * Data is allocated on the stack if it's small enough. It's possible to
 * change the maximum dimension at compile time by changing the define
 * MAX_STACK_MATRIX_SIZE
 */
class Matrix
{
private:
    /**
     * A flag indicating whether the object uses the stack allocated array or an
     * heap allocated one
     * */
    bool isHeapAllocated;

    /**
     * A  stack allocated array that is used if the matrix dimensions are small
     * enough. Speed improves drastically.
     */
    float stackData[MAX_STACK_MATRIX_SIZE];

    /**
     * \brief LU decomposition
     *
     * Decomposes a square matrix into two triangular matrices so that M =
     * lower*upper Source:
     * https://www.geeksforgeeks.org/doolittle-algorithm-lu-decomposition/
     *
     * \param M The matrix to be decomposed
     * \param lower A reference to a matrix with the same dimensions of M. Will
     * be set to the lower triangular matrix \param upper A reference to a
     * matrix with the same dimensions of M. Will be set to the upper triangular
     * matrix
     */
    void luDecomposition(Matrix M, Matrix& lower, Matrix& upper);

public:
    int rows;    /**< The number of rows in the matrix  */
    int columns; /**< The number of columns in the matrix  */
    float* data; /**< Pointer to the data array  */

    /* --- CLASS METHODS --- */

    /**
     * \brief Default constructor
     *
     * This constructor does nothing except setting the data pointer to nullptr
     */
    Matrix();

    /**
     * \brief Constructor
     *
     * Initializes a matrix with all the elements set to zero.
     * \param n_rows The number of rows of the matrix
     * \param n_cols The number of columns of the matrix
     */
    Matrix(int n_rows, int n_cols);

    /**
     * \brief Constructor
     *
     * Initializes a matrix with the elements set to the given values.
     * \param n_rows The number of rows of the matrix
     * \param n_cols The number of columns of the matrix
     * \param values An array of dimensions `n_rows*n_cols` containing all the
     * matrix values ordered left to right, top to bottom.
     */
    Matrix(int n_rows, int n_cols, float values[]);

    /**
     * \brief Copy constructor
     *
     * Initializes a matrix copying values and dimensions from the given matrix
     * \param M A reference to the matrix to be copied
     */
    Matrix(const Matrix& M);

    /**
     * \brief Mover
     *
     * Moves the contents from the given matrix to a new one.
     * If the source uses dynamic allocation this function copies the pointer in
     * M and sets it to nullptr. Otherwise it copies the values in the stack
     * allocated array. \param M A reference to a reference to the matrix that
     * will be moved
     */
    Matrix(Matrix&& M);

    /**
     * \brief Assignment overload
     *
     * Overloads the assigment operator in order to correctly copy the contents
     * of an instance. \param M A reference to a reference to the matrix that
     * will be moved
     */
    Matrix& operator=(Matrix&& M);

    /**
     * \brief Destructor
     *
     * Deallocates the array memory (if the pointer is valid) and dynamic
     * allocation is used.
     */
    virtual ~Matrix();

    /**
     * \brief Builds a n by n eye matrix
     *
     * \param n The dimension of the matrix
     * \return A square matrix with all the elements on the diagonal set to 1
     * and the others set to 0.
     */
    static Matrix eye(int n);

    /**
     * \brief Prints the given matrix
     * \param M The matrix object to print
     */
    static void printMatrix(Matrix M);

    /**
     * \brief Matrix multiplication
     *
     * Multipies the first matrix with the second one and returns the result as
     * a new matrix \param A A matrix with n columns \param B A matrix with n
     * rows \return A new matrix containing the result of `A * B`
     */
    static Matrix multiply(Matrix A, Matrix B);

    /**
     * \brief Matrix sum
     *
     * Sums the two matrices element by element
     * \param A First matrix addend
     * \param B Second matrix addend
     * \return A new matrix of the same dimension of the two addends containing
     * the result of `A + B`
     */
    static Matrix sum(Matrix A, Matrix B);

    /**
     * \brief Matrix subtraction
     *
     * Subtracts matrix B from  matrix A
     * \param A The minuend
     * \param B The subtrahend
     * \return A new matrix of the same dimension of the two passed containing
     * the result of `A - B`
     */
    static Matrix subtract(Matrix A, Matrix B);

    /* --- INSTANCE METHODS --- */

    /**
     * \brief Data setter
     *
     * Sets the data to the given values
     * \param values An array of dimensions `n_rows*n_cols` containing all the
     * matrix values ordered left to right, top to bottom.
     */
    void set(float values[]);

    /**
     * \brief Matrix sum, result placed in caller
     *
     * Sums the two matrices element by element and places the result in *this
     * \param B Second matrix addend
     */
    void sum(Matrix B);

    /**
     * \brief Matrix subtraction, result placed in caller
     *
     * Subtracts matrix B from the given matrix
     * \param B The matrix to be subtracted from `*this`matrix
     */
    void subtract(Matrix B);

    /**
     * \brief Matrix transpose
     *
     * Computes the transposed of the current matrix
     * \return A new matrix with the rows exchanged with the columns
     */
    Matrix transposed();

    /**
     * \brief Matrix inverse
     *
     *  Computes the inverse of the current matrix using the cofactor matrix
     * method. \return A new matrix computed as the inverse of `*this` \warning
     * This method could be slow for large matrices.
     */
    Matrix inverse();

    /**
     * \brief Minors computation
     *
     * Computes the determinant of the matrix obtained removing the i-th row and
     * j-th column \return A float representing the determinant of such matrix
     */
    float minor(int i, int j);

    /**
     * \brief Cofactor matrix
     *
     * Computes the matrix formed by the minors of the given matrix for each
     * element \return A matrix of the same dimensions of `*this` where each
     * element is the minor of the original matrix in that position
     */
    Matrix cofactorMatrix();

    /**
     * \brief Matrix determinant
     *
     * Computes the determinant of a matrix using a LU decomposition
     * \return A float representing the determinant
     */
    float determinant();

    /* --- OPERATORS OVERLOAD --- */
    // FOR NICE SYNTAX :)
    Matrix operator+(const Matrix& B);
    Matrix operator-(const Matrix& B);
    Matrix operator*(const Matrix& B);
    float& operator()(int i, int j);
    float& operator()(int i);
};
#endif /* Matrix_hpp */
