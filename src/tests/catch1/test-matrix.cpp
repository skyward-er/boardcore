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

#ifdef STANDALONE_CATCH1_TEST
#include  "catch1-tests-entry.cpp"
#endif

#include  <catch.hpp>
#include "kalman/Matrix.hpp"


/** TESTS LIST
 * Constructors
 * Static: eye
 * Static: multiply         OK  
 * Static: sum              OK
 * Static: subtract         OK
 * Static: transpose        OK
 * Static: inverse          OK
 * Static: minor
 * Static: cofactorMatrix
 * Static: determinant      OK
 * Static: LU decomposition OK
 * set
*/
TEST_CASE("Multiply test")
{
    float dataA[] = { 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    Matrix A{3,3, dataA};
    float dataB[] = { 1.0f, 2.0f, 3.7f};
    Matrix B{3,1, dataB};
    Matrix C{3,1};
    Matrix::multiply(A, B, C);

    REQUIRE( C.rows    == 3 );
    REQUIRE( C.columns == 1 );

    REQUIRE( C(0) == Approx( 8.1f)  );
    REQUIRE( C(1) == Approx(-7.7f)  );
    REQUIRE( C(2) == Approx(-10.3f) );

    // Wrong dimensions
    Matrix D{2,2};
    REQUIRE( Matrix::multiply(A, D, A) == false );
    REQUIRE( Matrix::multiply(B, B, B) == false );
}

TEST_CASE("Sum test")
{
    float dataA[] = { 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    Matrix A{3,3, dataA};
    float dataB[] = { 1.0f, 2.0f, 3.0f, 5.0f, 7.0f, 9.0f, 11.0f, 13.7f, 15.0f};
    Matrix B{3,3, dataB};

    // Static
    Matrix C{A.rows, A.columns};
    Matrix::sum(A, B, C);

    REQUIRE( C.rows    == 3 );
    REQUIRE( C.columns == 3 );

    REQUIRE( C(0,0) == Approx(2.0f ));
    REQUIRE( C(0,1) == Approx(0.0f ));
    REQUIRE( C(0,2) == Approx(6.0f ));
    REQUIRE( C(1,0) == Approx(9.5f ));
    REQUIRE( C(1,1) == Approx(12.0f));
    REQUIRE( C(1,2) == Approx(3.0f ));
    REQUIRE( C(2,0) == Approx(18.0f));
    REQUIRE( C(2,1) == Approx(21.7f));
    REQUIRE( C(2,2) == Approx(6.0f ));

    // Wrong dimensions
    B = Matrix(2,2);
    REQUIRE( Matrix::sum(A, B, A) == false );
    REQUIRE( Matrix::sum(A, A, B) == false );
}

TEST_CASE("Subtract test")
{
    float dataA[] = { 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    Matrix A{3,3, dataA};
    float dataB[] = { 1.0f, 2.0f, 3.0f, 5.0f, 7.0f, 9.0f, 11.0f, 13.7f, 15.0f};
    Matrix B{3,3, dataB};

    // Static
    Matrix C{A.rows, A.columns};
    Matrix::subtract(A, B, C);

    REQUIRE( C.rows    == 3 );
    REQUIRE( C.columns == 3 );

    REQUIRE( C(0,0) == Approx(0.0f  ));
    REQUIRE( C(0,1) == Approx(-4.0f ));
    REQUIRE( C(0,2) == Approx(0.0f  ));
    REQUIRE( C(1,0) == Approx(-0.5f ));
    REQUIRE( C(1,1) == Approx(-2.0f ));
    REQUIRE( C(1,2) == Approx(-15.0f));
    REQUIRE( C(2,0) == Approx(-4.0f ));
    REQUIRE( C(2,1) == Approx(-5.7f ));
    REQUIRE( C(2,2) == Approx(-24.0f));

    // Wrong dimensions
    B = Matrix(2,2);
    REQUIRE( Matrix::subtract(A, B, A) == false );
    REQUIRE( Matrix::subtract(A, A, B) == false );
}

TEST_CASE("Transpose test")
{
    float dataA[] = { 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    Matrix A = Matrix(3,3, dataA);
    Matrix::transpose(A, A);

    REQUIRE( A.rows    == 3);
    REQUIRE( A.columns == 3);

    REQUIRE( A(0,0) == Approx( 1.0f ));
    REQUIRE( A(0,1) == Approx( 4.5f ));
    REQUIRE( A(0,2) == Approx( 7.0f ));
    REQUIRE( A(1,0) == Approx(-2.0f ));
    REQUIRE( A(1,1) == Approx( 5.0f ));
    REQUIRE( A(1,2) == Approx( 8.0f ));
    REQUIRE( A(2,0) == Approx( 3.0f ));
    REQUIRE( A(2,1) == Approx(-6.0f ));
    REQUIRE( A(2,2) == Approx(-9.0f ));

    float dataB[] = { 1, 2, 3.7};
    Matrix B{3,1, dataB};
    Matrix C{1,3};
    Matrix::transpose(B, C);

    REQUIRE( C.rows    == 1);
    REQUIRE( C.columns == 3);
    
    REQUIRE( C(0) == Approx(1  ));
    REQUIRE( C(1) == Approx(2  ));
    REQUIRE( C(2) == Approx(3.7));

    // Wrong dimensions
    REQUIRE( Matrix::transpose(A, B) == false);
}

TEST_CASE("LU Decomposition test") {
    float dataA[] = { 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    Matrix A = Matrix(3,3, dataA);
    Matrix L{3,3};
    Matrix U{3,3};
    Matrix::luDecomposition(A, L, U);

    REQUIRE( L(0,0) == Approx( 1.0f ));
    REQUIRE( L(0,1) == Approx( 0.0f ));
    REQUIRE( L(0,2) == Approx( 0.0f ));
    REQUIRE( L(1,0) == Approx( 4.5f ));
    REQUIRE( L(1,1) == Approx( 1.0f ));
    REQUIRE( L(1,2) == Approx( 0.0f ));
    REQUIRE( L(2,0) == Approx( 7.0f ));
    REQUIRE( L(2,1) == Approx( 1.57f).epsilon(0.01));
    REQUIRE( L(2,2) == Approx( 1.0f ));

    REQUIRE( U(0,0) == Approx( 1.0f ));
    REQUIRE( U(0,1) == Approx(-2.0f ));
    REQUIRE( U(0,2) == Approx( 3.0f ));
    REQUIRE( U(1,0) == Approx( 0.0f ));
    REQUIRE( U(1,1) == Approx( 14.0f));
    REQUIRE( U(1,2) == Approx(-19.5f));
    REQUIRE( U(2,0) == Approx( 0.0f ));
    REQUIRE( U(2,1) == Approx( 0.0f ));
    REQUIRE( U(2,2) == Approx( 0.64f).epsilon(0.01));

    Matrix L2{1,3};
    REQUIRE( Matrix::luDecomposition(A, L2, U) == false );
    REQUIRE( Matrix::luDecomposition(A, L, L2) == false );
}

TEST_CASE("Determinant test") {
    float dataA[] = { 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    Matrix A{3,3, dataA};
    float det;
    Matrix::determinant(A, det);
    REQUIRE( det == Approx( 9.0 ));

    Matrix B{5, 4};
    REQUIRE( Matrix::determinant(B, det) == false);

    float dataC[] = { 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f, 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f, 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f };
    Matrix C{5, 5, dataC};
    Matrix::determinant(C, det);
    REQUIRE( det == Approx(-46544.0f));
}

TEST_CASE("Inverse test") {
    float dataA[] = { 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    Matrix A = Matrix(3,3, dataA);
    Matrix::invert(A, A);

    REQUIRE( A.rows    == 3);
    REQUIRE( A.columns == 3);

    REQUIRE( A(0,0) == Approx( 0.33f ).epsilon(0.01));
    REQUIRE( A(0,1) == Approx( 0.66f ).epsilon(0.01));
    REQUIRE( A(0,2) == Approx(-0.33f ).epsilon(0.01));
    REQUIRE( A(1,0) == Approx(-0.16f ).epsilon(0.01));
    REQUIRE( A(1,1) == Approx(-3.33f ).epsilon(0.01));
    REQUIRE( A(1,2) == Approx( 2.16f ).epsilon(0.01));
    REQUIRE( A(2,0) == Approx( 0.11f ).epsilon(0.01));
    REQUIRE( A(2,1) == Approx(-2.44f ).epsilon(0.01));
    REQUIRE( A(2,2) == Approx( 1.55f ).epsilon(0.01));

    Matrix B{1, 5};
    REQUIRE( Matrix::invert(A, B) == false);
    REQUIRE( Matrix::invert(B, B) == false);
}