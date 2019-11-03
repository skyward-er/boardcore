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
#include  "catch-tests-entry.cpp"
#endif

#include  <utils/testutils/catch.hpp>
#include <libs/simple-template-matrix/matrix.h>


TEST_CASE("Multiply test")
{
    MatrixBase<float, 3, 3> A{1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f};
    MatrixBase<float, 3, 1> B{1.0f, 2.0f, 3.7f};
    MatrixBase<float, 3, 1> C{};

    REQUIRE( C(0,0) == Approx( 8.1f)  );
    REQUIRE( C(1,0) == Approx(-7.7f)  );
    REQUIRE( C(2,0) == Approx(-10.3f) );
}

TEST_CASE("Sum test")
{
    MatrixBase<float, 3,3> A{1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f};
    MatrixBase<float, 3, 3> B{ 1.0f, 2.0f, 3.0f, 5.0f, 7.0f, 9.0f, 11.0f, 13.7f, 15.0f};

    MatrixBase<float, 3, 3> C{};

    REQUIRE( C(0,0) == Approx(2.0f ));
    REQUIRE( C(0,1) == Approx(0.0f ));
    REQUIRE( C(0,2) == Approx(6.0f ));
    REQUIRE( C(1,0) == Approx(9.5f ));
    REQUIRE( C(1,1) == Approx(12.0f));
    REQUIRE( C(1,2) == Approx(3.0f ));
    REQUIRE( C(2,0) == Approx(18.0f));
    REQUIRE( C(2,1) == Approx(21.7f));
    REQUIRE( C(2,2) == Approx(6.0f ));
}

TEST_CASE("Subtract test")
{
    MatrixBase<float, 3, 3> A{ 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    MatrixBase<float, 3, 3> B{ 1.0f, 2.0f, 3.0f, 5.0f, 7.0f, 9.0f, 11.0f, 13.7f, 15.0f};

    MatrixBase<float, 3, 3> C{};

    REQUIRE( C(0,0) == Approx(0.0f  ));
    REQUIRE( C(0,1) == Approx(-4.0f ));
    REQUIRE( C(0,2) == Approx(0.0f  ));
    REQUIRE( C(1,0) == Approx(-0.5f ));
    REQUIRE( C(1,1) == Approx(-2.0f ));
    REQUIRE( C(1,2) == Approx(-15.0f));
    REQUIRE( C(2,0) == Approx(-4.0f ));
    REQUIRE( C(2,1) == Approx(-5.7f ));
    REQUIRE( C(2,2) == Approx(-24.0f));
}

TEST_CASE("Transpose test")
{
    MatrixBase <float, 3, 3> A{ 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };

    A = transpose(A);

    REQUIRE( A(0,0) == Approx( 1.0f ));
    REQUIRE( A(0,1) == Approx( 4.5f ));
    REQUIRE( A(0,2) == Approx( 7.0f ));
    REQUIRE( A(1,0) == Approx(-2.0f ));
    REQUIRE( A(1,1) == Approx( 5.0f ));
    REQUIRE( A(1,2) == Approx( 8.0f ));
    REQUIRE( A(2,0) == Approx( 3.0f ));
    REQUIRE( A(2,1) == Approx(-6.0f ));
    REQUIRE( A(2,2) == Approx(-9.0f ));

    MatrixBase <float, 3, 1> B{ 1, 2, 3.7};
    MatrixBase <float, 1, 3> C;

    C = transpose(B);
    
    REQUIRE( C(0,0) == Approx(1  ));
    REQUIRE( C(0,1) == Approx(2  ));
    REQUIRE( C(0,2) == Approx(3.7));
}

TEST_CASE("Determinant test") {
    MatrixBase<float, 3, 3> A{ 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    REQUIRE( det(A) == Approx( 9.0 ));
}

TEST_CASE("Inverse test") {
    MatrixBase<float, 3, 3>  A{ 1.0f, -2.0f, 3.0f, 4.5f, 5.0f, -6.0f, 7.0f, 8.0f, -9.0f };
    
    A = inv(A);

    REQUIRE( A(0,0) == Approx( 0.33f ).epsilon(0.01));
    REQUIRE( A(0,1) == Approx( 0.66f ).epsilon(0.01));
    REQUIRE( A(0,2) == Approx(-0.33f ).epsilon(0.01));
    REQUIRE( A(1,0) == Approx(-0.16f ).epsilon(0.01));
    REQUIRE( A(1,1) == Approx(-3.33f ).epsilon(0.01));
    REQUIRE( A(1,2) == Approx( 2.16f ).epsilon(0.01));
    REQUIRE( A(2,0) == Approx( 0.11f ).epsilon(0.01));
    REQUIRE( A(2,1) == Approx(-2.44f ).epsilon(0.01));
    REQUIRE( A(2,2) == Approx( 1.55f ).epsilon(0.01));
}