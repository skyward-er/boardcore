#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#define CATCH_CONFIG_NO_POSIX_SIGNALS
#define CATCH_CONFIG_NO_CPP11_GENERATED_METHODS
#define CATCH_CONFIG_NO_CPP11_TYPE_TRAITS

#include <catch.hpp>

int Factorial( int number ) {

   return number <= 1 ? number : Factorial( number - 1 ) * number;  // fail

// return number <= 1 ? 1      : Factorial( number - 1 ) * number;  // pass

}


TEST_CASE( "Factorial of 0 is 1 (fail)", "[single-file]" ) {

    REQUIRE( Factorial(0) == 1 );

}


TEST_CASE( "Factorials of 1 and higher are computed (pass)", "[single-file]" ) {

    REQUIRE( Factorial(1) == 1 );

    REQUIRE( Factorial(2) == 2 );

    REQUIRE( Factorial(3) == 6 );

    REQUIRE( Factorial(10) == 3628800 );

}