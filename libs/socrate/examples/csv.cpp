/*
Simple example, showing the creation of csv printer for arrays, based on
reflector.
*/

#include "../include/csv.hpp"

#include <iostream>
#include <vector>

struct Foo
{
    Foo(int a, float b) : a(a), b(b) {}

    int a;
    float b;

    constexpr static auto reflect()
    {
        return STRUCT_DEF(Foo, FIELD_DEF(a) FIELD_DEF(b));
    }
};

struct Bar : Foo
{
    Bar(int a, float b, char c, bool d) : Foo(a, b), c(c), d(d) {}

    char c;
    bool d;

    constexpr static auto reflect()
    {
        return STRUCT_DEF(Bar, EXTEND_DEF(Foo) FIELD_DEF(c) FIELD_DEF(d));
    }
};

int main()
{
    // std::cout << std::boolalpha;

    std::vector<Bar> rows = {Bar{10, 30.0f, 'a', true},
                             Bar{20, 35.0f, 'b', false}};

    csv::print(std::cout, rows);

    return 0;
}
