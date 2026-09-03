#pragma once

#include <iostream>
#include <vector>

#include "reflect.hpp"

// ================================
//            CSV Printer
// ================================

namespace csv
{
template <typename T>
constexpr void header(std::ostream& output)
{
    bool first = true;
    T::reflect().for_each_field_type(
        [&](const char* name, auto _type)
        {
            if (!first)
                output << ",";

            output << name;
            first = false;
        });

    output << '\n';
}

template <typename T>
constexpr void row(std::ostream& output, T& value)
{
    bool first = true;
    T::reflect().for_each_field(value,
                                [&](const char* name, auto& field, auto _type)
                                {
                                    if (!first)
                                        output << ",";

                                    output << field;
                                    first = false;
                                });

    output << '\n';
}

template <typename T>
constexpr void print(std::ostream& output, std::vector<T>& rows)
{
    csv::header<T>(output);
    for (auto& row : rows)
        csv::row(output, row);
}
}  // namespace csv
