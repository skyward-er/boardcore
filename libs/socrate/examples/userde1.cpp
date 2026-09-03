#include <cstdint>
#include <iostream>

#include "../include/reflect.hpp"
#include "../include/userde.hpp"

using namespace socrate::userde;

struct Custom
{
    float x, y, z;
};

template <>
struct socrate::userde::Serde<Custom, void>
{
    static constexpr size_t size() { return sizeof(float) * 3; }

    static void serialize(const Custom& value, Stream& stream)
    {
        stream.write(&value.x, sizeof(float));
        stream.write(&value.y, sizeof(float));
        stream.write(&value.z, sizeof(float));
    }

    static void deserialize(Custom& value, Stream& stream)
    {
        stream.read(&value.x, sizeof(float));
        stream.read(&value.y, sizeof(float));
        stream.read(&value.z, sizeof(float));
    }
};

struct VeryInner
{
    int fifth;
};

struct Inner
{
    VeryInner inner;
};

struct Foo
{
    uint32_t first;
    float32_t second;
    int third[4];
    Custom fourth;
    Inner inner;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(Foo, FIELD_DEF(first) FIELD_DEF(second)
                                   FIELD_DEF(third) FIELD_DEF(fourth)
                                       FIELD_DEF3(inner, inner, fifth));
    }

    static constexpr auto reflect2()
    {
        return socrate::reflect::struct_def<Foo>("Foo")
            .add_field("first", &Foo::first)
            .add_field("second", &Foo::second)
            .add_field("third", &Foo::third)
            .add_field("fourth", &Foo::fourth)
            .add_field("fifth", &Foo::inner, &Inner::inner, &VeryInner::fifth);
    }
};

std::ostream& operator<<(std::ostream& stream, const Custom& value)
{
    stream << "x: " << value.x << " y: " << value.y << " z: " << value.z;
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const int (&value)[4])
{
    stream << "0: " << value[0];
    for (int i = 1; i < 4; i++)
        stream << " " << i << ": " << value[i];
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const Inner& value)
{
    stream << "inner.inner: " << value.inner.fifth;
    return stream;
}

template <typename T>
void debug(std::ostream& output, T& value)
{
    constexpr auto reflect = T::reflect();

    output << reflect.type_name() << " {" << std::endl;

    reflect.for_each_field(
        value, [&](const char* name, auto& value, auto _type)
        { output << "  " << name << ": " << value << ";" << std::endl; });

    output << "}" << std::endl;
}

int main()
{
    // Fixed buffer size, you can also use Layout::of
    constexpr size_t SIZE = 64;
    uint8_t buffer[SIZE];

    Foo before{10, 20.0f, {2, 4, 8, 16}, {45.0f, 50.0f, 55.0f}, {{80}}};

    std::cout << "Before: ";
    debug(std::cout, before);

    // Serialize the data
    auto ret = serialize_with_name<Foo>(before, buffer, SIZE);
    if (ret != Error::Success)
        std::cout << "Failure: " << error_to_msg(ret);

    // Print the buffer
    std::cout << "Buffer: [" << std::hex;
    for (int i = 0; i < 12; i++)
        std::cout << " 0x" << (int)buffer[i];
    std::cout << std::dec << " ]" << std::endl;

    Foo after;

    // Deserialize the data
    ret = deserialize_with_name<Foo>(after, buffer, SIZE);
    if (ret != Error::Success)
        std::cout << "Failure: " << error_to_msg(ret);

    std::cout << "After: ";
    debug(std::cout, after);

    return 0;
}
