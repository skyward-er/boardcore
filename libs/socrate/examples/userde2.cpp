#include <cstdint>
#include <iostream>

#include "../include/reflect.hpp"

#define USERDE_STREAM_DESERIALIZER
#include "../include/userde.hpp"

using namespace socrate::userde;

struct Foo
{
    uint32_t first;
    float32_t second;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(Foo, FIELD_DEF(first) FIELD_DEF(second));
    }
};

struct Bar
{
    float32_t third;
    uint32_t fourth;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(Bar, FIELD_DEF(third) FIELD_DEF(fourth));
    }
};

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
    constexpr size_t SIZE = 32;
    uint8_t buffer[SIZE];

    Foo foo{10, 20.0f};
    Bar bar{30.0f, 40};

    // Serialize the data
    auto ret = serialize_with_name<Foo>(foo, buffer, SIZE);
    if (ret != Error::Success)
        std::cout << "Failure: " << error_to_msg(ret);

    ret = serialize_with_name<Bar>(bar, buffer + 12, SIZE);
    if (ret != Error::Success)
        std::cout << "Failure: " << error_to_msg(ret);

    // Print the buffer
    std::cout << "Buffer: [" << std::hex;
    for (int i = 0; i < 24; i++)
        std::cout << " 0x" << (int)buffer[i];
    std::cout << std::dec << " ]" << std::endl;

    // Create deserializer
    StreamDeserializer deserializer;

    deserializer.register_type<Foo>([](Foo& value)
                                    { debug(std::cout, value); });
    deserializer.register_type<Bar>([](Bar& value)
                                    { debug(std::cout, value); });

    MemStream stream(buffer, 24);
    deserializer.deserialize(stream);

    return 0;
}
