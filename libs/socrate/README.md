# Socrate
Skyward API for reflecting structs and classes.

Goals:
- simple to use
- ~~just the bare minimum~~ (well this is no longer the case)
- single header
- ~~not too obscure~~ (well this is no longer the case)
- easy to automatically generate (maybe maybe...)

## 

## Example
```C++
#include "reflect.hpp"

using namespace Skyward;

// Without the usage of any macro
namespace no_macros {

struct Inner {
    char c;
};

struct Foo {
    int a;
    float b;
    Inner inner;

    constexpr static auto reflect() {
        return struct_def<Foo>("Foo")
            .add_field("a", &Foo::a)
            .add_field("b", &Foo::b)
            .add_field("c", &Foo::inner, &Inner::c);
    }
};

struct Bar : Foo {
    int c;
    float d;

    constexpr static auto reflect() {
        return struct_def<Bar>("Bar")
            .add_extends<Foo>()
            .add_field("c", &Bar::c)
            .add_field("d", &Bar::d);
    }
};

}

// With custom helper macros
namespace macros {

struct Inner {
    char c;
};

struct Foo {
    int a;
    float b;
    Inner inner;

    constexpr static auto reflect() {
        return STRUCT_DEF(Foo,
            FIELD_DEF(a)
            FIELD_DEF(b)
            FIELD_DEF2(inner, c));
    }
};

struct Bar : Foo {
    int c;
    float d;

    constexpr static auto reflect() {
        return STRUCT_DEF(Bar,
            EXTENDS_DEF(Foo)
            FIELD_DEF(c)
            FIELD_DEF(d));
    }
};

}
```

# μ-serde
> If you know, you know...

Microscopic serialization framework based on Socrate.

Define `USERDE_STREAM_DESERIALIZER` before including the header if you want to enable the _cool_ stream deserializer, disabled by default because it includes some really big libraries.

Serialization format is basically identical to the one used in tscpp, with the advantage of custom, stable, type names and compressed padding bytes.