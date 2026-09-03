
#include "../include/reflect.hpp"

// Non reflectable
struct A {
    int a;
};

struct B {
    int b;

    static constexpr auto reflect() {
        return STRUCT_DEF(B, FIELD_DEF(b));
    }
};

struct C : public B {
    int b;

    static constexpr auto reflect() {
        return STRUCT_DEF(C, EXTEND_DEF(B) FIELD_DEF(b));
    }
};

static_assert(!socrate::reflect::is_reflectable<A>, "A must be not reflectable!");
static_assert(socrate::reflect::is_reflectable<B>, "B must be reflectable!");
static_assert(socrate::reflect::is_reflectable<C>, "C must be reflectable!");

int main() {
    return 0;
}