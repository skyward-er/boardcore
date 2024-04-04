/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <utils/ModuleManager/ModuleHolster.h>
#include <utils/ModuleManager/ModuleManager.h>

#include <catch2/catch.hpp>

using namespace Boardcore;

namespace Boardcore
{
class B;

class A : public Module
{
public:
    A() {}

    void bing_a(bool value) { this->value = value; }

    bool bong_a() { return value; }

    void inject(ModuleInjector &getter) { b = getter.get<B>(); }

private:
    B *b       = nullptr;
    bool value = false;
};

class B : public Module
{
public:
    B() {}

    void bing_b(bool value) { this->value = value; }

    bool bong_b() { return value; }

    void inject(ModuleInjector &getter) { a = getter.get<A>(); }

private:
    A *a       = nullptr;
    bool value = false;
};

class CIface
{
public:
    virtual void bing_c() = 0;
    virtual bool bong_c() = 0;
};

class C : public CIface, public ModuleHolster<A, B>
{
public:
    void bing_c() override
    {
        value = getModule<A>()->bong_a() && getModule<B>()->bong_b();
    }

    bool bong_c() override { return value; }

private:
    bool value = false;
};

class D : public Module
{
public:
    void bing_d() { value = c->bong_c(); }

    bool bong_d() { return value; }

    void inject(ModuleInjector &getter) { c = getter.get<CIface>(); }

private:
    CIface *c  = nullptr;
    bool value = false;
};
}  // namespace Boardcore

TEST_CASE("ModuleManager - Circular dependencies")
{
    ModuleManager manager;

    Boardcore::A *a = new Boardcore::A();
    Boardcore::B *b = new Boardcore::B();

    REQUIRE(manager.insert<Boardcore::A>(a));
    REQUIRE(manager.insert<Boardcore::B>(b));
    REQUIRE(manager.inject());

    a->bing_a(true);
    REQUIRE(a->bong_a());

    a->bing_a(false);
    REQUIRE(!a->bong_a());

    b->bing_b(true);
    REQUIRE(b->bong_b());

    b->bing_b(false);
    REQUIRE(!b->bong_b());
}

TEST_CASE("ModuleManager - Virtual Dependencies")
{
    ModuleManager manager;

    Boardcore::A *a = new Boardcore::A();
    Boardcore::B *b = new Boardcore::B();
    Boardcore::C *c = new Boardcore::C();
    Boardcore::D *d = new Boardcore::D();

    REQUIRE(manager.insert<Boardcore::A>(a));
    REQUIRE(manager.insert<Boardcore::B>(b));
    REQUIRE(manager.insert<Boardcore::CIface>(c));
    REQUIRE(manager.insert<Boardcore::D>(d));
    REQUIRE(manager.inject());

    a->bing_a(false);
    b->bing_b(false);

    c->bing_c();
    REQUIRE(!c->bong_c());
    d->bing_d();
    REQUIRE(!d->bong_d());

    a->bing_a(true);
    b->bing_b(true);

    c->bing_c();
    REQUIRE(c->bong_c());
    d->bing_d();
    REQUIRE(d->bong_d());
}

TEST_CASE("ModuleManager - Inject fail")
{
    ModuleManager manager;

    Boardcore::A *a = new Boardcore::A();

    REQUIRE(manager.insert<Boardcore::A>(a));
    REQUIRE_FALSE(manager.inject());
}