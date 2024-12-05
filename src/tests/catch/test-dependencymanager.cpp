/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <utils/DependencyManager/DependencyManager.h>

#include <catch2/catch.hpp>
#include <iostream>

using namespace Boardcore;

namespace Boardcore
{
class B;

class A : public Injectable
{
public:
    A() {}

    void bing_a(bool value) { this->value = value; }

    bool bong_a() { return value; }

    void inject(DependencyInjector& getter) { b = getter.get<B>(); }

private:
    B* b       = nullptr;
    bool value = false;
};

class B : public InjectableWithDeps<A>
{
public:
    B() {}

    void bing_b(bool value) { this->value = value; }

    bool bong_b() { return value; }

private:
    bool value = false;
};

class CIface : public Injectable
{
public:
    virtual void bing_c() = 0;
    virtual bool bong_c() = 0;
};

class C : public InjectableWithDeps<InjectableBase<CIface>, A, B>
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

class D : public InjectableWithDeps<CIface>
{
public:
    void bing_d() { value = getModule<CIface>()->bong_c(); }

    bool bong_d() { return value; }

private:
    bool value = false;
};

class E : public Injectable
{
public:
    int get_answer() { return 42; }
};

class F : public Injectable
{
public:
    int get_true_answer() { return 69; }
};

class G : public InjectableWithDeps<E>
{
public:
    virtual int get_truest_answer() { return getModule<E>()->get_answer(); }
};

class H : public InjectableWithDeps<InjectableBase<G>, F>
{
public:
    int get_truest_answer() override
    {
        return getModule<F>()->get_true_answer() + G::get_truest_answer();
    }
};

class I : public InjectableWithDeps<G>
{
public:
    int get_ultimate_true_answer()
    {
        return getModule<G>()->get_truest_answer();
    }
};

}  // namespace Boardcore

TEST_CASE("DependencyManager - Circular dependencies")
{
    DependencyManager manager;

    Boardcore::A* a = new Boardcore::A();
    Boardcore::B* b = new Boardcore::B();

    REQUIRE(manager.insert(a));
    REQUIRE(manager.insert(b));
    REQUIRE(manager.inject());

    a->bing_a(true);
    REQUIRE(a->bong_a());

    a->bing_a(false);
    REQUIRE(!a->bong_a());

    b->bing_b(true);
    REQUIRE(b->bong_b());

    b->bing_b(false);
    REQUIRE(!b->bong_b());

    manager.graphviz(std::cout);
}

TEST_CASE("DependencyManager - Virtual Dependencies")
{
    DependencyManager manager;

    Boardcore::A* a = new Boardcore::A();
    Boardcore::B* b = new Boardcore::B();
    Boardcore::C* c = new Boardcore::C();
    Boardcore::D* d = new Boardcore::D();

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

    manager.graphviz(std::cout);
}

TEST_CASE("DependencyManager - Inject fail")
{
    DependencyManager manager;

    Boardcore::A* a = new Boardcore::A();

    REQUIRE(manager.insert(a));
    REQUIRE_FALSE(manager.inject());
}

TEST_CASE("DependencyManager - Insert two instances fail")
{
    DependencyManager manager;

    Boardcore::A* a1 = new Boardcore::A();
    Boardcore::A* a2 = new Boardcore::A();

    REQUIRE(manager.insert(a1));
    REQUIRE_FALSE(manager.insert(a2));
}

TEST_CASE("DependencyManager - Dependency tree")
{
    DependencyManager manager;

    Boardcore::E* e = new Boardcore::E();
    Boardcore::F* f = new Boardcore::F();
    Boardcore::H* h = new Boardcore::H();
    Boardcore::I* i = new Boardcore::I();

    REQUIRE(manager.insert(e));
    REQUIRE(manager.insert(f));
    REQUIRE(manager.insert<Boardcore::G>(h));
    REQUIRE(manager.insert(i));
    REQUIRE(manager.inject());

    REQUIRE(i->get_ultimate_true_answer() == 111);

    manager.graphviz(std::cout);
}
