#pragma once

#include <cstddef>
#include <functional>
#include <type_traits>

/*
This code was written by a deranged furry, be careful!

     @#...........@-                                @=..............@
     @.............@@      @@...%@@@              @.................@
    @=...............@%     @........#@@        @%..................@
    @.................:@    @@..........+@@    @+...................@.
    @...................@@   @+............@@ @@....................@@
   @#.....................@:  @%:............@@.....................@@
   @......................@@@..................@%...................@@
   @...................:@%......... ................................@@
   @...................###-::....::.................................@@
   @................................................................@:
   @#...............................................................@
    @...............................................................@
    @#.............................................................@*
     @......................................-%@@@@@@@@@@@.........@@
      @......@@@@@@@@@@@@@@@@............@@@@@@@@....-@..........%@
      @@......-@.....@@@@@@@@............@@@@@@@@.....@@........@%
       +@.....@......@@@@@@@@............@@@@@@@@......@......@@
@@@@@@@  @...-@......@@@@@@@@............@@@@@@@@......@....%:.....:@
 @...........@%......@@@@@@@-............@@@@@@@@......@..........@@
  @@.........#@......-@@@@@@..............@@@@@@.......@........-@.
    @@........@*.......@@@@....-==........ ...................-@+
      +@....-=......................................**..*....*@=
      @@..*+:*:**+................................:*-**=.......%@
      @......+....................**.....@@..........:..........=@
     @.....................%@@%@@+...:::..........................@
    @...................................................@@@++@@@@@@+
    @@@@@@.. @@@....................................@@@.
                 @@@*.........................@@@@*
                   @=.+@@@@@#...................@#
                    @@...........................@+
                      @@-.........................@=
                       @+..........................@
                     @@.............................@
                    @@@@@@..........................=@
                        @%...........................@#
                        @.............................@
                       @..............................@%
                      #@...............................@
 */

namespace socrate
{
namespace reflect
{

/**
 * A marker type used for overload dispatch on a given type.
 * Contains nothing other than a type.
 */
template <typename T>
struct TypeHolder
{
    using Type = T;
};

namespace detail
{

template <typename FieldT, typename... Inner>
struct FieldDef
{
    using FieldType  = FieldT;
    using StructType = void;

    constexpr const char* field_name() const { return name; }

    const FieldType& get_const(const FieldType& parent) const { return parent; }

    FieldType& get(FieldType& parent) const { return parent; }

    const char* name;
};

template <typename StructT, typename FieldT, typename... FieldT2>
struct FieldDef<StructT, FieldT, FieldT2...>
{
    using Inner      = FieldDef<FieldT, FieldT2...>;
    using FieldType  = typename Inner::FieldType;
    using StructType = StructT;

    constexpr const char* field_name() const { return inner.field_name(); }

    const FieldType& get_const(const StructType& parent) const
    {
        return inner.get_const(parent.*(ptr));
    }

    FieldType& get(StructType& parent) const
    {
        return inner.get(parent.*(ptr));
    }

    FieldT StructT::* ptr;
    Inner inner;
};

template <typename StructT, typename FieldT, typename... FieldT2>
constexpr auto field_def_ctor_helper(FieldT StructT::* ptr,
                                     FieldDef<FieldT, FieldT2...> inner)
{
    return FieldDef<StructT, FieldT, FieldT2...>{ptr, inner};
}

template <typename StructT, typename FieldT>
constexpr auto build_field_def(const char* name, FieldT StructT::* ptr)
{
    return FieldDef<StructT, FieldT>{ptr, FieldDef<FieldT>{name}};
}

template <typename StructT, typename FieldT, typename FieldT2,
          typename... Inner>
constexpr auto build_field_def(const char* name, FieldT StructT::* ptr,
                               FieldT2 FieldT::* ptr2, Inner... inner)
{
    return field_def_ctor_helper(ptr, build_field_def(name, ptr2, inner...));
}

template <typename StructT, typename... StructDefsT>
struct StructDef;

template <typename StructT, typename FieldDefT, typename... FieldDefsT>
constexpr auto add_field(const StructDef<StructT, FieldDefsT...>& struct_def,
                         FieldDefT field_def)
{
    return StructDef<StructT, FieldDefT, FieldDefsT...>{struct_def, field_def};
}

template <typename StructDefT, typename StructDefT2>
constexpr auto add_extends(const StructDefT& struct_def,
                           const StructDefT2& other)
{
    return other.add_fields_onto(struct_def);
}

/*
Base case specialization, used to end the recursion, only contains struct name.
*/
template <typename StructT>
struct StructDef<StructT>
{
    template <typename F>
    constexpr void for_each_field(StructT& value, F&& f) const
    {
        // No fields to iterate over
    }

    template <typename F>
    constexpr void for_each_field_const(const StructT& value, F&& f) const
    {
        // No fields to iterate over
    }

    template <typename F, typename T>
    constexpr auto fold_fields(StructT& value, T init, F&& f) const
    {
        // No fields to iterate over
        return init;
    }

    template <typename F, typename T>
    constexpr auto fold_fields_const(const StructT& value, T init, F&& f) const
    {
        // No fields to iterate over
        return init;
    }

    template <typename F>
    constexpr void for_each_field_type(F&& f) const
    {
        // No fields to iterate over
    }

    template <typename F, typename T>
    constexpr auto fold_fields_type(T init, F&& f) const
    {
        // No fields to iterate over
        return init;
    }

    constexpr const char* type_name() const { return name; }

    constexpr size_t field_count() const { return 0; }

    template <typename StructT2>
    constexpr auto add_fields_onto(const StructT2& other) const
    {
        return other;
    }

    const char* name;
};

/*
Recursive case specialization
*/

template <typename StructT, typename FieldDefT, typename... FieldDefsT>
struct StructDef<StructT, FieldDefT, FieldDefsT...>
{
    template <typename StructT2, typename... FieldDefsT2>
    friend class StructDef;

    using Inner     = StructDef<StructT, FieldDefsT...>;
    using FieldType = typename FieldDefT::FieldType;

    // cppcheck-suppress uninitMemberVar
    constexpr StructDef(const Inner& inner, FieldDefT field)
        : inner{inner}, field{field}
    {
    }

    template <typename F>
    constexpr void for_each_field(StructT& value, F&& f) const
    {
        // Call into inner recursively
        inner.for_each_field(value, f);

        FieldType& value2 = field.get(value);
        f(field.field_name(), value2, TypeHolder<FieldType>{});
    }

    template <typename F>
    constexpr void for_each_field_const(const StructT& value, F&& f) const
    {
        // Call into inner recursively
        inner.for_each_field_const(value, f);

        const FieldType& value2 = field.get_const(value);
        f(field.field_name(), value2, TypeHolder<FieldType>{});
    }

    template <typename F, typename T>
    constexpr auto fold_fields(StructT& value, T init, F&& f) const
    {
        // Call into inner recursively
        auto acc = inner.fold_fields(value, init, f);

        FieldType& value2 = field.get(value);
        return f(acc, field.field_name(), value2, TypeHolder<FieldType>{});
    }

    template <typename F, typename T>
    constexpr auto fold_fields_const(const StructT& value, T init, F&& f) const
    {
        // Call into inner recursively
        auto acc = inner.fold_fields_const(value, init, f);

        const FieldType& value2 = field.get_const(value);
        return f(acc, field.field_name(), value2, TypeHolder<FieldType>{});
    }

    template <typename F>
    constexpr void for_each_field_type(F&& f) const
    {
        // Call into inner recursively
        inner.for_each_field_type(f);

        f(field.field_name(), TypeHolder<FieldType>{});
    }

    template <typename F, typename T>
    constexpr auto fold_fields_type(T init, F&& f) const
    {
        // Call into inner recursively
        auto acc = inner.fold_fields_type(init, f);

        return f(acc, field.field_name(), TypeHolder<FieldType>{});
    }

    constexpr const char* type_name() const
    {
        // Recurse until you get to the actual root
        return inner.type_name();
    }

    constexpr size_t field_count() const
    {
        // Recurse until you get to the actual root
        return inner.field_count() + 1;
    }

    template <typename StructT2>
    constexpr auto add_fields_onto(const StructT2& other) const
    {
        return detail::add_field(inner.add_fields_onto(other), field);
    }

    Inner inner;
    FieldDefT field;
};

// Oh SFINAE, my dear

template <typename T, typename = void>
struct IsReflectable : std::false_type
{
};

template <typename T>
struct IsReflectable<T, decltype(T::reflect(), std::declval<void>())>
    : std::true_type
{
};

template <typename StructT, typename... Ptrs>
struct ValidatePtrArgs : std::false_type
{
};

template <typename StructT>
struct ValidatePtrArgs<StructT> : std::true_type
{
};

template <typename StructT, typename FieldT, typename... Ptrs>
struct ValidatePtrArgs<StructT, FieldT StructT::*, Ptrs...>
    : ValidatePtrArgs<FieldT, Ptrs...>
{
};

}  // namespace detail

/**
 * Definition of a struct (or class).
 */
template <typename StructT, typename... FieldDefsT>
class StructDef
{
private:
    template <typename StructT2>
    friend constexpr auto struct_def(const char* name);

    template <typename StructT2, typename... FieldDefsT2>
    friend class StructDef;

    /*
    This is used to help with automatic template argument deduction on
    constructor invocation.

    For various reasons in C++ constructor struggle with template deduction, but
    functions do not!
    */
    template <typename StructT2, typename... FieldDefsT2>
    static constexpr auto ctor_helper(
        const detail::StructDef<StructT2, FieldDefsT2...>& inner)
    {
        return StructDef<StructT2, FieldDefsT2...>{inner};
    }

public:
    /**
     * Iterate over each field by reference.
     *
     * @param value Instance of the struct to iterate.
     * @param f Callback, must be invokable.
     */
    template <typename F>
    constexpr void for_each_field(StructT& value, F&& f) const
    {
        inner.for_each_field(value, f);
    }

    /**
     * Iterate over each field by reference.
     *
     * @param value Instance of the struct to iterate.
     * @param f Callback, must be invokable.
     */
    template <typename F>
    constexpr void for_each_field_const(const StructT& value, F&& f) const
    {
        inner.for_each_field_const(value, f);
    }

    /**
     * Fold over each field by reference.
     *
     * @param value Instance of the struct to fold.
     * @param init Initial value of the accumulator
     * @param f Callback, must be invokable.
     */
    template <typename F, typename T>
    constexpr auto fold_fields(StructT& value, T init, F&& f) const
    {
        return inner.fold_fields(value, init, f);
    }

    /**
     * Fold over each field by constant reference.
     *
     * @param value Instance of the struct to fold.
     * @param init Initial value of the accumulator
     * @param f Callback, must be invokable.
     */
    template <typename F, typename T>
    constexpr auto fold_fields_const(const StructT& value, T init, F&& f) const
    {
        return inner.fold_fields_const(value, init, f);
    }

    /**
     * Iterate over the type of each field.
     *
     * @param f Callback, must be invokable.
     */
    template <typename F>
    constexpr void for_each_field_type(F&& f) const
    {
        inner.for_each_field_type(f);
    }

    /**
     * Fold over over the type of each field.
     *
     * @param init Initial value of the accumulator
     * @param f Callback, must be invokable.
     */
    template <typename F, typename T>
    constexpr auto fold_fields_type(T init, F&& f) const
    {
        return inner.fold_fields_type(init, f);
    }

    /**
     * Get the actual name of this type.
     */
    constexpr const char* type_name() const { return inner.type_name(); }

    /**
     * Get the number of fields.
     */
    constexpr size_t field_count() const { return inner.field_count(); }

    /**
     * Add a new field to this definition.
     */
    template <typename StructT2, typename FieldT2, typename... Ptrs>
    constexpr auto add_field(const char* name, FieldT2 StructT2::* ptr,
                             Ptrs... ptrs) const
    {
        static_assert(
            std::is_base_of<StructT2, StructT>::value,
            "You cannot add fields from a class that is not a parent of "
            "the current one");

        static_assert(detail::ValidatePtrArgs<FieldT2, Ptrs...>::value,
                      "Arguments are of invalid type");

        return ctor_helper(detail::add_field(
            inner, detail::build_field_def(name, ptr, ptrs...)));
    }

    /**
     * Make this class inherit from another one.
     */
    template <typename StructT2>
    constexpr auto add_extends() const
    {
        static_assert(
            std::is_base_of<StructT2, StructT>::value,
            "You cannot add fields from a class that is not a parent of "
            "the current one");

        return ctor_helper(
            detail::add_extends(inner, StructT2::reflect().inner));
    }

private:
    constexpr explicit StructDef(
        const detail::StructDef<StructT, FieldDefsT...>& inner)
        : inner{inner}
    {
    }

    detail::StructDef<StructT, FieldDefsT...> inner;
};

/**
 * Helper function to define a struct (or class).
 * Use of \def STRUCT_DEF is recommended for more readable code.
 */
template <typename StructT>
constexpr auto struct_def(const char* name)
{
    return StructDef<StructT>{detail::StructDef<StructT>{name}};
}

/**
 * Helper macro to define a field.
 * \warn{only usable inside \def STRUCT_DEF!}
 */
#define FIELD_DEF(name) .add_field(#name, &__StructTAlias::name)

/**
 * Helper macro to define a field to a nested struct.
 * \warn{only usable inside \def STRUCT_DEF!}
 */
#define FIELD_DEF2(name, name2)               \
    .add_field(#name2, &__StructTAlias::name, \
               &decltype(std::declval<__StructTAlias>().name)::name2)

/**
 * Helper macro to define a field to a nested struct.
 * \warn{only usable inside \def STRUCT_DEF!}
 */
#define FIELD_DEF3(name, name2, name3)                                \
    .add_field(#name3, &__StructTAlias::name,                         \
               &decltype(std::declval<__StructTAlias>().name)::name2, \
               &decltype(std::declval<__StructTAlias>().name.name2)::name3)

/**
 * Helper macro to inherit from another struct.
 * \warn{only usable inside \def STRUCT_DEF!}
 */
#define EXTEND_DEF(name) .add_extends<name>()

/**
 * Helper macro to define a struct (or class).
 */
#define STRUCT_DEF(name, defs)                                    \
    ({                                                            \
        using __StructTAlias = name;                              \
        socrate::reflect::struct_def<__StructTAlias>(#name) defs; \
    })

/**
 * Detect if a given type is reflectable.
 */
template <typename T>
struct IsReflectable : detail::IsReflectable<T>
{
};

/**
 * Detect if a given type is reflectable.
 */
template <typename T>
constexpr bool is_reflectable = IsReflectable<T>::value;

}  // namespace reflect
}  // namespace socrate
