namespace Boardcore
{

template <template <typename...> class, template <typename...> class>
struct is_same_template : std::false_type
{
};

template <template <typename...> class T>
struct is_same_template<T, T> : std::true_type
{
};

}  // namespace Boardcore
