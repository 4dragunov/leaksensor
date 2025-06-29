/*
 * tid.h
 *
 *  Created on: Dec 28, 2024
 *      Author: Andrey
 */

#ifndef SRC_BOARDS_SRWLD01_TID_H_
#define SRC_BOARDS_SRWLD01_TID_H_

#include <functional>
#include <type_traits>

template<typename T>
int* type_id_with_cvr()
{
    static int id;
    return &id;
};

template<typename T>
int* type_id()
{
    using t = std::remove_cv<std::remove_reference<T>>;
    return type_id_with_cvr<t>();
};

class type_index
{
//private:
public:
    int* id;
    type_index(int* id) : id(id) {}


    bool operator==(type_index const& t) const
    { return id == t.id; }
    bool operator!=(type_index const& t) const
    { return id != t.id; }
    bool operator<(type_index const& t) const
    { return std::less<int*>()(id, t.id); }
    bool operator<=(type_index const& t) const
    { return !(t > *this); }
    bool operator>(type_index const& t) const
    { return t < *this; }
    bool operator>=(type_index const& t) const
    { return !(*this < t); }

    std::size_t hash_code() const { return std::hash<int*>()(id); }

    template<typename T>
    friend type_index type_id_with_cvr()
    {
        static int id;
        return &id;
    }
    template<typename T>
    friend type_index type_id()
    {
        using t = std::remove_cv<std::remove_reference<T>>;
        return type_id_with_cvr<t>();
    }
};
/*
namespace std
{
    template<>
    struct hash<type_index>
    {
        std::size_t operator()(type_index const& t) const
        {
            return t.hash_code();
        }
    };
}
*/
#endif /* SRC_BOARDS_SRWLD01_TID_H_ */
