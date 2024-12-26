/*
 * utils.h
 *
 *  Created on: Dec 25, 2024
 *      Author: Andrey Belyakov
 */

#pragma once
#include <type_traits>

template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
    return static_cast<typename std::underlying_type<E>::type>(e);
}
