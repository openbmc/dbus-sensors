// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright 2018 Intel Corporation

#pragma once
#include <boost/type_index.hpp>

#include <concepts>
#include <stdexcept>
#include <string>
#include <vector>

namespace details
{

template <typename U>
struct VariantToNumericVisitor
{
    template <typename T>
    U operator()(const T& t) const
    {
        if constexpr (std::is_arithmetic_v<T>)
        {
            return static_cast<U>(t);
        }
        throw std::invalid_argument(
            "Cannot translate type " +
            boost::typeindex::type_id<T>().pretty_name() + " to " +
            boost::typeindex::type_id<U>().pretty_name());
    }
};

} // namespace details

using VariantToFloatVisitor = details::VariantToNumericVisitor<float>;
using VariantToIntVisitor = details::VariantToNumericVisitor<int>;
using VariantToUnsignedIntVisitor =
    details::VariantToNumericVisitor<unsigned int>;
using VariantToDoubleVisitor = details::VariantToNumericVisitor<double>;

struct VariantToStringVisitor
{
    template <typename T>
    std::string operator()(const T& t) const
    {
        if constexpr (std::is_same_v<T, std::string>)
        {
            return t;
        }
        else if constexpr (std::is_arithmetic_v<T>)
        {
            return std::to_string(t);
        }
        throw std::invalid_argument(
            "Cannot translate type " +
            boost::typeindex::type_id<T>().pretty_name() + " to string");
    }
};

template <std::integral V, std::integral U>
struct VariantToNumArrayVisitor
{
    template <typename T>
    std::vector<V> operator()(const T& t) const
    {
        if constexpr (std::is_same_v<T, std::vector<U>>)
        {
            std::vector<V> output;
            output.reserve(t.size());

            for (const auto& value : t)
            {
                output.push_back(static_cast<V>(value));
            }

            return output;
        }
        throw std::invalid_argument(
            "Cannot handle type " +
            boost::typeindex::type_id<T>().pretty_name() + " to vector<U>");
    }
};
