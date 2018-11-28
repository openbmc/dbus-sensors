/*
// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#pragma once
#include <stdexcept>
#include <string>
struct VariantToFloatVisitor
{
    template <typename T>
    std::enable_if_t<std::is_arithmetic<T>::value, float>
        operator()(const T &t) const
    {
        return static_cast<float>(t);
    }

    template <typename T>
    std::enable_if_t<!std::is_arithmetic<T>::value, float>
        operator()(const T &t) const
    {
        throw std::invalid_argument("Cannot translate type to float");
    }
};

struct VariantToIntVisitor
{
    template <typename T>
    std::enable_if_t<std::is_arithmetic<T>::value, int>
        operator()(const T &t) const
    {
        return static_cast<float>(t);
    }

    template <typename T>
    std::enable_if_t<!std::is_arithmetic<T>::value, int>
        operator()(const T &t) const
    {
        throw std::invalid_argument("Cannot translate type to int");
    }
};

struct VariantToUnsignedIntVisitor
{
    template <typename T>
    std::enable_if_t<std::is_arithmetic<T>::value, unsigned int>
        operator()(const T &t) const
    {
        return static_cast<float>(t);
    }

    template <typename T>
    std::enable_if_t<!std::is_arithmetic<T>::value, unsigned int>
        operator()(const T &t) const
    {
        throw std::invalid_argument("Cannot translate type to unsigned int");
    }
};

struct VariantToStringVisitor
{
    template <typename T>
    std::enable_if_t<std::is_arithmetic<T>::value, std::string>
        operator()(const T &t) const
    {
        return std::to_string(t);
    }

    template <typename T>
    std::enable_if_t<!std::is_arithmetic<T>::value, std::string>
        operator()(const T &t) const
    {
        return returnString(t);
    }

    template <typename T>
    std::enable_if_t<std::is_same<T, std::string>::value, std::string>
        returnString(const T &t) const
    {
        return t;
    }

    template <typename T>
    std::enable_if_t<!std::is_same<T, std::string>::value, std::string>
        returnString(const T &t) const
    {
        throw std::invalid_argument("Cannot translate type to string");
    }
};

struct VariantToDoubleVisitor
{
    template <typename T>
    std::enable_if_t<std::is_arithmetic<T>::value, double>
        operator()(const T &t) const
    {
        return static_cast<double>(t);
    }

    template <typename T>
    std::enable_if_t<!std::is_arithmetic<T>::value, double>
        operator()(const T &t) const
    {
        throw std::invalid_argument("Cannot translate type to double");
    }
};