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
#include <boost/variant.hpp>
#include <string>

struct VariantToFloatVisitor : public boost::static_visitor<float>
{
    template <typename T> float operator()(const T &t) const
    {
        return static_cast<float>(t);
    }
};
template <>
inline float VariantToFloatVisitor::
    operator()<std::string>(const std::string &s) const
{
    throw std::invalid_argument("Cannot translate string to float");
}

struct VariantToIntVisitor : public boost::static_visitor<int>
{
    template <typename T> int operator()(const T &t) const
    {
        return static_cast<int>(t);
    }
};
template <>
inline int VariantToIntVisitor::
    operator()<std::string>(const std::string &s) const
{
    throw std::invalid_argument("Cannot translate string to int");
}

struct VariantToUnsignedIntVisitor : public boost::static_visitor<unsigned int>
{
    template <typename T> unsigned int operator()(const T &t) const
    {
        return static_cast<int>(t);
    }
};
template <>
inline unsigned int VariantToUnsignedIntVisitor::
    operator()<std::string>(const std::string &s) const
{
    throw std::invalid_argument("Cannot translate string to unsigned int");
}

struct VariantToStringVisitor : public boost::static_visitor<std::string>
{
    template <typename T> std::string operator()(const T &t) const
    {
        return std::to_string(t);
    }
};
template <>
inline std::string VariantToStringVisitor::
    operator()<std::string>(const std::string &s) const
{
    return s;
}