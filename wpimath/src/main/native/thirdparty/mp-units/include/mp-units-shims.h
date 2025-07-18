// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <compare>
#include <string_view>
#include <__ranges/concepts.h>

namespace std {

template<class _ToType, class _FromType, class = enable_if_t<
  sizeof(_ToType) == sizeof(_FromType) &&
  is_trivially_copyable_v<_ToType> &&
  is_trivially_copyable_v<_FromType>
>>
_LIBCPP_NODISCARD_EXT _LIBCPP_HIDE_FROM_ABI
constexpr _ToType bit_cast(_FromType const& __from) noexcept {
    return __builtin_bit_cast(_ToType, __from);
}

template<class _Tp, class _Cat>
concept __compares_as =
  same_as<common_comparison_category_t<_Tp, _Cat>, _Cat>;

template<class _Tp, class _Cat = partial_ordering>
concept three_way_comparable =
  __weakly_equality_comparable_with<_Tp, _Tp> &&
  __partially_ordered_with<_Tp, _Tp> &&
  requires(__make_const_lvalue_ref<_Tp> __a, __make_const_lvalue_ref<_Tp> __b) {
    { __a <=> __b } -> __compares_as<_Cat>;
  };

template<class _Tp, class _Up, class _Cat = partial_ordering>
concept three_way_comparable_with =
  three_way_comparable<_Tp, _Cat> &&
  three_way_comparable<_Up, _Cat> &&
  common_reference_with<__make_const_lvalue_ref<_Tp>, __make_const_lvalue_ref<_Up>> &&
  three_way_comparable<common_reference_t<__make_const_lvalue_ref<_Tp>, __make_const_lvalue_ref<_Up>>, _Cat> &&
  __weakly_equality_comparable_with<_Tp, _Up> &&
  __partially_ordered_with<_Tp, _Up> &&
  requires(__make_const_lvalue_ref<_Tp> __t, __make_const_lvalue_ref<_Up> __u) {
    { __t <=> __u } -> __compares_as<_Cat>;
    { __u <=> __t } -> __compares_as<_Cat>;
  };

}  // namespace std

namespace shim {

template <typename Char>
struct constexpr_string {
  Char data[128] = {};
  size_t size = 0;

  template <size_t N>
    requires (N <= 128)
  constexpr constexpr_string(Char const (&arr)[N]) {
    for (size_t i = 0; i < N; ++i) {
      data[i] = arr[i];
    }
    size = N;
  }

  constexpr constexpr_string append(auto first, auto last) {
    if (size + (last - first) > 128) {
      std::abort();
    }
    for (auto it = first; it < last; ++it) {
      data[size + (it - first)] = *it;
    }
    size += last - first;
    return *this;
  }

  constexpr constexpr_string append(Char c) {
    data[size] = c;
    ++size;
    return *this;
  }

  constexpr std::basic_string_view<Char, std::char_traits<Char>> sv() const noexcept {
    return {data, size};
  }
};

}  // namespace shim

template <typename Char, typename Traits>
constexpr int operator<=>(std::basic_string_view<Char, Traits> lhs, std::basic_string_view<Char, Traits> rhs) {
  return lhs.compare(rhs);
}

