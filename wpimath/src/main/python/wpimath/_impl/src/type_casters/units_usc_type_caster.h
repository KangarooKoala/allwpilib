#pragma once

#include <wpi/units-usc.hpp>

namespace pybind11 {
namespace detail {
template <>
struct handle_type_name<mp::quantity<mp::lb>> {
  static constexpr auto name = _("lb");
};

template <>
struct handle_type_name<mp::quantity<mp::oz>> {
  static constexpr auto name = _("oz");
};

template <>
struct handle_type_name<mp::quantity<mp::ft>> {
  static constexpr auto name = _("ft");
};

template <>
struct handle_type_name<mp::quantity<mp::in>> {
  static constexpr auto name = _("in");
};

template <>
struct handle_type_name<mp::quantity<mp::mi>> {
  static constexpr auto name = _("mi");
};

template <>
struct handle_type_name<mp::quantity<mp::yd>> {
  static constexpr auto name = _("yd");
};

template <>
struct handle_type_name<mp::quantity<mp::mph>> {
  static constexpr auto name = _("mph");
};

}  // namespace detail
}  // namespace pybind11

#include "_units_base_type_caster.h"
