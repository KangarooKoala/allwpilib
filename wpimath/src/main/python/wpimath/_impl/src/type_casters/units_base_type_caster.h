#pragma once

#include <wpi/units.hpp>

namespace pybind11 {
namespace detail {
template <>
struct handle_type_name<mp::quantity<mp::deg>> {
  static constexpr auto name = _("deg");
};

template <>
struct handle_type_name<mp::quantity<mp::deg2>> {
  static constexpr auto name = _("deg2");
};

template <>
struct handle_type_name<mp::quantity<mp::grad>> {
  static constexpr auto name = _("grad");
};

template <>
struct handle_type_name<mp::quantity<mp::rad>> {
  static constexpr auto name = _("rad");
};

template <>
struct handle_type_name<mp::quantity<mp::rad2>> {
  static constexpr auto name = _("rad2");
};

template <>
struct handle_type_name<mp::quantity<mp::rev>> {
  static constexpr auto name = _("rev");
};

template <>
struct handle_type_name<mp::quantity<mp::sr>> {
  static constexpr auto name = _("sr");
};

template <>
struct handle_type_name<mp::quantity<mp::nm>> {
  static constexpr auto name = _("nm");
};

template <>
struct handle_type_name<mp::quantity<mp::um>> {
  static constexpr auto name = _("um");
};

template <>
struct handle_type_name<mp::quantity<mp::mm>> {
  static constexpr auto name = _("mm");
};

template <>
struct handle_type_name<mp::quantity<mp::m>> {
  static constexpr auto name = _("m");
};

template <>
struct handle_type_name<mp::quantity<mp::km>> {
  static constexpr auto name = _("km");
};

template <>
struct handle_type_name<mp::quantity<mp::ns>> {
  static constexpr auto name = _("ns");
};

template <>
struct handle_type_name<mp::quantity<mp::us>> {
  static constexpr auto name = _("us");
};

template <>
struct handle_type_name<mp::quantity<mp::ms>> {
  static constexpr auto name = _("ms");
};

template <>
struct handle_type_name<mp::quantity<mp::s>> {
  static constexpr auto name = _("s");
};

template <>
struct handle_type_name<mp::quantity<mp::ks>> {
  static constexpr auto name = _("ks");
};

template <>
struct handle_type_name<mp::quantity<mp::ng>> {
  static constexpr auto name = _("ng");
};

template <>
struct handle_type_name<mp::quantity<mp::ug>> {
  static constexpr auto name = _("ug");
};

template <>
struct handle_type_name<mp::quantity<mp::mg>> {
  static constexpr auto name = _("mg");
};

template <>
struct handle_type_name<mp::quantity<mp::g>> {
  static constexpr auto name = _("g");
};

template <>
struct handle_type_name<mp::quantity<mp::kg>> {
  static constexpr auto name = _("kg");
};

template <>
struct handle_type_name<mp::quantity<mp::nA>> {
  static constexpr auto name = _("nA");
};

template <>
struct handle_type_name<mp::quantity<mp::uA>> {
  static constexpr auto name = _("uA");
};

template <>
struct handle_type_name<mp::quantity<mp::mA>> {
  static constexpr auto name = _("mA");
};

template <>
struct handle_type_name<mp::quantity<mp::A>> {
  static constexpr auto name = _("A");
};

template <>
struct handle_type_name<mp::quantity<mp::kA>> {
  static constexpr auto name = _("kA");
};

template <>
struct handle_type_name<mp::quantity_point<mp::nK>> {
  static constexpr auto name = _("nK");
};

template <>
struct handle_type_name<mp::quantity_point<mp::uK>> {
  static constexpr auto name = _("uK");
};

template <>
struct handle_type_name<mp::quantity_point<mp::mK>> {
  static constexpr auto name = _("mK");
};

template <>
struct handle_type_name<mp::quantity_point<mp::K>> {
  static constexpr auto name = _("K");
};

template <>
struct handle_type_name<mp::quantity_point<mp::kK>> {
  static constexpr auto name = _("kK");
};

template <>
struct handle_type_name<mp::quantity<mp::nmol>> {
  static constexpr auto name = _("nmol");
};

template <>
struct handle_type_name<mp::quantity<mp::umol>> {
  static constexpr auto name = _("umol");
};

template <>
struct handle_type_name<mp::quantity<mp::mmol>> {
  static constexpr auto name = _("mmol");
};

template <>
struct handle_type_name<mp::quantity<mp::mol>> {
  static constexpr auto name = _("mol");
};

template <>
struct handle_type_name<mp::quantity<mp::kmol>> {
  static constexpr auto name = _("kmol");
};

template <>
struct handle_type_name<mp::quantity<mp::ncd>> {
  static constexpr auto name = _("ncd");
};

template <>
struct handle_type_name<mp::quantity<mp::ucd>> {
  static constexpr auto name = _("ucd");
};

template <>
struct handle_type_name<mp::quantity<mp::mcd>> {
  static constexpr auto name = _("mcd");
};

template <>
struct handle_type_name<mp::quantity<mp::cd>> {
  static constexpr auto name = _("cd");
};

template <>
struct handle_type_name<mp::quantity<mp::kcd>> {
  static constexpr auto name = _("kcd");
};

template <>
struct handle_type_name<mp::quantity<mp::nHz>> {
  static constexpr auto name = _("nHz");
};

template <>
struct handle_type_name<mp::quantity<mp::uHz>> {
  static constexpr auto name = _("uHz");
};

template <>
struct handle_type_name<mp::quantity<mp::mHz>> {
  static constexpr auto name = _("mHz");
};

template <>
struct handle_type_name<mp::quantity<mp::Hz>> {
  static constexpr auto name = _("Hz");
};

template <>
struct handle_type_name<mp::quantity<mp::kHz>> {
  static constexpr auto name = _("kHz");
};

template <>
struct handle_type_name<mp::quantity<mp::nN>> {
  static constexpr auto name = _("nN");
};

template <>
struct handle_type_name<mp::quantity<mp::uN>> {
  static constexpr auto name = _("uN");
};

template <>
struct handle_type_name<mp::quantity<mp::mN>> {
  static constexpr auto name = _("mN");
};

template <>
struct handle_type_name<mp::quantity<mp::N>> {
  static constexpr auto name = _("N");
};

template <>
struct handle_type_name<mp::quantity<mp::kN>> {
  static constexpr auto name = _("kN");
};

template <>
struct handle_type_name<mp::quantity<mp::nPa>> {
  static constexpr auto name = _("nPa");
};

template <>
struct handle_type_name<mp::quantity<mp::uPa>> {
  static constexpr auto name = _("uPa");
};

template <>
struct handle_type_name<mp::quantity<mp::mPa>> {
  static constexpr auto name = _("mPa");
};

template <>
struct handle_type_name<mp::quantity<mp::Pa>> {
  static constexpr auto name = _("Pa");
};

template <>
struct handle_type_name<mp::quantity<mp::kPa>> {
  static constexpr auto name = _("kPa");
};

template <>
struct handle_type_name<mp::quantity<mp::nJ>> {
  static constexpr auto name = _("nJ");
};

template <>
struct handle_type_name<mp::quantity<mp::uJ>> {
  static constexpr auto name = _("uJ");
};

template <>
struct handle_type_name<mp::quantity<mp::mJ>> {
  static constexpr auto name = _("mJ");
};

template <>
struct handle_type_name<mp::quantity<mp::J>> {
  static constexpr auto name = _("J");
};

template <>
struct handle_type_name<mp::quantity<mp::kJ>> {
  static constexpr auto name = _("kJ");
};

template <>
struct handle_type_name<mp::quantity<mp::nW>> {
  static constexpr auto name = _("nW");
};

template <>
struct handle_type_name<mp::quantity<mp::uW>> {
  static constexpr auto name = _("uW");
};

template <>
struct handle_type_name<mp::quantity<mp::mW>> {
  static constexpr auto name = _("mW");
};

template <>
struct handle_type_name<mp::quantity<mp::W>> {
  static constexpr auto name = _("W");
};

template <>
struct handle_type_name<mp::quantity<mp::kW>> {
  static constexpr auto name = _("kW");
};

template <>
struct handle_type_name<mp::quantity<mp::nC>> {
  static constexpr auto name = _("nC");
};

template <>
struct handle_type_name<mp::quantity<mp::uC>> {
  static constexpr auto name = _("uC");
};

template <>
struct handle_type_name<mp::quantity<mp::mC>> {
  static constexpr auto name = _("mC");
};

template <>
struct handle_type_name<mp::quantity<mp::C>> {
  static constexpr auto name = _("C");
};

template <>
struct handle_type_name<mp::quantity<mp::kC>> {
  static constexpr auto name = _("kC");
};

template <>
struct handle_type_name<mp::quantity<mp::nV>> {
  static constexpr auto name = _("nV");
};

template <>
struct handle_type_name<mp::quantity<mp::uV>> {
  static constexpr auto name = _("uV");
};

template <>
struct handle_type_name<mp::quantity<mp::mV>> {
  static constexpr auto name = _("mV");
};

template <>
struct handle_type_name<mp::quantity<mp::V>> {
  static constexpr auto name = _("V");
};

template <>
struct handle_type_name<mp::quantity<mp::kV>> {
  static constexpr auto name = _("kV");
};

template <>
struct handle_type_name<mp::quantity<mp::nF>> {
  static constexpr auto name = _("nF");
};

template <>
struct handle_type_name<mp::quantity<mp::uF>> {
  static constexpr auto name = _("uF");
};

template <>
struct handle_type_name<mp::quantity<mp::mF>> {
  static constexpr auto name = _("mF");
};

template <>
struct handle_type_name<mp::quantity<mp::F>> {
  static constexpr auto name = _("F");
};

template <>
struct handle_type_name<mp::quantity<mp::kF>> {
  static constexpr auto name = _("kF");
};

template <>
struct handle_type_name<mp::quantity<mp::nohm>> {
  static constexpr auto name = _("nohm");
};

template <>
struct handle_type_name<mp::quantity<mp::mohm>> {
  static constexpr auto name = _("mohm");
};

template <>
struct handle_type_name<mp::quantity<mp::ohm>> {
  static constexpr auto name = _("ohm");
};

template <>
struct handle_type_name<mp::quantity<mp::kohm>> {
  static constexpr auto name = _("kohm");
};

template <>
struct handle_type_name<mp::quantity<mp::uohm>> {
  static constexpr auto name = _("uohm");
};

template <>
struct handle_type_name<mp::quantity<mp::nΩ>> {
  static constexpr auto name = _("nΩ");
};

template <>
struct handle_type_name<mp::quantity<mp::mΩ>> {
  static constexpr auto name = _("mΩ");
};

template <>
struct handle_type_name<mp::quantity<mp::Ω>> {
  static constexpr auto name = _("Ω");
};

template <>
struct handle_type_name<mp::quantity<mp::kΩ>> {
  static constexpr auto name = _("kΩ");
};

template <>
struct handle_type_name<mp::quantity<mp::µΩ>> {
  static constexpr auto name = _("µΩ");
};

template <>
struct handle_type_name<mp::quantity<mp::nS>> {
  static constexpr auto name = _("nS");
};

template <>
struct handle_type_name<mp::quantity<mp::uS>> {
  static constexpr auto name = _("uS");
};

template <>
struct handle_type_name<mp::quantity<mp::mS>> {
  static constexpr auto name = _("mS");
};

template <>
struct handle_type_name<mp::quantity<mp::S>> {
  static constexpr auto name = _("S");
};

template <>
struct handle_type_name<mp::quantity<mp::kS>> {
  static constexpr auto name = _("kS");
};

template <>
struct handle_type_name<mp::quantity<mp::nWb>> {
  static constexpr auto name = _("nWb");
};

template <>
struct handle_type_name<mp::quantity<mp::uWb>> {
  static constexpr auto name = _("uWb");
};

template <>
struct handle_type_name<mp::quantity<mp::mWb>> {
  static constexpr auto name = _("mWb");
};

template <>
struct handle_type_name<mp::quantity<mp::Wb>> {
  static constexpr auto name = _("Wb");
};

template <>
struct handle_type_name<mp::quantity<mp::kWb>> {
  static constexpr auto name = _("kWb");
};

template <>
struct handle_type_name<mp::quantity<mp::nT>> {
  static constexpr auto name = _("nT");
};

template <>
struct handle_type_name<mp::quantity<mp::uT>> {
  static constexpr auto name = _("uT");
};

template <>
struct handle_type_name<mp::quantity<mp::mT>> {
  static constexpr auto name = _("mT");
};

template <>
struct handle_type_name<mp::quantity<mp::T>> {
  static constexpr auto name = _("T");
};

template <>
struct handle_type_name<mp::quantity<mp::kT>> {
  static constexpr auto name = _("kT");
};

template <>
struct handle_type_name<mp::quantity<mp::nH>> {
  static constexpr auto name = _("nH");
};

template <>
struct handle_type_name<mp::quantity<mp::uH>> {
  static constexpr auto name = _("uH");
};

template <>
struct handle_type_name<mp::quantity<mp::mH>> {
  static constexpr auto name = _("mH");
};

template <>
struct handle_type_name<mp::quantity<mp::H>> {
  static constexpr auto name = _("H");
};

template <>
struct handle_type_name<mp::quantity<mp::kH>> {
  static constexpr auto name = _("kH");
};

template <>
struct handle_type_name<mp::quantity<mp::nlm>> {
  static constexpr auto name = _("nlm");
};

template <>
struct handle_type_name<mp::quantity<mp::ulm>> {
  static constexpr auto name = _("ulm");
};

template <>
struct handle_type_name<mp::quantity<mp::mlm>> {
  static constexpr auto name = _("mlm");
};

template <>
struct handle_type_name<mp::quantity<mp::lm>> {
  static constexpr auto name = _("lm");
};

template <>
struct handle_type_name<mp::quantity<mp::klm>> {
  static constexpr auto name = _("klm");
};

template <>
struct handle_type_name<mp::quantity<mp::nlx>> {
  static constexpr auto name = _("nlx");
};

template <>
struct handle_type_name<mp::quantity<mp::ulx>> {
  static constexpr auto name = _("ulx");
};

template <>
struct handle_type_name<mp::quantity<mp::mlx>> {
  static constexpr auto name = _("mlx");
};

template <>
struct handle_type_name<mp::quantity<mp::lx>> {
  static constexpr auto name = _("lx");
};

template <>
struct handle_type_name<mp::quantity<mp::klx>> {
  static constexpr auto name = _("klx");
};

template <>
struct handle_type_name<mp::quantity<mp::nBq>> {
  static constexpr auto name = _("nBq");
};

template <>
struct handle_type_name<mp::quantity<mp::uBq>> {
  static constexpr auto name = _("uBq");
};

template <>
struct handle_type_name<mp::quantity<mp::mBq>> {
  static constexpr auto name = _("mBq");
};

template <>
struct handle_type_name<mp::quantity<mp::Bq>> {
  static constexpr auto name = _("Bq");
};

template <>
struct handle_type_name<mp::quantity<mp::kBq>> {
  static constexpr auto name = _("kBq");
};

template <>
struct handle_type_name<mp::quantity<mp::nGy>> {
  static constexpr auto name = _("nGy");
};

template <>
struct handle_type_name<mp::quantity<mp::uGy>> {
  static constexpr auto name = _("uGy");
};

template <>
struct handle_type_name<mp::quantity<mp::mGy>> {
  static constexpr auto name = _("mGy");
};

template <>
struct handle_type_name<mp::quantity<mp::Gy>> {
  static constexpr auto name = _("Gy");
};

template <>
struct handle_type_name<mp::quantity<mp::kGy>> {
  static constexpr auto name = _("kGy");
};

template <>
struct handle_type_name<mp::quantity<mp::nSv>> {
  static constexpr auto name = _("nSv");
};

template <>
struct handle_type_name<mp::quantity<mp::uSv>> {
  static constexpr auto name = _("uSv");
};

template <>
struct handle_type_name<mp::quantity<mp::mSv>> {
  static constexpr auto name = _("mSv");
};

template <>
struct handle_type_name<mp::quantity<mp::Sv>> {
  static constexpr auto name = _("Sv");
};

template <>
struct handle_type_name<mp::quantity<mp::kSv>> {
  static constexpr auto name = _("kSv");
};

template <>
struct handle_type_name<mp::quantity<mp::nkat>> {
  static constexpr auto name = _("nkat");
};

template <>
struct handle_type_name<mp::quantity<mp::ukat>> {
  static constexpr auto name = _("ukat");
};

template <>
struct handle_type_name<mp::quantity<mp::mkat>> {
  static constexpr auto name = _("mkat");
};

template <>
struct handle_type_name<mp::quantity<mp::kat>> {
  static constexpr auto name = _("kat");
};

template <>
struct handle_type_name<mp::quantity<mp::kkat>> {
  static constexpr auto name = _("kkat");
};

template <>
struct handle_type_name<mp::quantity_point<mp::deg_C>> {
  static constexpr auto name = _("deg_C");
};

template <>
struct handle_type_name<mp::quantity<mp::m2>> {
  static constexpr auto name = _("m2");
};

template <>
struct handle_type_name<mp::quantity<mp::m3>> {
  static constexpr auto name = _("m3");
};

template <>
struct handle_type_name<mp::quantity<mp::m4>> {
  static constexpr auto name = _("m4");
};

template <>
struct handle_type_name<mp::quantity<mp::s2>> {
  static constexpr auto name = _("s2");
};

template <>
struct handle_type_name<mp::quantity<mp::s3>> {
  static constexpr auto name = _("s3");
};

template <>
struct handle_type_name<mp::quantity<mp::a>> {
  static constexpr auto name = _("a");
};

template <>
struct handle_type_name<mp::quantity<mp::au>> {
  static constexpr auto name = _("au");
};

template <>
struct handle_type_name<mp::quantity<mp::d>> {
  static constexpr auto name = _("d");
};

template <>
struct handle_type_name<mp::quantity<mp::Da>> {
  static constexpr auto name = _("Da");
};

template <>
struct handle_type_name<mp::quantity<mp::eV>> {
  static constexpr auto name = _("eV");
};

template <>
struct handle_type_name<mp::quantity<mp::h>> {
  static constexpr auto name = _("h");
};

template <>
struct handle_type_name<mp::quantity<mp::ha>> {
  static constexpr auto name = _("ha");
};

template <>
struct handle_type_name<mp::quantity<mp::l>> {
  static constexpr auto name = _("l");
};

template <>
struct handle_type_name<mp::quantity<mp::L>> {
  static constexpr auto name = _("L");
};

template <>
struct handle_type_name<mp::quantity<mp::min>> {
  static constexpr auto name = _("min");
};

template <>
struct handle_type_name<mp::quantity<mp::t>> {
  static constexpr auto name = _("t");
};

}  // namespace detail
}  // namespace pybind11

#include "_units_base_type_caster.h"
