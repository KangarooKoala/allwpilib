// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units2/base.h"
#include "units2/charge.h"
#include "units2/energy.h"
#include "units2/mass.h"
#include "units2/substance.h"
#include "units2/temperature.h"
#include "units2/time.h"
#include "units2/velocity.h"

namespace units::constants {

// Ratio of a circle's circumference to its diameter.
UNIT_DEFINE_NAMED_UNIT(pi, pi, mp_units::mag_pi* mp_units::one)
// Speed of light in vacuum.
UNIT_DEFINE_NAMED_UNIT(c, c, mp_units::si::si2019::speed_of_light_in_vacuum)
// Newtonian constant of gravitation.
UNIT_DEFINE_NAMED_UNIT(G, G,
                       mp_units::mag_ratio<667'408, 100'000>*
                           mp_units::mag_power<10, -11>* meters_per_second)
// Planck constant.
UNIT_DEFINE_NAMED_UNIT(h, h,
                       mp_units::mag_ratio<662'607'004, 100'000'000>*
                           mp_units::mag_power<10, -34>* joule* second)
// Vacuum permeability.
UNIT_DEFINE_NAMED_UNIT(mu0, mu0, mp_units::si::magnetic_constant)
// Vacuum permitivity.
UNIT_DEFINE_NAMED_UNIT(epsilon0, epsilon0, mp_units::inverse(mu0* c* c))
// Characteristic impedance of vacuum.
UNIT_DEFINE_NAMED_UNIT(Z0, Z0, mu0* c)
// Coulomb's constant.
UNIT_DEFINE_NAMED_UNIT(k_e, k_e,
                       mp_units::inverse(mp_units::mag<4>* pi* epsilon0))
// Elementary charge.
UNIT_DEFINE_NAMED_UNIT(e, e,
                       mp_units::mag_ratio<16'021'766'208, 10'000'000'000>*
                           mp_units::mag_power<10, -19>* coulomb)
// Electron mass.
UNIT_DEFINE_NAMED_UNIT(m_e, m_e,
                       mp_units::mag_ratio<910'938'356, 100'000'000>*
                           mp_units::mag_power<10, -31>* kilogram)
// Proton mass.
UNIT_DEFINE_NAMED_UNIT(m_p, m_p,
                       mp_units::mag_ratio<1'672'621'898, 1'000'000'000>*
                           mp_units::mag_power<10, -27>* kilogram)
// Bohr magneton.
UNIT_DEFINE_NAMED_UNIT(mu_B, mu_B, e* h / (mp_units::mag<4> * pi * m_e))
// Avagadro's Number.
UNIT_DEFINE_NAMED_UNIT(N_A, N_A,
                       mp_units::mag_ratio<6'022'140'857, 1'000'000'000>*
                           mp_units::mag_power<10, 23>* mp_units::inverse(mole))
// Gas constant.
UNIT_DEFINE_NAMED_UNIT(R, R,
                       mp_units::mag_ratio<83'144'598, 10'000'000>* joule /
                           (kelvin * mole))
// Boltzmann constant.
UNIT_DEFINE_NAMED_UNIT(k_B, k_B, R / N_A)
// Faraday constant.
UNIT_DEFINE_NAMED_UNIT(F, F, N_A* e)
// Stefan-Boltzmann constant.
UNIT_DEFINE_NAMED_UNIT(sigma, sigma,
                       mp_units::mag<2>* mp_units::pow<5>(pi) *
                           mp_units::pow<4>(R) /
                           (mp_units::mag<15> * mp_units::cubic(h) *
                            mp_units::square(c) * mp_units::pow<4>(N_A)))

}  // namespace units::constants
