// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <mp-units/systems/usc.h>

#include "wpi/units.hpp"

namespace mp {

using namespace usc;

namespace unit_symbols {

// We can't use the entire usc::unit_symbols namespace because minim and minute
// would conflict. In addition, most unit symbols are highly unlikely to be
// used, and including them may lead to confusion.

using international::unit_symbols::lb;
using international::unit_symbols::oz;

using international::unit_symbols::ft;
using international::unit_symbols::in;
using international::unit_symbols::mi;
using international::unit_symbols::yd;

using international::unit_symbols::mph;

}  // namespace unit_symbols

using namespace unit_symbols;

}  // namespace mp
