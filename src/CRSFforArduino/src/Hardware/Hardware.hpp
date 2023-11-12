/**
 * @file Hardware.hpp
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief This file is the top level of the hardware abstraction layer.
 * @version 0.5.0
 * @date 2023-11-1
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This header file is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#pragma once

#if defined(ARDUINO) && defined(PLATFORMIO)
#include "CFA_Config.hpp"
#elif defined(ARDUINO) && !defined(PLATFORMIO)
#include "../CFA_Config.hpp"
#endif
#include "CompatibilityTable/CompatibilityTable.hpp"
#include "DevBoards/DevBoards.hpp"

using namespace crsfForArduinoConfig;
using namespace hal;
