/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Plane.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Plane class
 */
Plane::Plane(void)
    : logger(g.log_bitmask)
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
}

Plane plane;
AP_Vehicle& vehicle = plane;

FlightStage::FlightStage(void)
{
  _flight_stage = Stage_Idle;
  stage_idle_init = false;
  stage_underwater_init = false;
  stage_takeoff_init = false;
  stage_fixwing_init = false;
  specified_depth = -50;
  swimming_duration = 5;
  achived_depth_time = 0;
  depth_tolerance = 10;
  stage_achieve_depth = false;
}

FlightStage::FlightStage_t FlightStage::get_current_flight_stage(void)
{
  return _flight_stage;
}
void FlightStage::set_current_flight_stage(FlightStage_t stg)
{
  _flight_stage = stg;
}
void FlightStage::set_specified_depth(float depth)
{
  specified_depth = depth;
}
float FlightStage::get_specified_depth(void)
{
  return specified_depth;
}
void FlightStage::set_depth_tolerance(float tolerance)
{
  depth_tolerance = tolerance;
}
float FlightStage::get_depth_tolerance(void)
{
  return depth_tolerance;
}
void FlightStage::reset(void)
{
  _flight_stage = Stage_Idle;
  stage_idle_init = false;
  stage_underwater_init = false;
  stage_takeoff_init = false;
  stage_fixwing_init = false;
  stage_achieve_depth = false;
}
FlightStage flightstage;