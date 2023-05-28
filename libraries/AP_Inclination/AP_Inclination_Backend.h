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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Inclination.h"

class AP_Inclination_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_Inclination_Backend(Inclination::Inclination_State &_state, AP_Inclination_Params &_params);

    // we declare a virtual destructor so that Inclination drivers can
    // override with a custom destructor if need be
    virtual ~AP_Inclination_Backend(void) {}

    // update the state structure
    virtual void update(enum InstallLocation location) = 0;
    virtual void init_serial(uint8_t serial_instance) {};

    enum InstallLocation location() const
    {
        return (InstallLocation)params.location.get();
    }
    float get_roll_deg_from_location(enum InstallLocation loc) const
    {
        switch (loc) {
        case InstallLocation::Boom:
            return state.roll_deg.x;
        case InstallLocation::Forearm:
            return state.roll_deg.y;
        case InstallLocation::Bucket:
            return state.roll_deg.z;

        default:
            return state.roll_deg.x;
        }
    }
    Vector3f get_deg_from_location(enum InstallLocation loc)const{
        Vector3f deg;
        switch(loc){
        case InstallLocation::Boom:
            deg.x = state.roll_deg.x;
            deg.y = state.pitch_deg.x;
            deg.z = state.yaw_deg.x;
            return deg;
        case InstallLocation::Forearm:
            deg.x = state.roll_deg.y;
            deg.y = state.pitch_deg.y;
            deg.z = state.yaw_deg.y;
            return deg;
        case InstallLocation::Bucket:
            deg.x = state.roll_deg.z;
            deg.y = state.pitch_deg.z;
            deg.z = state.yaw_deg.z;
            return deg;

        default:
            deg.x = state.roll_deg.x;
            deg.y = state.pitch_deg.x;
            deg.z = state.yaw_deg.x;
            return deg;
        }
    }

    float get_max_roll_deg() const
    {
        return params.max_roll_deg;
    }
    float get_min_roll_deg() const
    {
        return params.min_roll_deg;
    }
    // float get_yaw_from_location(enum InstallLocation location) const { return nullptr }
    //float get_yaw_deg_from_location(enum InstallLocation loc) const { return get_yaw_from_location(loc); }
    float get_yaw_deg_from_location(enum InstallLocation loc) const
    {
        switch (loc) {
        case InstallLocation::Boom:
            return state.yaw_deg.x;
        case InstallLocation::Forearm:
            return state.yaw_deg.y;
        case InstallLocation::Bucket:
            return state.yaw_deg.z;

        default:
            return state.yaw_deg.x;
        }
    }

    Inclination::Status status() const;
    Inclination::Type type() const
    {
        return (Inclination::Type)params.type.get();
    }

    // true if sensor is returning data
    bool has_data() const;

    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const
    {
        return state.last_reading_ms;
    }

    // get temperature reading in Celsius  returns true on success and populates temp argument
    virtual bool get_temp_C_from_loc(enum InstallLocation location, float &temp) const
    {
        return false;
    }

protected:

    // update status based on angle measurement
    void update_status(enum InstallLocation location);

    // set status and update valid_count
    void set_status(Inclination::Status status);



    Inclination::Inclination_State &state;
    AP_Inclination_Params &params;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    //Type Backend initialised with
    Inclination::Type _backend_type;
};
