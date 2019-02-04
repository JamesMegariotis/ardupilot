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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_TerraRangerSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <AP_Math/crc.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_TerraRangerSerial::AP_RangeFinder_TerraRangerSerial(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IR, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_IR, 0));
    }
}

/*
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_TerraRangerSerial::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_IR, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_TerraRangerSerial::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    float sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes > 0) {
        linebuf[0] = uart->read();
        nbytes--;
        if ((linebuf[0] == 0x54) && (nbytes > 3)) {
            linebuf[1] = uart->read();
            linebuf[2] = uart->read();
            uint8_t checksum8 = uart->read();
            nbytes = nbytes - 3;
            if (checksum8 == crc_crc8(linebuf, 3)){
				uint16_t val = ((linebuf[1] << 8) | linebuf[2]);
				sum += (float)(val);
				count++;
				memset(linebuf, 0, sizeof(linebuf));
            }
        }
    }

    if (count == 0) {
        return false;
    }
    reading_cm = (sum / count) / 10;
    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_TerraRangerSerial::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
