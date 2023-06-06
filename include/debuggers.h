#pragma once

#include <ArduinoEigen.h>

namespace DEBUGGER {
    const bool debug_io_mode = DEBUG & 0b00000001;
    const bool debug_type = (DEBUG & 0b00001110) >> 1;

    void imu_debugger(bool io_mode);
    void ahrs_debugger(bool io_mode);

    void debug_msg() {
        if(debug_type == 0b000) imu_debugger(debug_io_mode);
        else if(debug_type == 0b001) ahrs_debugger(debug_io_mode);
        else if(debug_type == 0b010) ;
        else if(debug_type == 0b011) ;
        else if(debug_type == 0b100) ;
        else if(debug_type == 0b101) ;
        else if(debug_type == 0b110) ;
        else if(debug_type == 0b111) ;
    }
}