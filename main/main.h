#pragma once

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"

class Main final {
public: 
    Main(void);
    void run ();
};