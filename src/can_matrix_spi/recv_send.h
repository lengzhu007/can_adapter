#include "spi_dbus_hal.h"
#include "service_handler_module.h"
#include <memory>
#include "stdlib.h"
#include "string.h"
#include <iostream>
#include <stdio.h>
int recv_can_spi(SpiCanFrameType & spi_can_frame);
int send_can_spi(OutputCAN * outputcan);

