#include "port_device.hpp"

#include "drv/cc3100/cc3100.hpp"

void DeviceEnablePreamble()
{
    
}

void DeviceEnable()
{
    drv::cc3100::get_instance().enable(true);
}

void DeviceDisable()
{
    drv::cc3100::get_instance().enable(false);
}
