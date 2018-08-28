#include "stddef.h"
#include "port_if.hpp"
#include "../user.h"

#include "drv/cc3100/cc3100.hpp"
#include "spi.hpp"

// This API isn't used since we pass pointer to spi to sl_Start
Fd_t IfOpen(char *ifName, unsigned long flags)
{
    return NULL;
}

int IfClose(Fd_t fd)
{
    return SL_OS_RET_CODE_OK;
}

int IfRead(Fd_t fd, unsigned char *pBuff, int len)
{
    hal::spi *spi = static_cast<hal::spi *>(fd);
    
    spi->rx(pBuff, len, NULL);
    
    return len;
}

int IfWrite(Fd_t fd, unsigned char *pBuff, int len)
{
    hal::spi *spi = static_cast<hal::spi *>(fd);
    
    spi->tx(pBuff, len, NULL);
    
    return len;
}

int IfRegIntHdlr(P_EVENT_HANDLER InterruptHdl, void *pValue)
{
    drv::cc3100::get_instance().set_exti_hndlr(InterruptHdl);

    return SL_OS_RET_CODE_OK;
}

void IfMaskIntHdlr()
{
    drv::cc3100::get_instance().exti_enable(false);
}

void IfUnMaskIntHdlr()
{
    drv::cc3100::get_instance().exti_enable(true);
}
