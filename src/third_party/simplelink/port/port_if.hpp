#pragma once

typedef void *Fd_t;
typedef void (* P_EVENT_HANDLER)(void *pValue);

// This API isn't used since we pass pointer to spi to sl_Start
Fd_t IfOpen(char *ifName, unsigned long flags);

int IfClose(Fd_t fd);

int IfRead(Fd_t fd, unsigned char *pBuff, int len);

int IfWrite(Fd_t fd, unsigned char *pBuff, int len);

int IfRegIntHdlr(P_EVENT_HANDLER InterruptHdl, void* pValue);

void IfMaskIntHdlr();

void IfUnMaskIntHdlr();
