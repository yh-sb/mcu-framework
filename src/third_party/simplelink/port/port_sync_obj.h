#ifndef SIMPLELINK_PORT_SYNC_OBJ_H
#define SIMPLELINK_PORT_SYNC_OBJ_H

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

int8_t SyncObjCreate(SemaphoreHandle_t *pSyncObj, char *pName);

int8_t SyncObjDelete(SemaphoreHandle_t *pSyncObj);

int8_t SyncObjSignal(SemaphoreHandle_t *pSyncObj);

int8_t SyncObjSignalFromIRQ(SemaphoreHandle_t *pSyncObj);

int8_t SyncObjWait(SemaphoreHandle_t *pSyncObj, TickType_t Timeout);

#endif
