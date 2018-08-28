#ifndef SIMPLELINK_PORT_LOCK_OBJ_H
#define SIMPLELINK_PORT_LOCK_OBJ_H

#include "third_party/FreeRTOS/include/FreeRTOS.h"
#include "third_party/FreeRTOS/include/semphr.h"

int8_t LockObjCreate(SemaphoreHandle_t *pLockObj, char *pName);

int8_t LockObjDelete(SemaphoreHandle_t *pLockObj);

int8_t LockObjLock(SemaphoreHandle_t *pLockObj, TickType_t Timeout);

int8_t LockObjUnlock(SemaphoreHandle_t *pLockObj);

#endif

