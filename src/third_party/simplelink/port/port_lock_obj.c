#include "port_lock_obj.h"
#include "../user.h"

#define SL_OS_RET_CODE_ERR !SL_OS_RET_CODE_OK

int8_t LockObjCreate(SemaphoreHandle_t *pLockObj, char *pName)
{
    *pLockObj = xSemaphoreCreateMutex();
    
    return *pLockObj ? SL_OS_RET_CODE_OK : SL_OS_RET_CODE_ERR;
}

int8_t LockObjDelete(SemaphoreHandle_t *pLockObj)
{
    vSemaphoreDelete(*pLockObj);
    
    return SL_OS_RET_CODE_OK;
}

int8_t LockObjLock(SemaphoreHandle_t *pLockObj, TickType_t Timeout)
{
    return xSemaphoreTake(*pLockObj, Timeout / portTICK_PERIOD_MS) ?
        SL_OS_RET_CODE_OK : SL_OS_RET_CODE_ERR;
}

int8_t LockObjUnlock(SemaphoreHandle_t *pLockObj)
{
    return xSemaphoreGive(*pLockObj) ? SL_OS_RET_CODE_OK : SL_OS_RET_CODE_ERR;
}

