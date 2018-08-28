#include "port_sync_obj.h"
#include "../user.h"

#define SL_OS_RET_CODE_ERR !SL_OS_RET_CODE_OK

int8_t SyncObjCreate(SemaphoreHandle_t *pSyncObj, char *pName)
{
    *pSyncObj = xSemaphoreCreateBinary();
    
    return *pSyncObj ? SL_OS_RET_CODE_OK : SL_OS_RET_CODE_ERR;
}

int8_t SyncObjDelete(SemaphoreHandle_t *pSyncObj)
{
    vSemaphoreDelete(*pSyncObj);
    
    return SL_OS_RET_CODE_OK;
}

int8_t SyncObjSignal(SemaphoreHandle_t *pSyncObj)
{
    return xSemaphoreGive(*pSyncObj) ? SL_OS_RET_CODE_OK : SL_OS_RET_CODE_ERR;
}

int8_t SyncObjSignalFromIRQ(SemaphoreHandle_t *pSyncObj)
{
    BaseType_t hi_task_woken = 0;
    
    xSemaphoreGiveFromISR(*pSyncObj, &hi_task_woken);
    portYIELD_FROM_ISR(hi_task_woken);
    
    return SL_OS_RET_CODE_OK;
}

int8_t SyncObjWait(SemaphoreHandle_t *pSyncObj, TickType_t Timeout)
{
    return xSemaphoreTake(*pSyncObj, Timeout / portTICK_PERIOD_MS) ?
        SL_OS_RET_CODE_OK : SL_OS_RET_CODE_ERR;
}
