#include "easy_log.h"
#include <stdarg.h>

void LOGI(const char *fmt, ...)
{
#ifdef DEBUG_INFO
    va_list args;
    printf("[%lu.%03lu][I] ", HAL_GetTick() / 1000, HAL_GetTick() % 1000);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
#endif
}

void LOGW(const char *fmt, ...)
{
#ifdef DEBUG_WARN
    va_list args;
    printf("[%lu.%03lu][W] ", HAL_GetTick() / 1000, HAL_GetTick() % 1000);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
#endif
}

void LOGE(const char *fmt, ...)
{
#ifdef DEBUG_ERROR
    va_list args;
    printf("[%lu.%03lu][E] ", HAL_GetTick() / 1000, HAL_GetTick() % 1000);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
#endif
}