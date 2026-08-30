#ifndef __EASY_LOG_H
#define __EASY_LOG_H

#include "sys.h"
#include <stdio.h>

void LOGI(const char *fmt, ...);
void LOGW(const char *fmt, ...);
void LOGE(const char *fmt, ...);

#endif