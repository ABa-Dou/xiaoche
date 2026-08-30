#include "cmd.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static bool match_key(const char *s, const char *key, const char **val_start)
{
    size_t key_len = strlen(key);
    if (strncmp(s, key, key_len) == 0 && s[key_len] == '=')
    {
        *val_start = s + key_len + 1;
        return true;
    }
    return false;
}

static bool try_parse_float(const char *val_start, const char *val_end, float *out)
{
    char tmp[32];
    size_t len = (size_t)(val_end - val_start);
    if (len == 0 || len >= sizeof(tmp)) return false;
    memcpy(tmp, val_start, len);
    tmp[len] = '\0';

    char *endptr = NULL;
    *out = strtof(tmp, &endptr);
    return (endptr == tmp + len);
}

bool cmd_parse(const uint8_t *buf, uint16_t len, cmd_params_t *params)
{
    if (buf == NULL || len < 3 || params == NULL) return false;

    memset(params, 0, sizeof(*params));

    if (strncmp((const char *)buf, "pid", 3) != 0) return false;

    const char *p = (const char *)buf + 3;

    if (*p >= '1' && *p <= '4' && (*(p + 1) == ' ' || *(p + 1) == '\t'))
    {
        params->motor_id = (uint8_t)(*p - '0');
        p++;
    }

    while (*p != '\0')
    {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') break;

        const char *key_start = p;
        while (*p != '\0' && *p != '=' && *p != ' ' && *p != '\t') p++;
        const char *key_end = p;

        if (*p != '=') break;

        const char *val_start = p + 1;
        p++;
        while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') p++;
        const char *val_end = p;

        float val = 0.0f;
        if (!try_parse_float(val_start, val_end, &val)) continue;

        size_t klen = (size_t)(key_end - key_start);

        if (klen == 2 && strncmp(key_start, "kp", 2) == 0)
        {
            params->kp = val;
            params->has_kp = true;
        }
        else if (klen == 2 && strncmp(key_start, "ki", 2) == 0)
        {
            params->ki = val;
            params->has_ki = true;
        }
        else if (klen == 2 && strncmp(key_start, "kd", 2) == 0)
        {
            params->kd = val;
            params->has_kd = true;
        }
        else if (klen == 3 && strncmp(key_start, "tgt", 3) == 0)
        {
            params->tgt_speed_mm_s = val;
            params->has_tgt = true;
        }
        else if (klen == 4 && strncmp(key_start, "ilim", 4) == 0)
        {
            params->integral_limit = val;
            params->has_ilim = true;
        }
        else if (klen == 3 && strncmp(key_start, "sep", 3) == 0)
        {
            params->integral_separation_err = val;
            params->has_sep = true;
        }
        else if (klen == 3 && strncmp(key_start, "flt", 3) == 0)
        {
            params->filter_alpha = val;
            params->has_flt = true;
        }
        else if (klen == 2 && strncmp(key_start, "ff", 2) == 0)
        {
            params->ff_gain = val;
            params->has_ff = true;
        }
    }

    return (params->has_kp || params->has_ki || params->has_kd ||
            params->has_ff || params->has_tgt || params->has_ilim ||
            params->has_sep || params->has_flt);
}