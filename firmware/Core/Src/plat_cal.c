#include "plat_cal.h"
#include "plat_hw.h"
#include <string.h>

#define CAL_MAGIC    0x50414553u        /* "SEAP" */
#define CAL_VERSION  1u
#define CAL_WORDS    16u

Cal g_cal;

static uint32_t crc32_words(const uint32_t *w, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++)
    {
        crc ^= w[i];
        for (int b = 0; b < 32; b++) crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
    return ~crc;
}

void cal_defaults(void)
{
    g_cal.min[0] = DEF_MIN_LEG0;  g_cal.min[1] = DEF_MIN_LEG1;  g_cal.min[2] = DEF_MIN_LEG2;
    g_cal.max[0] = DEF_MAX_LEG0;  g_cal.max[1] = DEF_MAX_LEG1;  g_cal.max[2] = DEF_MAX_LEG2;
    g_cal.vmax = VMAX_DEFAULT;
    g_cal.amax = AMAX_DEFAULT;
    g_cal.pitch_pos_cdeg = g_cal.pitch_neg_cdeg = g_cal.roll_pos_cdeg = g_cal.roll_neg_cdeg = 0;
    g_cal.max_diff = 0;
    g_cal.from_flash = 0;
}

int cal_valid(const Cal *c)
{
    for (int i = 0; i < NUM_LEGS; i++)
    {
        if (c->min[i] < TRAVEL_FLOOR_STEPS || c->max[i] > TRAVEL_CEIL_STEPS) return 0;
        if (c->max[i] - c->min[i] < 500) return 0;
    }
    const int32_t tl[4] = { c->pitch_pos_cdeg, c->pitch_neg_cdeg, c->roll_pos_cdeg, c->roll_neg_cdeg };
    for (int i = 0; i < 4; i++)
        if (tl[i] != 0 && (tl[i] < MIN_TILT_CDEG || tl[i] > 9000)) return 0;
    if (c->max_diff != 0 && (c->max_diff < MIN_SPREAD_STEPS || c->max_diff > TRAVEL_CEIL_STEPS)) return 0;
    if (c->vmax < 50 || c->vmax > VMAX_LIMIT) return 0;
    if (c->amax < 100 || c->amax > AMAX_LIMIT) return 0;
    return 1;
}

void cal_load(void)
{
    uint32_t w[CAL_WORDS];
    cal_defaults();
    if (hw_flash_read(w, sizeof(w)) != 0) return;
    if (w[0] != CAL_MAGIC || w[1] != CAL_VERSION) return;
    if (crc32_words(w, CAL_WORDS - 1u) != w[CAL_WORDS - 1u]) return;

    Cal c;
    memcpy(c.min, &w[2], sizeof(c.min));
    memcpy(c.max, &w[5], sizeof(c.max));
    c.vmax = (int32_t)w[8];
    c.amax = (int32_t)w[9];
    c.pitch_pos_cdeg = (int32_t)w[10];     /* words 10..14 were zero in records written before these limits existed */
    c.pitch_neg_cdeg = (int32_t)w[11];
    c.roll_pos_cdeg  = (int32_t)w[12];
    c.roll_neg_cdeg  = (int32_t)w[13];
    c.max_diff       = (int32_t)w[14];
    c.from_flash = 1;
    if (cal_valid(&c)) g_cal = c;
}

int cal_save(void)
{
    if (!cal_valid(&g_cal)) return -2;
    uint32_t w[CAL_WORDS];
    memset(w, 0, sizeof(w));
    w[0] = CAL_MAGIC;
    w[1] = CAL_VERSION;
    memcpy(&w[2], g_cal.min, sizeof(g_cal.min));
    memcpy(&w[5], g_cal.max, sizeof(g_cal.max));
    w[8] = (uint32_t)g_cal.vmax;
    w[9] = (uint32_t)g_cal.amax;
    w[10] = (uint32_t)g_cal.pitch_pos_cdeg;
    w[11] = (uint32_t)g_cal.pitch_neg_cdeg;
    w[12] = (uint32_t)g_cal.roll_pos_cdeg;
    w[13] = (uint32_t)g_cal.roll_neg_cdeg;
    w[14] = (uint32_t)g_cal.max_diff;
    w[CAL_WORDS - 1u] = crc32_words(w, CAL_WORDS - 1u);
    int rc = hw_flash_write(w, sizeof(w));
    if (rc == 0) g_cal.from_flash = 1;
    return rc;
}
