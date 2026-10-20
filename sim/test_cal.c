/* Calibration storage: save / load round trip, records written before the safety limits existed,
 * damaged records and invalid values. */
#include "plat_cal.h"
#include "plat_hw.h"
#include "hw_host.h"
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

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

/* the 16-word record exactly as the first firmware release wrote it: words 10..14 were zero */
static void write_legacy_record(void)
{
    uint32_t w[16];
    memset(w, 0, sizeof w);
    w[0] = 0x50414553u; w[1] = 1u;
    w[2] = 700; w[3] = 710; w[4] = 720;
    w[5] = 8400; w[6] = 8410; w[7] = 8420;
    w[8] = 1200; w[9] = 1300;
    w[15] = crc32_words(w, 15);
    hw_flash_write(w, sizeof w);
}

int main(void)
{
    sim_echo = 0;
    sim_reset(0, 0, 0);

    printf("[defaults]\n");
    cal_defaults();
    CHECK(g_cal.pitch_pos_cdeg == 0 && g_cal.pitch_neg_cdeg == 0 && g_cal.roll_pos_cdeg == 0 && g_cal.roll_neg_cdeg == 0 && g_cal.max_diff == 0, "limits should start unset");
    CHECK(cal_valid(&g_cal), "defaults must be valid");

    printf("[record from the first release]\n");
    write_legacy_record();
    cal_load();
    CHECK(g_cal.from_flash, "old record not accepted");
    CHECK(g_cal.min[0] == 700 && g_cal.max[2] == 8420 && g_cal.vmax == 1200 && g_cal.amax == 1300, "old record values wrong");
    CHECK(g_cal.pitch_pos_cdeg == 0 && g_cal.roll_neg_cdeg == 0 && g_cal.max_diff == 0, "old record should leave the limits unset");

    printf("[round trip with limits]\n");
    g_cal.pitch_pos_cdeg = 1234; g_cal.pitch_neg_cdeg = 800; g_cal.roll_pos_cdeg = 950; g_cal.roll_neg_cdeg = 1100; g_cal.max_diff = 1750;
    CHECK(cal_save() == 0, "save failed");
    cal_defaults();
    CHECK(g_cal.max_diff == 0, "defaults did not clear the limits");
    cal_load();
    CHECK(g_cal.from_flash && g_cal.pitch_pos_cdeg == 1234 && g_cal.pitch_neg_cdeg == 800 && g_cal.roll_pos_cdeg == 950 && g_cal.roll_neg_cdeg == 1100 && g_cal.max_diff == 1750,
          "limits not restored: %ld %ld %ld %ld %ld", (long)g_cal.pitch_pos_cdeg, (long)g_cal.pitch_neg_cdeg, (long)g_cal.roll_pos_cdeg, (long)g_cal.roll_neg_cdeg, (long)g_cal.max_diff);
    CHECK(g_cal.min[0] == 700 && g_cal.vmax == 1200, "the older values were lost");

    printf("[invalid values are refused]\n");
    Cal c = g_cal;
    c.pitch_pos_cdeg = 50;   CHECK(!cal_valid(&c), "0.5 deg tilt limit should be refused");
    c = g_cal; c.roll_neg_cdeg = 9500; CHECK(!cal_valid(&c), "95 deg tilt limit should be refused");
    c = g_cal; c.max_diff = 100;       CHECK(!cal_valid(&c), "100 step difference limit should be refused");
    c = g_cal; c.max_diff = TRAVEL_CEIL_STEPS + 1; CHECK(!cal_valid(&c), "difference limit above the travel should be refused");
    c = g_cal; c.pitch_pos_cdeg = 0; c.max_diff = 0; CHECK(cal_valid(&c), "zero (= not set) must be valid");
    { Cal bad = g_cal; bad.pitch_pos_cdeg = 20; Cal keep = g_cal; g_cal = bad; CHECK(cal_save() == -2, "an invalid record must not be saved"); g_cal = keep; }

    printf("[damaged record]\n");
    {
        uint32_t w[16];
        hw_flash_read(w, sizeof w);
        w[11] ^= 0x4u;                                    /* flip a bit, leave the CRC alone */
        hw_flash_write(w, sizeof w);
        cal_load();
        CHECK(!g_cal.from_flash && g_cal.max_diff == 0, "a damaged record must be ignored (defaults)");
    }

    printf("\n%s (%d failures)\n", fails ? "SOME CAL TESTS FAILED" : "ALL CAL TESTS PASSED", fails);
    return fails ? 1 : 0;
}
