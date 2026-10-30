/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "app_runtime.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <string.h>
static bool newer(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) > 0;
}
static bool load_at(uint32_t address, AppParam *p, uint32_t *sequence) {
    return param_decode((const uint8_t *)address, p, sequence);
}
bool param_store_load(AppParam *p) {
    AppParam a, b;
    uint32_t sa = 0, sb = 0;
    bool va = load_at(APP_PARAM_SLOT0, &a, &sa), vb = load_at(APP_PARAM_SLOT1, &b, &sb);
    if (!va && !vb)
        return false;
    *p = va && (!vb || newer(sa, sb)) ? a : b;
    return true;
}
bool param_store_save(const AppParam *p) {
    ControlSnapshot s;
    app_exchange_read_control(&s);
    if (s.mode != RUN_STATIC_DEBUG || !s.feedback.valid || fabsf(s.feedback.speed_mps[0]) > .01f ||
        fabsf(s.feedback.speed_mps[1]) > .01f || !param_valid(p))
        return false;
    AppParam a, b;
    uint32_t sa = 0, sb = 0;
    bool va = load_at(APP_PARAM_SLOT0, &a, &sa), vb = load_at(APP_PARAM_SLOT1, &b, &sb);
    bool latest_a = va && (!vb || newer(sa, sb));
    uint32_t target = latest_a ? APP_PARAM_SLOT1 : APP_PARAM_SLOT0;
    uint32_t sequence = (va || vb) ? (latest_a ? sa : sb) + 1u : 1u;
    uint8_t record[APP_PARAM_RECORD_BYTES];
    if (!param_encode(p, sequence, record))
        return false;
    BoardIrqToken token = board_irq_save();
    FLASH_Unlock();
    bool ok = FLASH_ErasePage(target) == FLASH_COMPLETE;
    for (unsigned i = 0; i < APP_PARAM_RECORD_BYTES - 4 && ok; i += 4) {
        uint32_t word;
        memcpy(&word, record + i, 4);
        ok = FLASH_ProgramWord(target + i, word) == FLASH_COMPLETE;
    }
    if (ok)
        ok = memcmp((const void *)target, record, APP_PARAM_RECORD_BYTES - 4) == 0;
    if (ok) {
        uint32_t marker;
        memcpy(&marker, record + APP_PARAM_RECORD_BYTES - 4, 4);
        ok = FLASH_ProgramWord(target + APP_PARAM_RECORD_BYTES - 4, marker) == FLASH_COMPLETE;
    }
    FLASH_Lock();
    board_irq_restore(token);
    return ok && load_at(target, &a, &sa) && sa == sequence;
}
