# SPDX-License-Identifier: GPL-2.0
"""Run the driver's MTP parser with synthetic reports and mocked input I/O.

Standalone: python3 test_magicmouse_j700_parser.py. Requires a C compiler with
AddressSanitizer and UndefinedBehaviorSanitizer; no HID device is needed.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[5]


class J700ParserTest(unittest.TestCase):
    def test_positions_presence_release_and_bounds(self):
        compiler = shutil.which("clang") or shutil.which("cc")
        if compiler is None:
            self.skipTest("C compiler required")
        driver = ROOT / "drivers/hid/hid-magicmouse.c"
        if not driver.is_file():
            self.skipTest("kernel source checkout required")
        source = driver.read_text()
        constants = "\n".join(
            line for line in source.splitlines()
            if re.match(r"#define (MAX_CONTACTS|MTP_REPORT_ID|J700_MTP_\w+|"
                        r"J314_TP_MAX_FINGER_ORIENTATION)\s", line)
        )
        structs = source[source.index("struct tp_finger {"):
                         source.index("/**\n * struct standard HID mouse report")]
        start = source.index("static void report_finger_data(")
        functions = source[start:source.index("\nstatic int magicmouse_raw_event_spi", start)]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t u8;
typedef uint16_t __le16;
typedef uint32_t u32;
typedef int16_t s16;
#define MT_TOOL_FINGER 1
#define BTN_MOUSE 1
static int le16_to_int(__le16 v) { return (int16_t)v; }
static uint16_t get_unaligned_le16(const void *p) {
    const uint8_t *b = p;
    return b[0] | ((uint16_t)b[1] << 8);
}
#define hid_warn(...) ((void)0)
#define hid_dbg(...) ((void)0)
enum { ABS_MT_TOUCH_MAJOR, ABS_MT_TOUCH_MINOR, ABS_MT_WIDTH_MAJOR,
       ABS_MT_WIDTH_MINOR, ABS_MT_ORIENTATION, ABS_MT_PRESSURE,
       ABS_MT_POSITION_X, ABS_MT_POSITION_Y };
struct input_dev {
    int slot, count, button, syncs, unsupported;
    int x[MAX_CONTACTS], y[MAX_CONTACTS], pressure[MAX_CONTACTS];
    bool touched[MAX_CONTACTS];
};
struct input_mt_pos { int x, y; };
struct magicmouse_sc {
    struct input_dev *input;
    bool j700_mtp;
    struct input_mt_pos pos[MAX_CONTACTS];
    int tracking_ids[MAX_CONTACTS];
};
struct hid_device { struct magicmouse_sc *msc; };
struct hid_report { int unused; };
static void *hid_get_drvdata(struct hid_device *h) { return h->msc; }
static void input_mt_slot(struct input_dev *d, int slot) { d->slot = slot; }
static void input_mt_report_slot_state(struct input_dev *d, int tool, bool active) {
    (void)tool; d->touched[d->slot] = active;
}
static void input_report_abs(struct input_dev *d, int axis, int value) {
    if (axis == ABS_MT_POSITION_X) d->x[d->slot] = value;
    else if (axis == ABS_MT_POSITION_Y) d->y[d->slot] = value;
    else {
        d->unsupported++;
        if (axis == ABS_MT_PRESSURE) d->pressure[d->slot] = value;
    }
}
static void input_mt_assign_slots(struct input_dev *d, int *slots,
                                 struct input_mt_pos *pos, int n, int max) {
    (void)d; (void)pos; (void)max;
    for (int i = 0; i < n; i++) slots[i] = i;
}
static void input_mt_sync_frame(struct input_dev *d) {
    d->count = 0;
    for (int i = 0; i < MAX_CONTACTS; i++) {
        d->count += d->touched[i];
        d->touched[i] = false;
    }
}
static void input_report_key(struct input_dev *d, int key, int value) {
    (void)key; d->button = value;
}
static void input_sync(struct input_dev *d) { d->syncs++; }
'''
        cases = r'''
static void put16(unsigned char *p, int v) {
    p[0] = (uint16_t)v;
    p[1] = (uint16_t)v >> 8;
}
int main(void) {
    _Static_assert(sizeof(struct tp_header) == 38, "legacy header");
    _Static_assert(sizeof(struct tp_finger) == 30, "contact stride");
    struct input_dev input = {0};
    struct magicmouse_sc msc = {.input = &input, .j700_mtp = true};
    struct hid_device hid = { &msc };
    unsigned char frame[40 + 30 * 17] = {0};
    frame[0] = 0x75; frame[22] = 1;
    /* Absolute X rises while the old, incorrect fields move the other way. */
    for (int i = 0; i < 4; i++) {
        put16(frame + 36, -1000 + 500 * i);
        put16(frame + 38, 200 - 100 * i);
        put16(frame + 40, 150 - 100 * i);
        put16(frame + 42, -300 + 75 * i);
        assert(magicmouse_raw_event_mtp(&hid, NULL, frame, 70) == 1);
        assert(input.count == 1 && input.x[0] == -1000 + 500 * i);
        assert(input.y[0] == -200 + 100 * i && input.unsupported == 0);
    }
    frame[22] = 2; frame[23] = 1;
    put16(frame + 66, -32768); put16(frame + 68, 32767);
    assert(magicmouse_raw_event_mtp(&hid, NULL, frame, 100) == 1);
    assert(input.count == 2 && input.button == 1);
    assert(input.x[1] == -32768 && input.y[1] == -32767);
    frame[22] = 16;
    put16(frame + 36 + 15 * 30, 321);
    put16(frame + 38 + 15 * 30, -123);
    assert(magicmouse_raw_event_mtp(&hid, NULL, frame, 520) == 1);
    assert(input.count == 16 && input.x[15] == 321 && input.y[15] == 123);
    frame[22] = 0; frame[23] = 0;
    assert(magicmouse_raw_event_mtp(&hid, NULL, frame, 40) == 1);
    assert(input.count == 0 && input.button == 0);
    int syncs = input.syncs;
    frame[22] = 2;
    assert(magicmouse_raw_event_mtp(&hid, NULL, frame, 70) == 0);
    frame[22] = 17;
    assert(magicmouse_raw_event_mtp(&hid, NULL, frame, sizeof(frame)) == 0);
    frame[22] = 1; frame[0] = 0;
    assert(magicmouse_raw_event_mtp(&hid, NULL, frame, 70) == 0);
    assert(magicmouse_raw_event_mtp(&hid, NULL, NULL, -1) == 0);
    assert(magicmouse_raw_event_mtp(&hid, NULL, NULL, 0) == 0);
    assert(input.syncs == syncs);
    /* Exact allocations let ASan detect reads beyond every report length. */
    for (int size = 1; size <= 550; size++) {
        unsigned char *packet = calloc(1, size);
        assert(packet); packet[0] = 0x75;
        int expected = size >= 40 && (size - 40) % 30 == 0 && size <= 520;
        assert(magicmouse_raw_event_mtp(&hid, NULL, packet, size) == expected);
        free(packet);
    }
    /* Other MTP devices retain their touch-size filter and pressure reports. */
    msc.j700_mtp = false;
    unsigned char legacy[68] = {0};
    struct tp_finger finger = {.abs_x = 400, .abs_y = 500,
                              .touch_major = 20, .pressure = 77};
    legacy[22] = 1;
    memcpy(legacy + 38, &finger, sizeof(finger));
    assert(magicmouse_raw_event_mtp(&hid, NULL, legacy, 68) == 1);
    assert(input.count == 1 && input.x[0] == 400 && input.y[0] == -500);
    assert(input.pressure[0] == 77);
    finger.touch_major = 0;
    memcpy(legacy + 38, &finger, sizeof(finger));
    assert(magicmouse_raw_event_mtp(&hid, NULL, legacy, 68) == 1);
    assert(input.count == 0);
    frame[0] = 0x75;
    assert(magicmouse_raw_event_mtp(&hid, NULL, frame, 70) == 0);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            c_file = Path(directory) / "parser.c"
            binary = Path(directory) / "parser"
            c_file.write_text(constants + "\n" + harness + structs + functions + cases)
            subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-Wno-unused-parameter", "-Wno-sign-compare",
                            "-fsanitize=address,undefined", str(c_file), "-o", str(binary)],
                           check=True)
            subprocess.run([str(binary)], check=True, timeout=30)


if __name__ == "__main__":
    unittest.main()
