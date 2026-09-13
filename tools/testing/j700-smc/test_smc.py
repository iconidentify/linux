"""Exercise the actual diagnostic whitelist; reject functions and malformed reads."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]

class SmcTest(unittest.TestCase):
    def test_read_only_power_probe(self):
        source = (ROOT / 'drivers/mfd/macsmc.c').read_text()
        start = source.index('static int apple_smc_j700_log_power(')
        end = source.index('\nstatic int apple_smc_probe(', start)
        function = source[start:end]
        header = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define APPLE_SMC_READABLE 0x80
#define APPLE_SMC_FUNCTION 0x10
struct apple_smc { void *dev; };
struct apple_smc_key_info { unsigned int size, type_code, flags; };
static unsigned int info_calls, reads, logged, warnings, mode;
static uint32_t _SMC_KEY(const char *key) {
    return ((uint32_t)key[0] << 24) | ((uint32_t)key[1] << 16) |
           ((uint32_t)key[2] << 8) | (uint32_t)key[3];
}
static int apple_smc_get_key_info(struct apple_smc *s, uint32_t key,
                                struct apple_smc_key_info *info) {
    (void)s; (void)key; info_calls++;
    info->size = mode == 1 ? 9 : 2;
    info->flags = mode == 2 ? APPLE_SMC_FUNCTION | APPLE_SMC_READABLE : APPLE_SMC_READABLE;
    if (mode == 5 && key == _SMC_KEY("BUIC"))
        info->flags |= APPLE_SMC_FUNCTION;
    info->type_code = 0;
    return mode == 3 ? -ETIMEDOUT : 0;
}
static int apple_smc_read(struct apple_smc *s, uint32_t key, void *buf, unsigned int size) {
    (void)s;
    const char *keys[] = {"B0AV", "B0AC", "B0AP", "BUIC", "B0AT", "B0CT", "ACPW"};
    assert(reads < ARRAY_SIZE(keys));
    assert(key == _SMC_KEY(keys[info_calls - 1]));
    assert(mode != 5 || key != _SMC_KEY("BUIC"));
    reads++;
    memset(buf, 0, size);
    return mode == 4 ? size - 1 : (int)size;
}
#define dev_info(...) (logged++)
#define dev_warn(...) (warnings++)
'''
        tests = r'''
int main(void) {
    struct apple_smc smc = {0};
    assert(apple_smc_j700_log_power(&smc) == 0);
    assert(reads == 7 && logged == 14 && info_calls == 7 && warnings == 0);
    for (mode = 1; mode <= 4; mode++) {
        reads = logged = info_calls = warnings = 0;
        int expected = mode == 3 ? -ETIMEDOUT : mode == 4 ? -EIO : -EINVAL;
        assert(apple_smc_j700_log_power(&smc) == expected);
        assert(info_calls == (mode <= 2 ? 7 : 1));
        assert(logged == (mode <= 2 ? 7 : mode == 4 ? 1 : 0));
        assert(warnings == (mode <= 2 ? 7 : 0));
        assert(reads == (mode == 4 ? 1 : 0));
    }
    mode = 5; reads = logged = info_calls = warnings = 0;
    assert(apple_smc_j700_log_power(&smc) == -EINVAL);
    assert(info_calls == 7 && reads == 6 && logged == 13 && warnings == 1);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            src, binary = Path(directory) / 'test.c', Path(directory) / 'test'
            src.write_text(header + function + tests)
            subprocess.run(['/usr/bin/cc', '-fsanitize=address,undefined',
                            str(src), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

if __name__ == '__main__':
    unittest.main()
