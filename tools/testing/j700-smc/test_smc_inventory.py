"""Execute the real metadata inventory with size, time, and protocol failures."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (Path(os.environ.get('NEO_TEST_TREE', root))/'drivers/mfd/macsmc.c').read_text()
start = source.index('static int apple_smc_j700_log_sensor_inventory(')
end = source.index('\n/* Existing macsmc-power read keys', start)
header = r'''
#include <assert.h>
#include <stdint.h>
#include <errno.h>
typedef uint32_t smc_key;
struct apple_smc { void *dev; unsigned int key_count; };
struct apple_smc_key_info { unsigned int size, type_code, flags; };
static unsigned long jiffies;
static unsigned int index_calls, info_calls, log_calls, mode;
#define msecs_to_jiffies(n) (n)
#define time_after(a, b) ((long)((b) - (a)) < 0)
#define dev_info(...) (log_calls++)
static int apple_smc_get_key_by_index(struct apple_smc *s, unsigned int i, smc_key *key) {
    (void)s; index_calls++;
    *key = mode == 1 ? 0x54000000 | i : mode == 5 ? (i < 150 ? 0x42000000 : 0x54000000) | i : i == 0 ? 0x42304156 : i == 1 ? 0x54703031 : 0x7063494f;
    if (mode == 2) jiffies += 6000;
    return mode == 3 ? -EIO : 0;
}
static int apple_smc_get_key_info(struct apple_smc *s, smc_key key, struct apple_smc_key_info *info) {
    (void)s; (void)key; info_calls++;
    *info = (struct apple_smc_key_info){.size=4,.flags=0x80};
    return mode == 4 ? -EIO : 0;
}
/* No apple_smc_read or write stub: adding value I/O must fail to link. */
'''
tests = r'''
int main(void) {
    struct apple_smc smc = {.key_count=3};
    assert(apple_smc_j700_log_sensor_inventory(&smc) == 0);
    assert(index_calls == 3 && info_calls == 2 && log_calls == 4);
    smc.key_count=4097; index_calls=0;
    assert(apple_smc_j700_log_sensor_inventory(&smc) == -E2BIG);
    assert(index_calls == 0);
    smc.key_count=200; mode=1; index_calls=info_calls=log_calls=0;
    assert(apple_smc_j700_log_sensor_inventory(&smc) == 0);
    assert(index_calls == 200 && info_calls == 64 && log_calls == 66);
    smc.key_count=250; mode=5; index_calls=info_calls=log_calls=0;
    assert(apple_smc_j700_log_sensor_inventory(&smc) == 0);
    assert(index_calls == 214 && info_calls == 128 && log_calls == 130);
    for (mode=2; mode<=4; mode++) {
        jiffies=index_calls=info_calls=log_calls=0;
        assert(apple_smc_j700_log_sensor_inventory(&smc) == (mode == 2 ? -ETIMEDOUT : -EIO));
        assert(index_calls == 1);
    }
}
'''
with tempfile.TemporaryDirectory() as directory:
    src, binary = Path(directory)/'test.c', Path(directory)/'test'
    src.write_text(header + source[start:end] + tests)
    subprocess.run(['/usr/bin/cc', '-fsanitize=address,undefined', str(src), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('SMC metadata inventory bounds, filtering, timeout, and errors: PASS')
