"""Run actual optional-property discovery against missing/malformed SMC keys."""
from pathlib import Path
import re
import subprocess
import tempfile
import sys

root = Path(__file__).resolve().parents[3]
source_path = Path(sys.argv[1]) if len(sys.argv) > 1 else root/'drivers/power/supply/macsmc-power.c'
source = source_path.read_text()
start = source.index('static int macsmc_battery_optional_properties(')
end = source.index('\nstatic int macsmc_power_probe(', start)
body = source[start:end]
entries = re.findall(r'BATT_OPTIONAL\((\w+), (\w+), (\d+)\)', body)
assert len(entries) == 16
properties = [p for p,k,n in entries]
keys = list(dict.fromkeys(k for p,k,n in entries))
header = r'''
#include <assert.h>
#include <stdint.h>
#include <errno.h>
typedef uint32_t smc_key;
typedef uint8_t u8;
struct apple_smc {};
struct macsmc_power {struct apple_smc *smc; void *dev;};
struct apple_smc_key_info {unsigned size,flags;};
#define APPLE_SMC_READABLE 0x80
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define dev_info(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define SMC_KEY(k) KEY_##k
'''
header += 'enum power_supply_property {' + ','.join('POWER_SUPPLY_PROP_'+p for p in properties) + '};\n'
header += 'enum keys {' + ','.join('KEY_'+k for k in keys) + '};\n'
header += r'''
static int mode, reads;
static int apple_smc_get_key_info(struct apple_smc *s,smc_key key,struct apple_smc_key_info *i) {
 i->size=key==KEY_BAAC?8:2; i->flags=0x80;
 if (mode) {
   if(key==KEY_BITV) return -ENOENT;
   if(key==KEY_B0RC) i->size=4;
   if(key==KEY_BVVN) i->flags=0;
 }
 return 0;
}
static int apple_smc_read(struct apple_smc *s,smc_key key,void *v,unsigned n) {
 reads++;
 if(mode && key==KEY_BLPM) return -EIO;
 if(mode && key==KEY_BLPX) return n-1;
 return n;
}
'''
tests = r'''
int main(void) {
 struct macsmc_power p={0}; enum power_supply_property props[50];
 props[0]=999; props[1]=998;
 assert(macsmc_battery_optional_properties(&p,props,2)==18);
 assert(reads==16 && props[0]==999 && props[1]==998);
 mode=1; reads=0;
 int n=macsmc_battery_optional_properties(&p,props,2);
 assert(n==13 && reads==13);
 for(int i=2;i<n;i++) {
   assert(props[i]!=POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN);
   assert(props[i]!=POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN);
   assert(props[i]!=POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT);
   assert(props[i]!=POWER_SUPPLY_PROP_VOLTAGE_MIN);
   assert(props[i]!=POWER_SUPPLY_PROP_VOLTAGE_MAX);
 }
}
'''
with tempfile.TemporaryDirectory() as directory:
    src, binary = Path(directory)/'test.c', Path(directory)/'test'
    src.write_text(header+body+tests)
    subprocess.run(['/usr/bin/cc','-fsanitize=address,undefined',str(src),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('Optional battery properties: missing, wrong-size, unreadable, failed and short-read keys omitted; working fields retained: PASS')
