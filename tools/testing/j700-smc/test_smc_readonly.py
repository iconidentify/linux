"""Check default write blocking and the explicit NTAP-only notification opt-in."""
from pathlib import Path
import subprocess
import tempfile
import os

root = Path(__file__).resolve().parents[3]
tree = Path(os.environ.get('NEO_TEST_TREE', root))
source = (tree/'drivers/mfd/macsmc.c').read_text()

def function(signature):
    start = source.index(signature)
    end = source.index('\n}', start) + 2
    return source[start:end]

header = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint32_t smc_key;
typedef uint8_t u8;
struct apple_smc { bool read_only, notifications_only, atomic_mode, atomic_pending; int mutex, lock;
 int boot_stage, msg_id; u64 cmd_ret; void *rtk, *dev; struct {void *iomem;} shmem; };
static unsigned io_writes, commands;
static struct apple_smc *active;
#define guard(x) (void)
#define lockdep_assert_held(x) ((void)0)
#define SMC_MAX_SIZE 256
#define SMC_MSG_RW_KEY 1
#define SMC_MSG_WRITE_KEY 2
#define SMC_MSG_READ_KEY 3
#define APPLE_SMC_INITIALIZED 1
#define SMC_ENDPOINT 1
#define SMC_KEY(x) ((smc_key)((#x)[0]<<24 | (#x)[1]<<16 | (#x)[2]<<8 | (#x)[3]))
#define SMC_MSG 0
#define SMC_SIZE 1
#define SMC_ID 2
#define SMC_DATA 3
#define SMC_RESULT 4
#define FIELD_PREP(a,b) ((u64)(b))
#define FIELD_GET(a,b) ((a) == SMC_ID ? active->msg_id : (a) == SMC_SIZE ? 1 : 0)
#define dev_err(...) ((void)0)
#define udelay(n) ((void)0)
static void memcpy_toio(void *p, const void *b, size_t n) {io_writes++; memcpy(p,b,n);}
#define memcpy_fromio memcpy
static int apple_smc_cmd_locked(struct apple_smc *s,u64 c,u64 k,u64 n,u64 w,u32 *v) {
 commands++; *v=0x12345678; return n;
}
static int apple_rtkit_send_message(void *r,int e,u64 m,void *x,bool a) {
 commands++; active->atomic_pending=false; return 0;
}
static int apple_rtkit_poll(void *r) {return 0;}
'''
tests = r'''
int main(void) {
 unsigned char memory[256]={0}, value[8]={1}, output[8]={0};
 struct apple_smc smc={.read_only=true,.shmem={memory},.boot_stage=1}; active=&smc;
 for (size_t n=1;n<=256;n++) {
   assert(apple_smc_rw_locked(&smc,0,value,n,NULL,0)==-EPERM);
   assert(apple_smc_rw_locked(&smc,0,value,n,output,1)==-EPERM);
   assert(apple_smc_write_atomic(&smc,0,value,n)==-EPERM);
 }
 assert(apple_smc_enter_atomic(&smc)==-EPERM);
 assert(!smc.atomic_mode && !smc.atomic_pending && commands==0 && io_writes==0);
 assert(apple_smc_rw_locked(&smc,0,NULL,0,output,4)==4);
 assert(commands==1 && io_writes==0 && output[0]==0x78);
 smc.notifications_only=true;
 for (size_t n=1;n<=256;n++) {
   assert(apple_smc_rw_locked(&smc,SMC_KEY(CH0B),value,n,NULL,0)==-EPERM);
   assert(apple_smc_rw_locked(&smc,SMC_KEY(NTAP),value,n,output,1)==-EPERM);
   assert(apple_smc_write_atomic(&smc,SMC_KEY(NTAP),value,n)==-EPERM);
   if (n!=1) assert(apple_smc_rw_locked(&smc,SMC_KEY(NTAP),value,n,NULL,0)==-EPERM);
 }
 assert(apple_smc_rw_locked(&smc,SMC_KEY(NTAP),NULL,1,NULL,0)==-EPERM);
 for (unsigned v=2;v<256;v++) {
   value[0]=v;
   assert(apple_smc_rw_locked(&smc,SMC_KEY(NTAP),value,1,NULL,0)==-EPERM);
 }
 assert(commands==1 && io_writes==0);
 for (unsigned v=0;v<=1;v++) {
   value[0]=v;
   assert(apple_smc_rw_locked(&smc,SMC_KEY(NTAP),value,1,NULL,0)==1);
   assert(memory[0]==v);
 }
 assert(commands==3 && io_writes==2);
 assert(apple_smc_enter_atomic(&smc)==-EPERM && !smc.atomic_mode);
 smc.notifications_only=false;
 commands=1; io_writes=0;
 smc.read_only=false;
 assert(apple_smc_rw_locked(&smc,0,value,1,NULL,0)==1);
 assert(io_writes==1 && commands==2 && memory[0]==1);
 assert(apple_smc_enter_atomic(&smc)==0 && smc.atomic_mode);
 assert(apple_smc_write_atomic(&smc,0,value,1)==1);
 assert(io_writes==3 && commands==4);
}
'''
body = '\n'.join(function(s) for s in ['static int apple_smc_rw_locked(',
       'int apple_smc_enter_atomic(', 'int apple_smc_write_atomic('])
with tempfile.TemporaryDirectory() as directory:
    src, binary = Path(directory)/'test.c', Path(directory)/'test'
    src.write_text(header+body+tests)
    subprocess.run(['/usr/bin/cc','-fsanitize=address,undefined',str(src),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('SMC read-only and NTAP-only gates: all other keys/payloads/RW/atomic paths blocked before I/O; normal paths retained: PASS')
