# J700 SMC battery and temperature qualification

This branch preserves the local September 5, 2026 SMC patch series on the
previously published J700 base
`bedd2558acdf8deef4bd4ad2c3cf41873f43a7df`. It is a J700 qualification branch;
the patches have not been rebased onto `aurora-wip`.

## Included work

- Correct the T8140 SMC SRAM address using the live firmware ADT.
- Provide diagnostic-only and telemetry-only device-tree variants.
- Bound sensor metadata enumeration and reject malformed diagnostic reads.
- Expose battery/AC power supplies with charge-policy writes blocked.
- Discover readable optional battery properties so missing keys do not abort
  the battery uevent. The same fix was separately published as
  [`4676c48ecf1b`](https://github.com/aurora-silicon/linux/commit/4676c48ecf1b1acaba7c442d33fe9aae0f037e77)
  on `fix/macsmc-optional-battery-properties`; this branch needs its local
  counterpart because it uses the older J700 base.
- Expose eight measured temperature keys through hwmon: `TB0T`, `TCMb`,
  `TCMz`, `TG0A`, `Te04`, `Te05`, `Te06`, and `Te0R`.
- Freeze the 30-second telemetry polling work before device suspend.
- Preserve the later opt-in input-event patch and notifier cleanup.
  Input events are disabled by default. When explicitly enabled, the write
  gate permits only one-byte `NTAP` values 0/1; other key writes and atomic
  writes remain blocked in telemetry mode.

The telemetry device tree is
`arch/arm64/boot/dts/apple/t8140-j700-smc-telemetry.dts`.
`t8140-j700-smc-readonly.dts` is the diagnostic-only variant.
`t8140-j700-smc.dts` enables normal SMC startup and does not itself request
telemetry-only behavior. The telemetry variant is the one covered by the
battery/temperature qualification below.

## Original patch provenance

| Local patch | Original commit |
| --- | --- |
| 0002 diagnostic battery/temperature reads | `29a584dd03f5f1775b8f4662b84447cf0fce142f` |
| 0004 live SRAM correction | `a6ea57277cbc9d34c7dab4831907a6d9afe0873b` |
| 0005 rejected diagnostic metadata | `8aa05430a7e48e111add794992b909b175cad342` |
| 0006 bounded metadata inventory | `a8a9dd3811e5206547537df221c7b28abc19da00` |
| 0007 telemetry-only power supplies | `92d4fbf1d294b459fc587dd1fb3a2ea8e5ec4e85` |
| 0008 optional battery properties | `940478b00f3f839093bf2bc5e078c139468812f9` |
| 0009 eight temperature keys | `25ac3e3699a415b5341ca9b447b54400ae92c9b5` |
| 0012 suspend-safe polling | `afa5d186bd82f6e507f9bcce63a5f3f8d5befce7` |
| 0014 opt-in input events | `5b19c4dae4424b5dfcfee33524b6b6f72daa1cb4` |

## Host regression tests

From the repository root, run:

```sh
python3 tools/testing/j700-smc/test_smc.py
python3 tools/testing/j700-smc/test_smc_readonly.py
python3 tools/testing/j700-smc/test_smc_inventory.py
python3 tools/testing/j700-smc/test_smc_optional_properties.py
```

These existing qualification tests extract actual driver functions, compile
them with AddressSanitizer and UndefinedBehaviorSanitizer, and exercise mocked
SMC responses. They require Python 3 and `/usr/bin/cc` with sanitizer support.
They cover malformed/missing metadata, short/failed reads, enumeration bounds
and timeout, write rejection, the notification-only exception, and retention
of normal write behavior. They do not access hardware.

Publication validation on September 13, 2026 passed all four host tests with
ASan/UBSan. The SMC core, battery, input, and hwmon translation units also
compiled for ARM64 using the existing Linux 7.1.6 configuration and generated
headers. All three SMC device-tree variants compiled with no dtc warnings.
This was targeted compilation, not a new full kernel build or hardware boot.

## Recorded hardware results and limits

The September 5 local qualification notes recorded 4.379 V battery voltage,
31.7 C battery temperature, current and cycle-count reads, 100% Full status,
AC online, and complete supply uevents. Eight hwmon temperature keys read
31.3–65.5 C. Battery and temperature readouts remained available after the
recorded timed device suspend/resume test.

These are historical observations from the local qualification tree, not a
new hardware run of this publication branch. Physical temperature-sensor
locations still need a measured map; the labels intentionally retain SMC key
names. These results do not establish normal SMC startup, charging-control,
or deep-sleep qualification.
