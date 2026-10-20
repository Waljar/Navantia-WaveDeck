#!/bin/sh
# Builds and runs the PC tests of the platform code (no hardware needed):
#   sh sim/build_tests.sh        (from the repository root, needs gcc)
set -e
cd "$(dirname "$0")"
CORE=../firmware/Core
OUT="${TMPDIR:-/tmp}"
CFLAGS="-O2 -g -Wall -Wextra -Wno-unused-parameter -I$CORE/Inc -I."

echo "== syntax check of the STM32-only files against stub HAL headers"
for f in plat_hw_stm32.c main.c; do
  gcc -fsyntax-only -Wall -Wextra -Wno-unused-parameter -Istubs -I$CORE/Inc $CORE/Src/$f
done

echo "== core maths and step engine"
gcc $CFLAGS -o "$OUT/sea_test_core" test_core.c hw_host.c $CORE/Src/plat_kin.c $CORE/Src/plat_waves.c $CORE/Src/plat_engine.c -lm
"$OUT/sea_test_core"

echo "== calibration storage"
gcc $CFLAGS -o "$OUT/sea_test_cal" test_cal.c hw_host.c $CORE/Src/plat_cal.c -lm
"$OUT/sea_test_cal"

echo "== whole firmware, scripted menu sessions (about 30 s)"
gcc $CFLAGS -o "$OUT/sea_test_menu" test_menu.c hw_host.c $CORE/Src/plat_kin.c $CORE/Src/plat_waves.c $CORE/Src/plat_engine.c \
    $CORE/Src/plat_cal.c $CORE/Src/plat_homing.c $CORE/Src/app_common.c $CORE/Src/mode_*.c -lm
"$OUT/sea_test_menu"

if command -v node >/dev/null 2>&1; then
  echo "== JavaScript port (visualiser) against the C code"
  gcc -O2 -I$CORE/Inc -o "$OUT/sea_dump_ref" dump_ref.c $CORE/Src/plat_kin.c $CORE/Src/plat_waves.c -lm
  "$OUT/sea_dump_ref" > "$OUT/sea_ref.txt"
  node ../tools/verify-js.js "$OUT/sea_ref.txt"
else
  echo "(node not found: skipping the JavaScript check)"
fi
