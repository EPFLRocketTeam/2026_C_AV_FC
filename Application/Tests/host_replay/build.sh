#!/bin/sh
# Build the host replay harness: the firmware Kalman runtime (kalman_process.cpp,
# Kalman library, data stores) compiled for the host against shadow HAL headers.
# Usage: build.sh [src_tree] [out_dir]   (defaults: this checkout, ./build)
# src_tree can be another export of the repo (e.g. `git archive` of an older
# commit plus its submodules) to compare firmware versions on the same data.
# Set EXTRA=-UHARNESS_HAS_RESET for trees without kalman_request_reset().
H=$(cd "$(dirname "$0")" && pwd)
S=${1:-$(cd "$H/../../.." && pwd)}; O=${2:-$H/build}; mkdir -p "$O"
FLAGS="-std=gnu++20 -O2 -g -DUNIT_TEST_ENV -DKALMAN_DEBUG_PRINT=0 -DKALMAN_DEBUG_FORCE_FLIGHT=0 -DCOMPILE_OPT_LEVEL=2 \
 -DHARNESS_HAS_RESET -I$H/inc -I$S -I$S/Application/Config/2026_C_AV_FLIGHT_PARAMS \
 -I$S/Application/Config/2026_C_AV_FLIGHT_PARAMS/2026_C_AV_CONFMAN -I$S/Drivers/PRC_CAN/2026_C_AV_FC_PRC_INTRANET/include -w $EXTRA"
# data.cpp also implements the app timebase on HAL_GetTick (ms resolution in
# UNIT_TEST_ENV); rename it so the harness's simulated microsecond clock is used.
DATAFLAGS="-Dapp_timebase_init=dc_tb_init -Dapp_timebase_now_us=dc_tb_now_us -Dapp_timebase_now_ms=dc_tb_now_ms -Dapp_timebase_print_init_diag=dc_tb_diag"
SRCS="$(find $S/Application/Kalman -name '*.cpp') $S/Application/Data/data.cpp $(ls $S/Application/Data/Stores/*.cpp) $S/Application/Data/gps_store.cpp $H/stubs.cpp $H/run.cpp"
OBJS=""; FAIL=0
for f in $SRCS; do
  o=$O/$(echo "$f $EXTRA" | md5sum | cut -c1-12).o; OBJS="$OBJS $o"
  if [ ! -f $o ] || [ $f -nt $o ]; then
    XF=""; case $f in */Application/Data/data.cpp) XF=$DATAFLAGS;; esac
    g++ $FLAGS $XF -c $f -o $o 2> $o.err || { FAIL=1; echo "== $f"; head -15 $o.err; }
  fi
done
[ $FAIL = 0 ] && g++ $OBJS -o $O/harness -lpthread && echo "built $O/harness"
