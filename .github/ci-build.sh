#!/bin/sh
# Build mwlwifi against a kernel.org release with W=1 and sparse, and
# fail on any compiler or sparse warning.
#
# usage: ci-build.sh <kernel version> <x86_64|arm> [work dir]
#
# x86_64 is built with gcc, arm (the routers this driver runs on) with
# LLVM=1, which also gives clang coverage.
set -eu

ver=$1
arch=$2
work=${3:-${TMPDIR:-/tmp}/mwlwifi-ci}
src=$(cd "$(dirname "$0")/.." && pwd)
ksrc=$work/linux-$ver
kout=$work/build-$ver-$arch
out=$work/mwlwifi-$ver-$arch

case $arch in
x86_64)	kmake="make ARCH=x86_64"; defconfig=x86_64_defconfig ;;
arm)	kmake="make ARCH=arm LLVM=1"; defconfig=multi_v7_defconfig ;;
*)	echo "unknown arch $arch" >&2; exit 2 ;;
esac

mkdir -p "$work"
if [ ! -d "$ksrc" ]; then
	curl -sSfL "https://cdn.kernel.org/pub/linux/kernel/v${ver%%.*}.x/linux-$ver.tar.xz" |
		tar -xJ -C "$work"
fi

if [ ! -f "$kout/.config" ]; then
	$kmake -C "$ksrc" O="$kout" $defconfig
	"$ksrc/scripts/config" --file "$kout/.config" \
		-e PCI -e WLAN -e WLAN_VENDOR_MARVELL -e CFG80211 -m MAC80211 \
		-e THERMAL -e HWMON -e DEBUG_FS -d WERROR
	$kmake -C "$ksrc" O="$kout" olddefconfig
	$kmake -C "$ksrc" O="$kout" -j"$(nproc)" modules_prepare
fi

# Build out of a copy, so that the build does not leave objects in the
# source tree.
rm -rf "$out"
mkdir -p "$out"
(cd "$src" && git ls-files -z | xargs -0 cp --parents -t "$out")

$kmake -C "$kout" M="$out" W=1 C=1 KBUILD_MODPOST_WARN=1 \
	-j"$(nproc)" modules 2>&1 | tee "$out/build.log"

# Undefined symbols are expected: modules_prepare does not build the
# kernel's Module.symvers.
if grep -E 'warning:|error:' "$out/build.log" | grep -v 'undefined!$'; then
	echo "build produced warnings" >&2
	exit 1
fi
test -f "$out/mwlwifi.ko"
