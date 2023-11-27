#!/bin/bash

set -e

MERGE_FRAGMENT=merge.config
if [ ! -f arch/arm64/configs/$MERGE_FRAGMENT ]; then
MERGE_FRAGMENT=
fi

for def in arch/arm64/configs/exynos7885-*; do
	make O=out LLVM=1 $(basename $def) $MERGE_FRAGMENT;
	cp out/.config $(dirname $def)/full/$(basename $def);
	make O=out LLVM=1 savedefconfig;
	cp out/defconfig $def;
done
