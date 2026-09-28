#!/bin/sh
# Fetches mbelib (ISC licence) for the optional DMR voice decoder.
# The AMBE+2 algorithm is covered by patents in some jurisdictions: check that
# using it is lawful where you are before enabling it.
set -e
cd "$(dirname "$0")/.."
DEST=components/mbelib/mbelib
REPO=https://github.com/lwvmobile/mbelib
COMMIT=34adf9f054bc5650ace162a4917dcbc2cfa6102e   # version tested with this firmware
if [ -d "$DEST/.git" ]; then
  echo "mbelib already present in $DEST"
else
  git clone "$REPO" "$DEST"
fi
git -C "$DEST" checkout -q "$COMMIT"
echo "mbelib ready. Now: idf.py menuconfig -> SDR: DMR -> DMR voice (mbelib, AMBE+2)"
