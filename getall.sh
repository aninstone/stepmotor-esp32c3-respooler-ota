#!/bin/bash
# Batch download + install PlatformIO packages via 16-conn range requests
set -u
PDL="D:/WorkPRJ/VscodePRJ/stepmotor/pdl.sh"
DLDIR="D:/WorkPRJ/VscodePRJ/stepmotor/dl"
PKGDIR="$USERPROFILE/.platformio/packages"
BACKUP="$USERPROFILE/.platformio/platforms_backup"
mkdir -p "$DLDIR" "$PKGDIR" "$BACKUP"

install_pkg() {
  local NAME="$1" OWNER="$2" VER="$3" URL="$4"
  local DST="$PKGDIR/$NAME"
  if [ -f "$DST/.piopm" ]; then echo "[$NAME] already installed, skip"; return 0; fi

  echo "[$NAME] fetching size..."
  local SIZE
  SIZE=$(curl -sIL --max-time 40 "$URL" 2>/dev/null | grep -i "^content-length" | tail -1 | tr -d '\r' | awk '{print $2}')
  if [ -z "$SIZE" ] || [ "$SIZE" -lt 100 ] 2>/dev/null; then echo "[$NAME] FAIL size"; return 1; fi
  echo "[$NAME] $VER  $((SIZE/1048576)) MB"

  if ! bash "$PDL" "$URL" "$DLDIR/$NAME.tgz" "$SIZE" 16 2>&1 | tail -2; then
    echo "[$NAME] incomplete, retry"
    bash "$PDL" "$URL" "$DLDIR/$NAME.tgz" "$SIZE" 16 2>&1 | tail -2 || { echo "[$NAME] FAIL dl"; return 1; }
  fi
  local GOT; GOT=$(stat -c%s "$DLDIR/$NAME.tgz")
  [ "$GOT" != "$SIZE" ] && { echo "[$NAME] size mismatch $GOT vs $SIZE"; return 1; }

  if [ -e "$DST" ]; then mv "$DST" "$BACKUP/${NAME}_old_20260713" 2>/dev/null; fi
  mkdir -p "$DST"
  ( cd "$DLDIR" && tar --force-local -xzf "$NAME.tgz" -C "$DST" ) || { echo "[$NAME] extract fail"; return 1; }

  printf '{"type": "tool", "name": "%s", "version": "%s", "spec": {"owner": "%s", "id": null, "name": "%s", "requirements": null, "uri": null}}' \
    "$NAME" "$VER" "$OWNER" "$NAME" > "$DST/.piopm"
  echo "[$NAME] INSTALLED -> $DST"
}

install_pkg tool-esptoolpy platformio 1.40501.0 \
 "https://dl.registry.platformio.org/download/platformio/tool/tool-esptoolpy/1.40501.0/tool-esptoolpy-1.40501.0.tar.gz"

install_pkg tool-mklittlefs platformio 1.203.210628 \
 "https://dl.registry.platformio.org/download/platformio/tool/tool-mklittlefs/1.203.210628/tool-mklittlefs-windows_amd64-1.203.210628.tar.gz"

install_pkg tool-mkfatfs platformio 2.0.1 \
 "https://dl.registry.platformio.org/download/platformio/tool/tool-mkfatfs/2.0.1/tool-mkfatfs-windows-2.0.1.tar.gz"

install_pkg tool-mkspiffs platformio 2.230.0 \
 "https://dl.registry.platformio.org/download/platformio/tool/tool-mkspiffs/2.230.0/tool-mkspiffs-windows-2.230.0.tar.gz"

install_pkg tool-riscv32-esp-elf-gdb espressif 11.2.0+20220823 \
 "https://dl.registry.platformio.org/download/espressif/tool/tool-riscv32-esp-elf-gdb/11.2.0+20220823/tool-riscv32-esp-elf-gdb-windows_amd64-11.2.0+20220823.tar.gz"

install_pkg framework-arduinoespressif32 platformio 3.20014.231204 \
 "https://dl.registry.platformio.org/download/platformio/tool/framework-arduinoespressif32/3.20014.231204/framework-arduinoespressif32-3.20014.231204.tar.gz"

echo "=== ALL DONE ==="