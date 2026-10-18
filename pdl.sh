#!/bin/bash
# Parallel range downloader: pdl.sh <url> <outfile> <total_bytes> [conns]
URL="$1"; OUT="$2"; TOTAL="$3"; N="${4:-16}"
PD="${OUT}.parts"
mkdir -p "$PD" "$(dirname "$OUT")"
CH=$(( (TOTAL + N - 1) / N ))
echo "total=$TOTAL bytes, $N conns, chunk=$CH"

for i in $(seq 0 $((N-1))); do
  A=$(( i * CH ))
  B=$(( A + CH - 1 ))
  [ $B -ge $TOTAL ] && B=$(( TOTAL - 1 ))
  PART="$PD/p$i"
  HAVE=0
  [ -f "$PART" ] && HAVE=$(stat -c%s "$PART")
  S=$(( A + HAVE ))
  [ $S -gt $B ] && continue
  (
    curl -sL --retry 50 --retry-delay 2 --retry-all-errors --speed-time 60 --speed-limit 1000 \
         -r ${S}-${B} "$URL" >> "$PART"
  ) &
done
wait

# verify
echo "--- verifying parts ---"
OK=1
for i in $(seq 0 $((N-1))); do
  A=$(( i * CH )); B=$(( A + CH - 1 )); [ $B -ge $TOTAL ] && B=$((TOTAL-1))
  WANT=$(( B - A + 1 ))
  PART="$PD/p$i"
  if [ ! -f "$PART" ]; then echo "p$i MISSING"; OK=0; continue; fi
  GOT=$(stat -c%s "$PART")
  if [ "$GOT" -ne "$WANT" ]; then echo "p$i SHORT want=$WANT got=$GOT"; OK=0; fi
done

if [ "$OK" = "1" ]; then
  cat "$PD"/p0 "$PD"/p1 "$PD"/p2 "$PD"/p3 "$PD"/p4 "$PD"/p5 "$PD"/p6 "$PD"/p7 \
      "$PD"/p8 "$PD"/p9 "$PD"/p10 "$PD"/p11 "$PD"/p12 "$PD"/p13 "$PD"/p14 "$PD"/p15 > "$OUT"
  echo "MERGED $(stat -c%s "$OUT") bytes (expect $TOTAL)"
else
  echo "INCOMPLETE - rerun to resume"
  exit 1
fi