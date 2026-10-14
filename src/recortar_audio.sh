#!/usr/bin/env bash
# Regenera audio/*.mp3: clips de las comunicaciones del alunizaje del Apollo 11.
# Fuente: grabaciones del canal de Asuntos Públicos de NASA (Johnson Space Center, Audio Control
# Room, cortesía de John Stoll) publicadas en el Apollo Lunar Surface Journal. Dominio público
# (obras del Gobierno de EE. UU., 17 U.S.C. §105).
#   A: https://apollojournals.org/alsj/a11/a11a1021133-3114.mp3  (desde GET 102:11:33)
#   B: https://apollojournals.org/alsj/a11/a11a1023114-4746.mp3  (desde GET 102:31:14)
# Cada clip es un tramo continuo; mono, 64 kbps, normalizado a -16 LUFS con loudnorm en dos pasadas.
# Requisitos: curl, ffmpeg, python3.
set -euo pipefail
cd "$(dirname "$0")/.."
WORK=$(mktemp -d)
curl -sfL -o "$WORK/A.mp3" https://apollojournals.org/alsj/a11/a11a1021133-3114.mp3
curl -sfL -o "$WORK/B.mp3" https://apollojournals.org/alsj/a11/a11a1023114-4746.mp3
mkdir -p audio

clip() {   # clip fuente inicio fin salida
  local src="$WORK/$1.mp3" ss=$2 to=$3 out=audio/$4.mp3
  local fo; fo=$(python3 -c "print(round($to-$ss-0.2,3))")
  local pre="highpass=f=80,afade=t=in:st=0:d=0.1,afade=t=out:st=$fo:d=0.2"
  local j; j=$(ffmpeg -nostdin -hide_banner -ss "$ss" -to "$to" -i "$src" -ac 1 -af "$pre,loudnorm=I=-16:TP=-1.5:LRA=11:print_format=json" -f null - 2>&1 | sed -n '/^{/,/^}/p')
  g() { echo "$j" | python3 -c "import json,sys;print(json.load(sys.stdin)['$1'])"; }
  local ln="loudnorm=I=-16:TP=-1.5:LRA=11:measured_I=$(g input_i):measured_TP=$(g input_tp):measured_LRA=$(g input_lra):measured_thresh=$(g input_thresh):offset=$(g target_offset):linear=true"
  ffmpeg -nostdin -hide_banner -loglevel error -y -ss "$ss" -to "$to" -i "$src" -ac 1 -af "$pre,$ln" -ar 44100 -c:a libmp3lame -b:a 64k -map_metadata -1 "$out"
  echo "→ $out"
}

clip A 994.6 1007.5 pdi            # "Eagle, Houston. If you read, you're go for powered descent" (102:28:08)
clip B 427.2 438.6  1202           # "Program alarm." ... "It's a 1202." (102:38:26)
clip B 456.9 460.65 go-alarma      # "We're go on that alarm." (102:38:53)
clip B 659.0 673.85 1201           # "1201" ... "We're go. Same type. We're go." (102:42:19)
clip B 862.7 868.2  contact        # "Contact light." ... "Okay, engine stop." (102:45:40)
clip B 885.3 896.6  eagle-landed   # "Tranquility Base here. The Eagle has landed." (102:45:58)
rm -rf "$WORK"
