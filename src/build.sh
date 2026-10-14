#!/usr/bin/env bash
# Recompila el motor AGC a WebAssembly y regenera index.html.
# Requisitos: git, make, un compilador C nativo (gcc o clang, para yaYUL) y python3 con pip
# (instala zig por pip para compilar el motor a wasm32-wasi).
set -euo pipefail
cd "$(dirname "$0")"
WORK=$(mktemp -d)
COMMIT=$(cat VIRTUALAGC_COMMIT)

echo "→ Clonando Virtual AGC ($COMMIT)"
git clone -q https://github.com/virtualagc/virtualagc.git "$WORK/virtualagc"
git -C "$WORK/virtualagc" checkout -q "$COMMIT"
Y="$WORK/virtualagc/yaAGC"

echo "→ Compilando ensamblador yaYUL y ensamblando las cuerdas de vuelo"
make -s -C "$WORK/virtualagc/yaYUL" cc=gcc NVER='\"local\"'
for P in Luminary099 Comanche055; do
  (cd "$WORK/virtualagc/$P" && ../yaYUL/yaYUL --force MAIN.agc > /dev/null)
  cp "$WORK/virtualagc/$P/MAIN.agc.bin" "$WORK/$P.bin"
done

echo "→ Compilando el motor AGC a WebAssembly"
python3 -m pip install -q --user ziglang 2>/dev/null || python3 -m pip install -q --break-system-packages ziglang
python3 -m ziglang cc -target wasm32-wasi -O2 -s -DWASI=yes -DNVER='"web"' -I"$Y" -Wno-everything \
  agc_web.c "$Y/agc_engine.c" "$Y/agc_engine_init.c" \
  -Wl,--no-entry -mexec-model=reactor -o "$WORK/agc.wasm"

echo "→ Generando ../index.html"
python3 - "$WORK" <<'EOF'
import base64, sys
w = sys.argv[1]
b = lambda f: base64.b64encode(open(f"{w}/{f}", "rb").read()).decode()
t = open("dsky_template.html").read()
t = t.replace("%%SIM%%", open("lm_sim.js").read().strip())
t = t.replace("%%ESCENARIO%%", open("escenario_pdi.json").read().strip())
t = t.replace("%%WASM%%", b("agc.wasm")).replace("%%LM%%", b("Luminary099.bin")).replace("%%CM%%", b("Comanche055.bin"))
open("../index.html", "w").write(t)
EOF
rm -rf "$WORK"
echo "Listo."
