#!/usr/bin/env python3
"""Extrae del escenario de NASSP "Apollo 11 - 10 - PDI T-15m" el estado del computador del LM.

NASSP (Project Apollo para Orbiter, GPLv2): https://github.com/orbiternassp/NASSP
Uso: python3 extraer_escenario.py "Apollo 11 - 10 - PDI T-15m.scn" > escenario_pdi.json

Guarda la memoria borrable completa (2048 palabras, octal en el escenario) como base64 de
uint16 little-endian, y los canales de entrada 030-033 (decimales en el escenario).
"""
import base64
import json
import re
import struct
import sys

scn = open(sys.argv[1], encoding="utf-8", errors="replace").read()
lm = scn[scn.index("Eagle:ProjectApollo/LEM"):]
agc = lm[lm.index("AGC_BEGIN"):lm.index("AGC_END")]

mem = [0] * 0o4000
for addr, val in re.findall(r"EMEM(\d+) (\d+)", agc):
    mem[int(addr, 8)] = int(val, 8)
chans = {int(c): int(v) for c, v in re.findall(r"VICHAN(\d+) (\d+)", agc)}

json.dump({
    "fuente": "NASSP, escenario 'Apollo 11 - 10 - PDI T-15m' (GPLv2)",
    "memoria": base64.b64encode(struct.pack("<2048H", *mem)).decode(),
    "canales": {oct(ch)[2:]: chans[ch] for ch in (0o30, 0o31, 0o32, 0o33)},
}, sys.stdout, ensure_ascii=False)
sys.stdout.write("\n")
