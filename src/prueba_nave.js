// Prueba de la nave simulada: corre Luminary099 (desde ../index.html) con lm_sim.js, sin navegador.
//   node src/prueba_nave.js
// Comprueba que el AGC lee la plataforma, que su autopiloto frena rotaciones en los tres ejes,
// que la maniobra automática V49 llega a destino y que la palanca manda en ATT HOLD.
"use strict";
const fs = require("fs");
const path = require("path");
const { LMSim } = require("./lm_sim.js");

const AGC_HZ = 1024000 / 12;
const SLICE = Math.round(AGC_HZ / 100);
const DEG = Math.PI / 180;
const KEY = { V: 17, N: 31, E: 28, R: 18, "+": 26, "-": 27, "0": 16 };
for (let d = 1; d <= 9; d++) KEY[String(d)] = d;
const DIG = { 0: " ", 21: "0", 3: "1", 25: "2", 27: "3", 15: "4", 30: "5", 28: "6", 19: "7", 29: "8", 31: "9" };

const html = fs.readFileSync(path.join(__dirname, "..", "index.html"), "utf8");
const blob = (k) => { const i = html.indexOf(k) + k.length; return Buffer.from(html.slice(i, html.indexOf('"', i)), "base64"); };
const WASM = blob('WASM_B64 = "'), ROPE = blob('LM: "');

async function lm() {
  const wasi = new Proxy({}, { get: () => () => 8 });
  const { instance } = await WebAssembly.instantiate(WASM, { wasi_snapshot_preview1: wasi });
  const agc = instance.exports;
  if (agc._initialize) agc._initialize();
  new Uint8Array(agc.memory.buffer, agc.rom_ptr(), ROPE.length).set(ROPE);
  if (agc.agc_boot(ROPE.length)) throw new Error("no arranca la cuerda");
  const sim = LMSim(agc);
  sim.powerUp();
  const run = (sec) => { for (let c = 0; c < sec * AGC_HZ; c += SLICE) { agc.agc_run(SLICE); sim.step(SLICE / AGC_HZ); } };
  const keys = (s) => {
    for (const c of s) {
      if (c === "P") { agc.agc_pro(1); run(0.5); agc.agc_pro(0); run(0.6); } else { agc.agc_key(KEY[c]); run(0.25); }
    }
  };
  const dsky = () => {
    const d = new Int32Array(agc.memory.buffer, agc.disp_ptr(), 32);
    const dg = (v, hi) => DIG[hi ? (v >> 5) & 31 : v & 31] ?? "?";
    return { prog: (d[12] & 0o400) !== 0, r1: dg(d[8], 0) + dg(d[7], 1) + dg(d[7], 0) + dg(d[6], 1) + dg(d[6], 0) };
  };
  run(0.8); keys("R"); run(0.4);
  keys("V48EV21E20102EPV21E+33300EPV34E");   // autopiloto: configuración y masa del LM
  run(2); keys("V37E00E"); run(1);
  return { agc, sim, run, keys, dsky };
}

const ESCENARIO = JSON.parse(html.slice(html.indexOf("const ESCENARIO = ") + 18, html.indexOf(";\n", html.indexOf("const ESCENARIO = "))));

async function descent(rrNoise = 0) {
  const wasi = new Proxy({}, { get: () => () => 8 });
  const { instance } = await WebAssembly.instantiate(WASM, { wasi_snapshot_preview1: wasi });
  const agc = instance.exports;
  if (agc._initialize) agc._initialize();
  new Uint8Array(agc.memory.buffer, agc.rom_ptr(), ROPE.length).set(ROPE);
  agc.agc_boot(ROPE.length);
  const buf = Buffer.from(ESCENARIO.memoria, "base64");
  for (let a = 0o10; a < 0o4000; a++) agc.agc_set_erasable(a, buf.readUInt16LE(2 * a));
  for (const ch in ESCENARIO.canales) agc.agc_set_chan(parseInt(ch, 8), ESCENARIO.canales[ch]);
  const sim = LMSim(agc);
  sim.reset();
  sim.initFromMemory();
  sim.setMode("auto");
  agc.agc_rr_noise(rrNoise);
  let t = 0;
  const run = (sec) => { for (let c = 0; c < sec * AGC_HZ; c += SLICE) { agc.agc_run(SLICE); sim.step(SLICE / AGC_HZ); t += SLICE / AGC_HZ; } };
  const keys = (s) => { for (const c of s) { if (c === "P") { agc.agc_pro(1); run(0.5); agc.agc_pro(0); run(0.6); } else { agc.agc_key(KEY[c]); run(0.25); } } };
  const d = () => new Int32Array(agc.memory.buffer, agc.disp_ptr(), 32);
  const two = (v) => (DIG[(v >> 5) & 31] ?? "?") + (DIG[v & 31] ?? "?");
  const vn = () => ({ prog: two(d()[11]), verb: two(d()[10]), noun: two(d()[9]), flash: (d()[16] & 0o40) !== 0 });
  const dsky = () => { const x = d(), g = (v, hi) => DIG[hi ? (v >> 5) & 31 : v & 31] ?? "?"; return { r1: g(x[8], 0) + g(x[7], 1) + g(x[7], 0) + g(x[6], 1) + g(x[6], 0) }; };
  run(2); keys("E"); run(3); keys("E"); run(2);
  while (vn().verb !== "99" && t < 1200) run(1);
  keys("P");
  const t0 = t;
  let yawed = false, v57 = false;
  while (!sim.landed && t - t0 < 1000) {
    run(1);
    const s = vn();
    if (!yawed && t - t0 > 240) {
      yawed = true; sim.setMode("hold"); sim.setStick(0, 42, 0);
      let ang = 0; while (ang < Math.PI * 0.93) { run(0.1); ang += sim.rates[0] * 0.1; }
      sim.setStick(0, 0, 0); run(4); sim.setMode("auto");
    }
    if (!v57 && sim.radar.good) { keys("V57E"); v57 = true; }
    if (s.prog === "64" && s.verb === "06" && s.noun === "64" && s.flash) keys("P");
  }
  return { sim, keys, dsky };
}

let fallas = 0;
function check(nombre, ok, detalle) {
  console.log(`${ok ? "ok  " : "FALLA"} ${nombre}${detalle ? " — " + detalle : ""}`);
  if (!ok) fallas++;
}
const deg = (v) => v.map((x) => (x / DEG).toFixed(2)).join(" / ");

(async () => {
  {
    const h = await lm();
    check("arranque sin alarmas", !h.dsky().prog);
    h.keys("V16N20E");
    h.sim.setMode("off");
    h.sim.setRates([2 * DEG, 0, 0]); h.run(10); h.sim.setRates([0, 0, 0]); h.run(1);
    const o = h.sim.gimbals[0] / DEG, r1 = Number(h.dsky().r1) / 100;
    check("V16 N20 lee el gimbal externo", Math.abs(o - r1) < 0.05, `simulado ${o.toFixed(2)}°, DSKY ${r1.toFixed(2)}°`);
  }
  for (let k = 0; k < 3; k++) {
    for (const s of [1, -1]) {
      const h = await lm();
      h.sim.setMode("auto"); h.run(3);
      const w = [0, 0, 0]; w[k] = s * DEG; h.sim.setRates(w); h.run(20);
      const rest = Math.abs(h.sim.rates[k]) / DEG;
      check(`el DAP frena ${s > 0 ? "+" : "-"}1 °/s en ${"XYZ"[k]}`, rest < 0.1, `queda ${rest.toFixed(3)} °/s`);
    }
  }
  {
    const h = await lm();
    h.sim.setMode("auto"); h.run(2);
    h.keys("V49EV25E+03000E+02000E+01000EPP"); h.run(60);
    const g = h.sim.gimbals.map((x) => x / DEG), err = Math.max(Math.abs(g[0] - 30), Math.abs(g[1] - 20), Math.abs(g[2] - 10));
    check("V49 lleva el LM a 30° / 20° / 10°", err < 1.5, `llegó a ${deg(h.sim.gimbals)}`);
  }
  {
    // Lo que hace la macro del sitio para repetir V49: V34 si el AGC espera respuesta (V50 N18
    // parpadeando) y, con una maniobra en curso, ATT HOLD primero para que el AGC la aborte.
    const h = await lm();
    h.sim.setMode("auto"); h.run(2);
    h.keys("V49EV25E+03000E+02000E+01000EPP"); h.run(40);
    h.keys("V34E"); h.run(1.5);
    h.keys("V49EV25E+00000E+00000E+00000EPP"); h.run(5);
    h.sim.setMode("hold"); h.run(3);
    h.keys("V34E"); h.run(1.5);
    h.sim.setMode("auto");
    h.keys("V49EV25E+03000E+02000E+01000EPP"); h.run(40);
    const g = h.sim.gimbals.map((x) => x / DEG), err = Math.max(Math.abs(g[0] - 30), Math.abs(g[1] - 20), Math.abs(g[2] - 10));
    check("V49 repetido e interrumpido con ATT HOLD", err < 1.5 && !h.dsky().prog, `llegó a ${deg(h.sim.gimbals)}`);
  }
  {
    const h = await lm();
    h.sim.setMode("hold"); h.run(2);
    h.sim.setStick(21, 0, 0); h.run(3);
    const rate = h.sim.rates[1] / DEG;
    h.sim.setStick(0, 0, 0); h.run(8);
    const after = Math.abs(h.sim.rates[1]) / DEG;
    check("la palanca gira el LM en cabeceo", rate > 3, `${rate.toFixed(2)} °/s`);
    check("al soltarla el AGC lo detiene", after < 0.1, `queda ${after.toFixed(3)} °/s`);
  }
  {
    // Descenso del Apollo 11 desde PDI −15 min, con la tripulación haciendo lo de la misión:
    // ENTR, ENTR, PRO en V99, giro a ventanas arriba, V57 con datos del radar, PRO en P64.
    const h = await descent();
    check("descenso: termina posado", h.sim.landed, h.sim.landed ? "" : `h = ${h.sim.surface.h.toFixed(0)} m`);
    if (h.sim.landed) {
      const td = h.sim.touchdown;
      check("descenso: contacto suave", Math.abs(td.vVert) < 1.5 && td.vHoriz < 1.5, `vertical ${td.vVert.toFixed(2)} m/s, horizontal ${td.vHoriz.toFixed(2)} m/s`);
      check("descenso: queda combustible", h.sim.fuel > 100, `${h.sim.fuel.toFixed(0)} kg`);
    }
    h.keys("V05N09E");
    check("descenso: sin alarmas", h.dsky().r1 === "00000", `R1 de V05 N09 = ${h.dsky().r1}`);
  }
  {
    // Con el radar de encuentro robando ciclos como en el Apollo 11: alarmas 1201/1202 por
    // sobrecarga del ejecutivo, reinicios por software y aun así un aterrizaje suave.
    const h = await descent(6400);
    const td = h.sim.touchdown;
    check("1202: aun así se posa suave", h.sim.landed && !td.crash, td ? `vertical ${td.vVert.toFixed(2)} m/s` : "no se posó");
    h.keys("V05N09E");
    const codes = [h.dsky().r1];
    check("1202: el AGC registró 1201 o 1202", codes.some((c) => c === "01201" || c === "01202"), `R1 de V05 N09 = ${codes[0]}`);
  }
  console.log(fallas ? `\n${fallas} prueba(s) fallaron` : "\ntodo bien");
  process.exit(fallas ? 1 : 0);
})();
