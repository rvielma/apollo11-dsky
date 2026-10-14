// lm_sim.js — la nave que rodea al AGC: actitud del LM, plataforma inercial (IMU) y chorros RCS.
//
// Ejes del LM: X hacia arriba (guiñada), Y a la derecha (cabeceo), Z hacia adelante (alabeo).
// La plataforma (stable member, SM) mantiene su orientación en el espacio salvo cuando el AGC
// la gira: torque fino de giróscopos (canal ficticio 0177) o arrastre grueso de los CDU (0174-0176).
// Los ángulos de gimbal se leen como la orientación del cuerpo respecto de la plataforma, en el orden
// del IMU del Apollo: interno (Y, IGA), medio (Z, MGA), externo (X, OGA). Cada cambio de ángulo
// llega al AGC como pulsos en los contadores CDUX/CDUY/CDUZ (032-034), 2^-15 revoluciones cada uno.
//
// Modelo simplificado a propósito: sin traslación (los PIPA quedan en cero, como en vuelo libre),
// inercias constantes, empuje de los chorros constante mientras están encendidos.

// eslint-disable-next-line no-unused-vars
function LMSim(agc) {
  "use strict";
  const AGC_HZ = 1024000 / 12;
  const CDU_UNIT = (2 * Math.PI) / 32768;        // un pulso de CDU
  const GYRO_UNIT = (0.617981 / 3600) * Math.PI / 180; // un pulso de torque fino
  const COARSE_UNIT = (2 * Math.PI) / 8192;      // un pulso de arrastre grueso
  const GIMBAL_LOCK = 85 * Math.PI / 180;        // el gimbal medio se traba cerca de 90°
  // Inercias del LM con las dos etapas cargadas, aproximadas (kg·m²).
  const INERTIA = [34000, 45000, 42000];
  const JET_N = 445;                             // empuje de un chorro RCS (N)
  const ARM = 1.68;                              // brazo de palanca efectivo (m)
  // Chorros por bit (1..8). Canal 05: arriba/abajo de cada cuadrante → cabeceo (Y) y alabeo (Z).
  // Canal 06: delante/atrás/izquierda/derecha → guiñada (X). Signos: [X, Y, Z].
  const U = { Q1: [0, -1, -1], Q2: [0, 1, -1], Q3: [0, 1, 1], Q4: [0, -1, 1] };
  const neg = (v) => v.map((x) => -x);
  const JETS5 = [["Q4U", U.Q4], ["Q4D", neg(U.Q4)], ["Q3U", U.Q3], ["Q3D", neg(U.Q3)],
                 ["Q2U", U.Q2], ["Q2D", neg(U.Q2)], ["Q1U", U.Q1], ["Q1D", neg(U.Q1)]];
  const JETS6 = [["Q3A", [1, 0, 0]], ["Q4F", [-1, 0, 0]], ["Q1F", [1, 0, 0]], ["Q2A", [-1, 0, 0]],
                 ["Q2L", [1, 0, 0]], ["Q3R", [-1, 0, 0]], ["Q4R", [1, 0, 0]], ["Q1L", [-1, 0, 0]]];
  const JETS = JETS5.concat(JETS6);

  // ── Álgebra mínima: matrices 3×3 como arreglos de filas ──
  const mul = (A, B) => A.map((r) => [0, 1, 2].map((j) => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
  const tr = (A) => [0, 1, 2].map((i) => [A[0][i], A[1][i], A[2][i]]);
  const rx = (a) => { const c = Math.cos(a), s = Math.sin(a); return [[1, 0, 0], [0, c, -s], [0, s, c]]; };
  const ry = (a) => { const c = Math.cos(a), s = Math.sin(a); return [[c, 0, s], [0, 1, 0], [-s, 0, c]]; };
  const rz = (a) => { const c = Math.cos(a), s = Math.sin(a); return [[c, -s, 0], [s, c, 0], [0, 0, 1]]; };
  const I3 = () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  // Rotación por un vector pequeño (fórmula de Rodrigues), para integrar giros.
  function rotv(w) {
    const t = Math.hypot(w[0], w[1], w[2]);
    if (t < 1e-15) return I3();
    const k = w.map((x) => x / t), c = Math.cos(t), s = Math.sin(t), v = 1 - c;
    return [
      [c + k[0] * k[0] * v, k[0] * k[1] * v - k[2] * s, k[0] * k[2] * v + k[1] * s],
      [k[1] * k[0] * v + k[2] * s, c + k[1] * k[1] * v, k[1] * k[2] * v - k[0] * s],
      [k[2] * k[0] * v - k[1] * s, k[2] * k[1] * v + k[0] * s, c + k[2] * k[2] * v],
    ];
  }
  // Re-ortonormaliza para que el error numérico no se acumule.
  function ortho(A) {
    const n = (v) => { const l = Math.hypot(v[0], v[1], v[2]); return v.map((x) => x / l); };
    const c0 = n([A[0][0], A[1][0], A[2][0]]);
    let c1 = [A[0][1], A[1][1], A[2][1]];
    const d = c0[0] * c1[0] + c0[1] * c1[1] + c0[2] * c1[2];
    c1 = n(c1.map((x, i) => x - d * c0[i]));
    const c2 = [c0[1] * c1[2] - c0[2] * c1[1], c0[2] * c1[0] - c0[0] * c1[2], c0[0] * c1[1] - c0[1] * c1[0]];
    return [0, 1, 2].map((i) => [c0[i], c1[i], c2[i]]);
  }
  // Cuerpo en coordenadas de la plataforma: M = Ry(IGA)·Rz(MGA)·Rx(OGA).
  const gimbalsToM = (o, i, m) => mul(ry(i), mul(rz(m), rx(o)));
  function mToGimbals(M) {
    const m = Math.asin(Math.max(-1, Math.min(1, M[1][0])));
    const o = Math.atan2(-M[1][2], M[1][1]);
    const i = Math.atan2(-M[2][0], M[0][0]);
    return [o, i, m];
  }
  const wrap = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));

  let B, S, w, fed, needles, jetsOn, locked;
  function reset() {
    B = I3();          // ejes del cuerpo en el espacio inercial (columnas)
    S = I3();          // ejes de la plataforma en el espacio inercial
    w = [0, 0, 0];     // velocidad angular en ejes del cuerpo (rad/s)
    fed = [0, 0, 0];   // pulsos de CDU ya entregados al AGC por eje
    needles = [0, 0, 0];
    jetsOn = new Array(16).fill(0);
    locked = false;
  }
  reset();

  let io = null;
  const ioView = () => {
    if (!io || io.buffer !== agc.memory.buffer) io = new Int32Array(agc.memory.buffer, agc.io_ptr(), 32);
    return io;
  };

  // IMU operando, AGC al mando del LM. Los canales 30-33 son activos en bajo.
  function powerUp() {
    agc.agc_set_chan(0o30, agc.agc_chan(0o30) & ~(0o400 | 0o1000));
  }

  // Selector de modo del PGNS (canal 31: bit 13 = ATT HOLD, bit 14 = AUTO; ninguno = OFF).
  function setMode(mode) {
    let v = agc.agc_chan(0o31) | 0o30000;
    if (mode === "auto") v &= ~0o20000;
    if (mode === "hold") v &= ~0o10000;
    agc.agc_set_chan(0o31, v);
  }
  // Palanca de mando (RHC): cuentas en cabeceo, guiñada y alabeo, ±42 a fondo.
  // Fuera del centro, el bit 15 del canal 31 avisa al AGC (activo en bajo).
  function setStick(p, y, r) {
    const d = ioView(), c = (x) => Math.max(-42, Math.min(42, Math.round(x)));
    d[24] = c(p); d[25] = c(y); d[26] = c(r);
    const out = d[24] || d[25] || d[26];
    const v = agc.agc_chan(0o31);
    agc.agc_set_chan(0o31, out ? v & ~0o40000 : v | 0o40000);
  }

  const gimbals = () => mToGimbals(mul(tr(S), B));

  function setGimbals(g) {           // mueve la plataforma (el cuerpo queda quieto)
    S = ortho(mul(B, tr(gimbalsToM(g[0], g[1], g[2]))));
  }

  // Tras cargar un volcado de memoria: los contadores CDU del AGC ya traen los ángulos de gimbal.
  // La plataforma se orienta para que coincidan y se dan por entregados, sin pulsos de más.
  function syncFromCdu() {
    // Los CDU cuentan en módulo 2^15: 0..32767 recorre la vuelta completa.
    const counts = [0o32, 0o33, 0o34].map((a) => agc.agc_erasable(a) & 0o77777);
    setGimbals(counts.map((c) => c * CDU_UNIT));
    fed = counts.slice();
  }

  // Avanza dt segundos de nave. Lee lo que el AGC ordenó desde el último paso.
  function step(dt) {
    const d = ioView();
    const ch12 = agc.agc_chan(0o12);
    const coarse = (ch12 & 0o10) !== 0, zeroCdu = (ch12 & 0o20) !== 0, errCnt = (ch12 & 0o40) !== 0;

    // Torque de los chorros, ponderado por la fracción del paso que estuvo encendido cada uno.
    const tau = [0, 0, 0], cycles = dt * AGC_HZ;
    for (let j = 0; j < 16; j++) {
      const c = d[8 + j];
      if (!c) { jetsOn[j] = 0; continue; }
      d[8 + j] = 0;
      const f = Math.min(1, c / cycles);
      jetsOn[j] = f;
      const t = JETS[j][1];
      for (let k = 0; k < 3; k++) tau[k] += t[k] * JET_N * ARM * f;
    }
    // Euler: I·dω/dt = τ − ω × (I·ω)
    const Iw = w.map((x, k) => x * INERTIA[k]);
    const gyro = [w[1] * Iw[2] - w[2] * Iw[1], w[2] * Iw[0] - w[0] * Iw[2], w[0] * Iw[1] - w[1] * Iw[0]];
    for (let k = 0; k < 3; k++) w[k] += ((tau[k] - gyro[k]) / INERTIA[k]) * dt;
    B = ortho(mul(B, rotv(w.map((x) => x * dt))));

    // Torque fino de giróscopos: gira la plataforma sobre sus propios ejes.
    if (d[0] || d[1] || d[2]) {
      S = ortho(mul(S, rotv([d[0] * GYRO_UNIT, d[1] * GYRO_UNIT, d[2] * GYRO_UNIT])));
      d[0] = d[1] = d[2] = 0;
    }
    // Arrastre de CDU: con alineación gruesa mueve los gimbals; si no, las agujas de error del FDAI.
    if (d[3] || d[4] || d[5]) {
      if (coarse) {
        const g = gimbals();
        setGimbals([g[0] + d[3] * COARSE_UNIT, g[1] + d[4] * COARSE_UNIT, g[2] + d[5] * COARSE_UNIT]);
      } else if (errCnt) {
        for (let k = 0; k < 3; k++) needles[k] = Math.max(-384, Math.min(384, needles[k] + d[3 + k]));
      }
      d[3] = d[4] = d[5] = 0;
    }
    if (!errCnt) needles = [0, 0, 0];

    // Bloqueo de gimbal: pasado el límite, el cuerpo arrastra la plataforma.
    let g = gimbals();
    locked = Math.abs(g[2]) > GIMBAL_LOCK;
    if (locked) { g[2] = Math.sign(g[2]) * GIMBAL_LOCK; setGimbals(g); }

    // CDU → AGC: pulsos hasta que el contador alcance el ángulo real.
    for (let k = 0; k < 3; k++) {
      if (zeroCdu) { fed[k] = 0; continue; }   // el AGC está poniendo los contadores a cero
      const target = Math.round(wrap(g[k]) / CDU_UNIT);
      let diff = target - fed[k];
      diff -= 32768 * Math.round(diff / 32768);
      const type = diff > 0 ? 0o21 : 0o23;    // PCDU / MCDU rápidos
      for (let n = Math.abs(diff); n > 0; n--) agc.agc_inc(0o32 + k, type);
      fed[k] += diff;
    }
  }

  return {
    reset, powerUp, step, setMode, setStick, syncFromCdu,
    get gimbals() { return gimbals(); },
    // Ejes del cuerpo expresados en la plataforma (columnas): lo que dibuja la FDAI.
    get gimbalMatrix() { return mul(tr(S), B); },
    get rates() { return w.slice(); },
    get body() { return B; },
    get platform() { return S; },
    get needles() { return needles.slice(); },
    get jets() { return JETS.map((j, i) => [j[0], jetsOn[i]]); },
    get locked() { return locked; },
    // Para pruebas y controles manuales: fija una velocidad angular (rad/s, ejes del cuerpo).
    setRates(v) { w = v.slice(); },
  };
}

if (typeof module !== "undefined") module.exports = { LMSim };
