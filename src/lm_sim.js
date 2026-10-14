// lm_sim.js — la nave que rodea al AGC: actitud del LM, plataforma inercial (IMU) y chorros RCS.
//
// Ejes del LM: X hacia arriba (guiñada), Y a la derecha (cabeceo), Z hacia adelante (alabeo).
// La plataforma (stable member, SM) mantiene su orientación en el espacio salvo cuando el AGC
// la gira: torque fino de giróscopos (canal ficticio 0177) o arrastre grueso de los CDU (0174-0176).
// Los ángulos de gimbal se leen como la orientación del cuerpo respecto de la plataforma, en el orden
// del IMU del Apollo: interno (Y, IGA), medio (Z, MGA), externo (X, OGA). Cada cambio de ángulo
// llega al AGC como pulsos en los contadores CDUX/CDUY/CDUZ (032-034), 2^-15 revoluciones cada uno.
//
// Traslación: posición y velocidad en un marco inercial centrado en la Luna (el mismo del AGC cuando
// se carga un escenario), gravedad lunar puntual, empuje de chorros y motor de descenso a lo largo de
// los ejes del cuerpo. Los PIPA miden la aceleración no gravitacional en ejes de la plataforma y la
// entregan como pulsos en los contadores PIPAX/Y/Z (037-041).
//
// Simplificaciones: inercias constantes, empuje de chorros constante mientras están encendidos,
// motor de descenso alineado con el centro de masa (sin torque del cardán).

// eslint-disable-next-line no-unused-vars
function LMSim(agc) {
  "use strict";
  const AGC_HZ = 1024000 / 12;
  const CDU_UNIT = (2 * Math.PI) / 32768;        // un pulso de CDU
  const GYRO_UNIT = (0.617981 / 3600) * Math.PI / 180; // un pulso de torque fino
  const COARSE_UNIT = (2 * Math.PI) / 8192;      // un pulso de arrastre grueso
  const GIMBAL_LOCK = 85 * Math.PI / 180;        // el gimbal medio se traba cerca de 90°
  // Inercias del LM con las dos etapas cargadas, aproximadas (kg·m², las de NASSP).
  const INERTIA = [34500, 38400, 41600];
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
  // Empuje de cada chorro en ejes del cuerpo: se nombran por hacia dónde sale el escape, así que
  // empujan al revés. U (escape arriba) empuja hacia −X; D hacia +X; F hacia −Z; A hacia +Z;
  // L hacia +Y; R hacia −Y.
  const PUSH = { U: [-1, 0, 0], D: [1, 0, 0], F: [0, 0, -1], A: [0, 0, 1], L: [0, 1, 0], R: [0, -1, 0] };
  const JET_PUSH = JETS.map(([name]) => PUSH[name[2]]);

  // ── Traslación, motor de descenso y radar de aterrizaje ──
  // Valores de Luminary099 (CONTROLLED_CONSTANTS, THROTTLE_CONTROL_ROUTINES, SERVICER) y NASSP.
  const MU_MOON = 4.9027780e12;                  // m³/s², el del AGC
  const PIPA_UNIT = 0.01;                        // m/s por pulso de PIPA (LM: 1 cm/s, KPIP1)
  const THRUST_PULSE_N = 12.532;                 // N por pulso del contador THRUST (1/SCALEFAC)
  const DPS_MIN_N = 4670;                        // acelerador manual en mínimo (10 %): el DECA lo suma
  const DPS_FTP_N = 43203;                       // tope mecánico, "full throttle position" (~92,5 %)
  const DPS_SLEW = 40102;                        // N/s, la tasa de drenado que supone el AGC (3200 pps)
  const DPS_LAG = 0.2;                           // s, retardo del motor (THROTLAG)
  const DPS_VEX = 2955.89;                       // m/s de velocidad de escape (DPSVEX)
  const RCS_ISP = 2840;
  const FT = 0.3048;
  const LR_MAX_FT = 50000;                       // el radar entrega datos por debajo de esto (NASSP)
  const LR_LOW_SCALE_FT = 2500;                  // cambio a escala baja de altura
  const LR_ALPHA = -6 * Math.PI / 180;           // montaje de la antena (LRALPHA, convención NASSP)
  const LR_POS1 = 24 * Math.PI / 180;            // inclinación de la antena en posición 1; en la 2 es 0
  const LR_SLEW = 2.4 * Math.PI / 180;           // rad/s al pasar de posición 1 a 2
  const HBEAM = [-Math.cos(20.38 * Math.PI / 180), 0, -Math.sin(20.38 * Math.PI / 180)];  // haz de altura

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
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const wrap = (a) => a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI));

  let B, S, w, fed, needles, jetsOn, locked;
  let pos, vel, mass, engineOn, pipaRest, lastAccel;
  let fAuto, thrust, omega, landR, lrAngle, lrToPos2, landed, touchdown, contact, dpsFuel;
  function reset() {
    pos = [0, 0, 0]; vel = [0, 0, 0];   // marco inercial lunar (m, m/s); cero = sin traslación
    mass = 15000;                        // kg
    engineOn = false;
    fAuto = 0;                           // empuje acumulado por el AGC en el integrador del DECA (N)
    thrust = 0;                          // empuje real del motor de descenso (N)
    omega = [0, 0, 0];                   // rotación lunar en el marco inercial (rad/s)
    landR = 0;                           // radio de la superficie en el sitio de alunizaje (m)
    lrAngle = LR_POS1; lrToPos2 = false;
    landed = false; touchdown = null; contact = false;
    dpsFuel = 0;                         // propelente del motor de descenso (kg)
    pipaRest = [0, 0, 0];                // fracción de pulso aún no entregada, por eje de la plataforma
    lastAccel = [0, 0, 0];
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
    // Fuerza no gravitacional en ejes del cuerpo: chorros + motor de descenso
    const force = [0, 0, 0];
    for (let j = 0; j < 16; j++) if (jetsOn[j]) for (let k = 0; k < 3; k++) force[k] += JET_PUSH[j][k] * JET_N * jetsOn[j];
    let flow = force.some((x) => x) ? (JETS.reduce((s, _, j) => s + jetsOn[j], 0) * JET_N) / RCS_ISP : 0;
    // Motor: el canal 11 enciende (bit 13) o apaga (bit 14); con ninguno, queda como estaba.
    // Solo arranca armado (canal 30 bit 3, que pone la tripulación) y antes de posarse.
    const ch11 = agc.agc_chan(0o11), armed = (agc.agc_chan(0o30) & 0o4) === 0;
    if (ch11 & 0o20000) engineOn = false;
    else if (ch11 & 0o10000) engineOn = armed && !landed;
    // Acelerador: pulsos del contador THRUST a un integrador; empuje = mínimo + integrador,
    // limitado por el tope mecánico, con rampa y retardo.
    if (d[27]) { fAuto = Math.max(0, Math.min(DPS_FTP_N, fAuto + d[27] * THRUST_PULSE_N)); d[27] = 0; }
    const target = engineOn ? Math.min(DPS_FTP_N, DPS_MIN_N + fAuto) : 0;
    const lagged = thrust + (target - thrust) * Math.min(1, dt / DPS_LAG);
    thrust = Math.max(thrust - DPS_SLEW * dt, Math.min(thrust + DPS_SLEW * dt, lagged));
    if (thrust > 1 && dpsFuel <= 0 && landR) thrust = 0;   // sin propelente no hay empuje
    if (thrust > 1) { force[0] += thrust; flow += thrust / DPS_VEX; dpsFuel = Math.max(0, dpsFuel - (thrust / DPS_VEX) * dt); }
    mass = Math.max(4000, mass - flow * dt);
    if (landed) {
      // Posado: gira con la Luna, sin aceleración medida salvo el apoyo contra la gravedad.
      const wr = cross(omega, pos);
      for (let i = 0; i < 3; i++) pos[i] += wr[i] * dt;
      vel = cross(omega, pos);
      w = [0, 0, 0];
    } else if (pos[0] || pos[1] || pos[2]) {
      // Aceleración inercial: empuje rotado al marco inercial + gravedad lunar
      const aI = [0, 1, 2].map((i) => (B[i][0] * force[0] + B[i][1] * force[1] + B[i][2] * force[2]) / mass);
      const r = Math.hypot(pos[0], pos[1], pos[2]), gk = -MU_MOON / (r * r * r);
      const a = [0, 1, 2].map((i) => aI[i] + gk * pos[i]);
      for (let i = 0; i < 3; i++) { vel[i] += a[i] * dt; pos[i] += vel[i] * dt; }
      lastAccel = aI;
      if (landR) {
        const h = Math.hypot(pos[0], pos[1], pos[2]) - landR;
        contact = h < 1.7;                         // las sondas de 1,7 m tocan el suelo: CONTACT
        if (h <= 0) {
          const up = pos.map((x) => x / (h + landR)), rel = sub(vel, cross(omega, pos));
          touchdown = { vVert: dot(rel, up), vHoriz: Math.hypot(...sub(rel, up.map((x) => x * dot(rel, up)))) };
          // Límites del tren de aterrizaje del LM: ~10 ft/s vertical y ~4 ft/s horizontal.
          touchdown.crash = -touchdown.vVert > 3.05 || touchdown.vHoriz > 1.22;
          // Al posarse la tripulación apaga el motor (ENGINE STOP) y lo desarma.
          landed = true; engineOn = false; thrust = 0;
          agc.agc_set_chan(0o30, agc.agc_chan(0o30) | 0o4);
          for (let i = 0; i < 3; i++) pos[i] = up[i] * landR;
        }
      }
      // PIPA: lo no gravitacional, en ejes de la plataforma, en pulsos enteros
      for (let k = 0; k < 3; k++) {
        pipaRest[k] += (S[0][k] * aI[0] + S[1][k] * aI[1] + S[2][k] * aI[2]) * dt / PIPA_UNIT;
        const n = Math.trunc(pipaRest[k]);
        if (n) {
          pipaRest[k] -= n;
          for (let c = Math.abs(n); c > 0; c--) agc.agc_inc(0o37 + k, n > 0 ? 0 : 2);   // PINC / MINC
        }
      }
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

    if (landR) landingRadar(dt, ch12);
  }

  // Ejes de la antena del radar de aterrizaje en ejes del cuerpo (filas), para una inclinación b.
  function lrAxes(b) {
    const a = LR_ALPHA, be = -b, sa = Math.sin(a), ca = Math.cos(a), sb = Math.sin(be), cb = Math.cos(be);
    return [[cb, sa * sb, -sb * ca], [0, ca, sa], [sb, -sa * cb, cb * ca]];
  }
  let lr = { good: false, slantFt: 0, lowScale: false, v: [0, 0, 0] };
  function landingRadar(dt, ch12) {
    // Posición de la antena: el canal 12 bit 13 la manda a la posición 2 (vertical para P64) y
    // allí se queda; volver a la 1 requiere el interruptor de cabina (como en NASSP).
    if (ch12 & 0o10000) lrToPos2 = true;
    const goal = lrToPos2 ? 0 : LR_POS1;
    lrAngle += Math.max(-LR_SLEW * dt, Math.min(LR_SLEW * dt, goal - lrAngle));
    const U = lrAxes(lrAngle);
    const beamB = [0, 1, 2].map((k) => HBEAM[0] * U[0][k] + HBEAM[1] * U[1][k] + HBEAM[2] * U[2][k]);
    const beamI = [0, 1, 2].map((i) => B[i][0] * beamB[0] + B[i][1] * beamB[1] + B[i][2] * beamB[2]);
    const rr = Math.hypot(pos[0], pos[1], pos[2]), down = pos.map((x) => -x / rr);
    const h = rr - landR, cosT = dot(beamI, down);
    const slantFt = cosT > 0.2 ? h / cosT / FT : Infinity;
    const good = !landed && slantFt < LR_MAX_FT;
    const lowScale = slantFt < LR_LOW_SCALE_FT;
    // Velocidad relativa al terreno, en ejes de la antena (ft/s)
    const relI = sub(vel, cross(omega, pos));
    const relB = [0, 1, 2].map((k) => B[0][k] * relI[0] + B[1][k] * relI[1] + B[2][k] * relI[2]);
    const v = U.map((u) => dot(u, relB) / FT);
    const d = ioView(), cnt = (x) => Math.max(0, Math.min(0o77777, Math.round(x)));
    d[28] = cnt(slantFt === Infinity ? 0 : slantFt / (lowScale ? 1.079 : 5.395));
    d[29] = cnt(12288 - v[0] / 0.644);
    d[30] = cnt(12288 + v[1] / 1.212);
    d[31] = cnt(12288 + v[2] / 0.8668);
    // Canal 33 (activo en bajo): b5 altura buena, b6 antena en pos. 1, b7 en pos. 2, b8 velocidad buena, b9 escala baja
    let c33 = agc.agc_chan(0o33) | 0o760;
    if (good) c33 &= ~(0o20 | 0o200);
    if (Math.abs(lrAngle - LR_POS1) < 1e-3) c33 &= ~0o40;
    if (Math.abs(lrAngle) < 1e-3) c33 &= ~0o100;
    if (lowScale) c33 &= ~0o400;
    agc.agc_set_chan(0o33, c33);
    lr = { good, slantFt, lowScale, v, angle: lrAngle };
  }

  // Estado inicial a partir de la memoria del AGC (tras cargar un volcado de misión). El marco
  // inercial de la simulación pasa a ser el de la plataforma: la nave arranca justo donde el AGC
  // cree que está, con su masa, la rotación lunar y el radio del sitio de alunizaje.
  // fuel: propelente del motor de descenso al cargar el escenario (kg).
  function initFromMemory(fuel = 8248) {
    const sp = (a) => { const w = agc.agc_erasable(a); return w & 0o40000 ? -(~w & 0o77777) : w; };
    const dp = (a, b) => (sp(a) * 16384 + sp(a + 1)) * 2 ** (b - 28);
    const vec = (a, b) => [0, 1, 2].map((i) => dp(a + 2 * i, b));
    const now = (agc.agc_erasable(0o24) * 16384 + agc.agc_erasable(0o25)) / 100;
    let r = vec(0o1220, 29), v = vec(0o1226, 7).map((x) => x * 100);   // RN, VN (referencia)
    let t = dp(0o1234, 28) / 100;                                         // PIPTIME
    const acc = (p) => { const k = -MU_MOON / Math.hypot(...p) ** 3; return p.map((x) => k * x); };
    while (t < now - 1e-9) {                                              // RK4 hasta la hora actual
      const h = Math.min(0.5, now - t), add = (a, b, s) => a.map((x, i) => x + b[i] * s);
      const a1 = acc(r), r2 = add(r, v, h / 2), v2 = add(v, a1, h / 2), a2 = acc(r2);
      const r3 = add(r, v2, h / 2), v3 = add(v, a2, h / 2), a3 = acc(r3), r4 = add(r, v3, h), v4 = add(v, a3, h), a4 = acc(r4);
      r = r.map((x, i) => x + (h / 6) * (v[i] + 2 * v2[i] + 2 * v3[i] + v4[i]));
      v = v.map((x, i) => x + (h / 6) * (a1[i] + 2 * a2[i] + 2 * a3[i] + a4[i]));
      t += h;
    }
    const REF = [0, 1, 2].map((i) => [0, 1, 2].map((j) => dp(0o1733 + 6 * i + 2 * j, 1)));
    pos = REF.map((row) => dot(row, r));
    vel = REF.map((row) => dot(row, v));
    omega = vec(0o2324, -17).map((x) => x * 100);                        // WM, ya en la plataforma
    landR = dp(0o2332, 24);                                               // /LAND/
    mass = dp(0o1244, 16);                                                // MASS
    dpsFuel = fuel;
    agc.agc_set_chan(0o30, agc.agc_chan(0o30) & ~0o4);                   // motor de descenso armado
    S = I3();
    const counts = [0o32, 0o33, 0o34].map((a) => agc.agc_erasable(a) & 0o77777);
    B = gimbalsToM(...counts.map((c) => c * CDU_UNIT));
    fed = counts.slice();
    lrAngle = LR_POS1; lrToPos2 = false;
    // El AGC lee la posición de la antena al reiniciar: debe verla ya en la posición 1 (canal 33
    // bit 6 en bajo) y sin datos del radar, o anota la posición 2 y luego da la alarma 522.
    agc.agc_set_chan(0o33, (agc.agc_chan(0o33) | 0o760) & ~0o40);
  }

  return {
    reset, powerUp, step, setMode, setStick, syncFromCdu, initFromMemory,
    get gimbals() { return gimbals(); },
    // Ejes del cuerpo expresados en la plataforma (columnas): lo que dibuja la FDAI.
    get gimbalMatrix() { return mul(tr(S), B); },
    get rates() { return w.slice(); },
    get body() { return B; },
    get platform() { return S; },
    get needles() { return needles.slice(); },
    get jets() { return JETS.map((j, i) => [j[0], jetsOn[i]]); },
    get locked() { return locked; },
    get position() { return pos.slice(); },
    get velocity() { return vel.slice(); },
    get mass() { return mass; },
    get engine() { return { on: engineOn, thrust, fraction: thrust / 46706 }; },
    get radar() { return lr; },
    // Para la vista por la ventana: radio de la superficie y rotación lunar (marco inercial)
    get surfaceRadius() { return landR; },
    get moonRate() { return omega.slice(); },
    get fuel() { return dpsFuel; },
    get landed() { return landed; },
    get touchdown() { return touchdown; },
    get contact() { return contact; },
    // Altura sobre el sitio, velocidad vertical y horizontal relativas al terreno (m, m/s)
    get surface() {
      if (!landR) return null;
      const rr = Math.hypot(...pos), up = pos.map((x) => x / rr), rel = sub(vel, cross(omega, pos)), vv = dot(rel, up);
      return { h: rr - landR, vVert: vv, vHoriz: Math.hypot(...sub(rel, up.map((x) => x * vv))) };
    },
    get accel() { return lastAccel.slice(); },
    // Estado traslacional inicial (marco inercial lunar), masa y orientación de la plataforma.
    setState(r, v, m) { pos = r.slice(); vel = v.slice(); if (m) mass = m; },

    // Para pruebas y controles manuales: fija una velocidad angular (rad/s, ejes del cuerpo).
    setRates(v) { w = v.slice(); },
  };
}

if (typeof module !== "undefined") module.exports = { LMSim };
