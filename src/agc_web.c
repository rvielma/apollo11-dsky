/*
 * agc_web.c — puente mínimo entre el motor yaAGC (Virtual AGC, GPLv2) y el navegador.
 * Carga la cuerda (ROM) desde memoria, avanza ciclos de máquina y expone el estado del DSKY.
 */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "yaAGC.h"
#include "agc_engine.h"

#define EXPORT(name) __attribute__((export_name(#name)))

static agc_t State;
static uint8_t Rom[36 * 02000 * 2];
static int32_t Disp[32];   /* [0..15] filas del canal 010, [16]=ch011, [17]=ch0163, [18]=contador de cambios */

/* Salidas hacia la nave simulada. El JavaScript las lee y pone a cero lo acumulado.
 *   [0..2]   pulsos de torque de giróscopo X, Y, Z (canal ficticio 0177), con signo
 *   [3..5]   pulsos de arrastre de CDU X, Y, Z (canales ficticios 0174-0176), con signo
 *   [8..15]  ciclos encendidos de cada chorro del canal 05, bits 1-8
 *   [16..23] ciclos encendidos de cada chorro del canal 06, bits 1-8
 *   [24..26] palanca RHC en cabeceo, guiñada y alabeo (-42..42 cuentas); la escribe el JavaScript */
static int32_t Io[32];

/* ---- API de E/S que el motor espera ---- */
void ChannelOutput(agc_t *s, int Channel, int Value) {
  /* Canal 7 = superbanco (FEB): tiene efecto dentro del propio CPU. */
  if (Channel == 7) {
    s->InputChannel[7] = s->OutputChannel7 = (Value & 0160);
    return;
  }
  if (Channel == 0177) {
    /* Bits 14-11 copian los del giróscopo del canal 014: signo, selección A y B, energía. */
    int n = Value & 03777, axis = (Value >> 12) & 3;
    if (axis) Io[axis - 1] += (Value & 040000) ? -n : n;   /* B=X (1), A=Y (2), A+B=Z (3) */
    return;
  }
  if (Channel >= 0174 && Channel <= 0176) {
    int n = Value & 037777;
    Io[3 + Channel - 0174] += (Value & 040000) ? -n : n;
    return;
  }
  /* El AGC pide leer la palanca de mando (RHC): canal 013, bits 8 y 9. */
  if (Channel == 013 && (Value & 0600) == 0600) {
    for (int k = 0; k < 3; k++) {
      int v = Io[24 + k];
      s->Erasable[0][042 + k] = v < 0 ? (~(-v) & 077777) : v;   /* complemento a uno */
    }
  }
  if (Channel == 010) {
    Disp[(Value >> 11) & 017] = Value & 03777;
    Disp[18]++;
  } else if (Channel == 011) {
    Disp[16] = Value;
    Disp[18]++;
  } else if (Channel == 0163) {
    Disp[17] = Value;
    Disp[18]++;
  }
}
int ChannelInput(agc_t *s) { (void)s; return 0; }
void ChannelRoutine(agc_t *s) { (void)s; }
void ShiftToDeda(agc_t *s, int Data) { (void)s; (void)Data; }
void RequestRadarData(agc_t *s) { (void)s; }

/* ---- Exportaciones al JavaScript ---- */
EXPORT(rom_ptr) uint8_t *rom_ptr(void) { return Rom; }
EXPORT(disp_ptr) int32_t *disp_ptr(void) { return Disp; }
EXPORT(io_ptr) int32_t *io_ptr(void) { return Io; }

EXPORT(agc_boot) int agc_boot(int nbytes) {
  int Bank, i, j;
  memset(&State, 0, sizeof(State));
  memset(Disp, 0, sizeof(Disp));
  memset(Io, 0, sizeof(Io));
  agc_engine_init(&State, NULL, NULL, 0);
  if (nbytes & 1 || nbytes > (int)sizeof(Rom)) return 1;
  State.CheckParity = 0;
  memset(&State.Parities, 0, sizeof(State.Parities));
  /* En el archivo los bancos van en orden 2, 3, 0, 1, 4, 5, ... 35 */
  for (Bank = 2, j = 0, i = 0; i < nbytes / 2; i++) {
    uint16_t raw = (uint16_t)(Rom[2 * i] << 8 | Rom[2 * i + 1]);
    State.Fixed[Bank][j] = raw >> 1;
    if (++j == 02000) {
      j = 0;
      Bank = Bank == 2 ? 3 : Bank == 3 ? 0 : Bank == 0 ? 1 : Bank == 1 ? 4 : Bank + 1;
    }
  }
  return 0;
}

/* Ejecuta n ciclos de máquina (cada uno ≈ 11,72 µs del AGC real). */
EXPORT(agc_run) int agc_run(int n) {
  for (int i = 0; i < n; i++) {
    agc_engine(&State);
    /* Los chorros RCS se encienden por milisegundos: se integra el tiempo encendido ciclo a ciclo. */
    int j5 = State.InputChannel[05] & 0377, j6 = State.InputChannel[06] & 0377;
    if (j5 | j6)
      for (int b = 0; b < 8; b++) {
        Io[8 + b] += (j5 >> b) & 1;
        Io[16 + b] += (j6 >> b) & 1;
      }
  }
  return Disp[18];
}

/* Pulso externo a un contador (CDU, PIPA, etc.): tipos de UnprogrammedIncrement. */
EXPORT(agc_inc) void agc_inc(int counter, int type) { UnprogrammedIncrement(&State, counter, type); }

/* Canales de E/S: entradas 030-033 (activas en bajo) y salidas 011-014. */
EXPORT(agc_chan) int agc_chan(int ch) { return State.InputChannel[ch & 0377]; }
EXPORT(agc_set_chan) void agc_set_chan(int ch, int v) { State.InputChannel[ch & 0377] = v & 077777; }

/* Memoria borrable: lectura (depurar, tests) y escritura (cargar un volcado de misión). */
EXPORT(agc_erasable) int agc_erasable(int addr) { return State.Erasable[(addr >> 8) & 7][addr & 0377]; }
EXPORT(agc_set_erasable) void agc_set_erasable(int addr, int v) { State.Erasable[(addr >> 8) & 7][addr & 0377] = v & 077777; }

/* Tecla del DSKY: código de 5 bits al canal 015 + KEYRUPT1. */
EXPORT(agc_key) void agc_key(int code) {
  WriteIO(&State, 015, code & 037);
  State.InterruptRequests[5] = 1;
}

/* PRO (canal 032, bit 14, activo en bajo). */
EXPORT(agc_pro) void agc_pro(int pressed) {
  int v = ReadIO(&State, 032);
  v = pressed ? (v & ~020000) : (v | 020000);
  WriteIO(&State, 032, v);
}

/* Stubs de utilidades de depuración y archivos que el motor referencia pero no usamos. */
void BacktraceAdd(agc_t *s, int Cause) { (void)s; (void)Cause; }
FILE *rfopen(const char *Filename, const char *mode) { (void)Filename; (void)mode; return NULL; }
