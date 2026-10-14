# Apollo 11 DSKY

El software de vuelo original del Apollo 11 corriendo en el navegador. Luminary099 (Módulo Lunar) y Comanche055 (Módulo de Comando) se ejecutan ciclo a ciclo con el motor de Virtual AGC compilado a WebAssembly, con un DSKY interactivo.

Con Luminary099 el AGC vuela un LM simulado: plataforma inercial (IMU), actitud y los 16 chorros RCS. El computador lee los gimbals por sus contadores CDU, su autopiloto (DAP) mantiene la actitud, ejecuta maniobras automáticas (`V49`) y obedece la palanca de mando en ATT HOLD. La página muestra la bola de actitud (FDAI) y los chorros encendidos.

**Descenso del Apollo 11.** Un botón carga el computador del Eagle 15 minutos antes del descenso motorizado (memoria borrable del escenario de NASSP) y el software de vuelo hace el resto: P63 (frenado), P64 (aproximación) y P65 o P66 hasta posarse, con el motor de descenso, los acelerómetros y el radar de aterrizaje simulados. Un asistente hace lo que hacía la tripulación (ENTR, PRO para encender, giro a ventanas arriba, `V57` para aceptar el radar, PRO en P64) o te deja hacerlo a ti. En la prueba de referencia se posa a ~3 ft/s con ~7 % de combustible, sin alarmas.

## Qué hay aquí

```
index.html              El sitio completo (≈340 KB). Motor wasm, cuerdas y simulador van embebidos.
src/agc_web.c           Puente en C entre el motor yaAGC y el navegador.
src/lm_sim.js           La nave: IMU, dinámica de actitud y chorros RCS del LM.
src/dsky_template.html  Plantilla del panel (HTML/CSS/JS) con marcadores para wasm, ROMs y simulador.
src/prueba_nave.js      Prueba en Node: IMU, autopiloto, palanca y el descenso completo hasta posarse.
src/escenario_pdi.json  Memoria del computador del LM 15 min antes del PDI (de NASSP).
src/extraer_escenario.py Regenera escenario_pdi.json desde el .scn de NASSP.
src/build.sh            Clona Virtual AGC, ensambla las cuerdas, compila a wasm y regenera index.html.
src/VIRTUALAGC_COMMIT   Commit exacto de Virtual AGC usado.
```

## Publicar

Es un sitio estático de un solo archivo: no necesita servidor de aplicaciones, base de datos ni configuración de MIME para `.wasm` (va embebido en base64). Sube `index.html` a cualquier hosting estático.

**Servidor propio (nginx)**

```nginx
server {
    server_name apollo.tudominio.cl;
    root /var/www/apollo11-dsky;
    index index.html;
    gzip on;
    gzip_types text/html;
    add_header Cache-Control "public, max-age=3600";
}
```

Con gzip el archivo baja a unos 200 KB en la transferencia.

**S3 + CloudFront**

```bash
aws s3 cp index.html s3://TU-BUCKET/index.html --content-type "text/html; charset=utf-8"
```

**GitHub Pages / Cloudflare Pages / Netlify**

Sube la carpeta a un repositorio y apunta el sitio a la raíz. En Netlify también puedes arrastrar la carpeta a app.netlify.com/drop.

**Probar localmente**

```bash
python3 -m http.server 8000   # luego abre http://localhost:8000
```

## Recompilar

```bash
cd src && ./build.sh
```

Necesita `git`, `make`, un compilador C nativo y `python3` con pip. El compilador nativo solo se usa para el ensamblador yaYUL: el script lo invoca como `gcc`, que en macOS es clang y sirve igual. El motor no usa ese compilador: el script instala `ziglang` por pip y compila a `wasm32-wasi`.

La versión de `ziglang` no está fijada, así que el `.wasm` puede variar en algunos bytes respecto del incluido. Las cuerdas de memoria salen idénticas, porque vienen del commit fijado en `VIRTUALAGC_COMMIT`.

Clonar Virtual AGC descarga el repositorio completo, que es grande. Cuenta con varios minutos y algunos GB libres.

## Probar la nave

```bash
node src/prueba_nave.js
```

Carga el wasm, Luminary099 y el escenario desde `index.html`, corre ~40 minutos de tiempo AGC en unos 15 segundos y comprueba el comportamiento de vuelo, incluido el descenso completo hasta posarse. Correrla después de tocar `lm_sim.js`, `agc_web.c` o la secuencia de arranque.

## Licencias

- Software de vuelo del AGC (Luminary099, Comanche055): MIT Instrumentation Laboratory, dominio público.
- Motor yaAGC: Virtual AGC (Ron Burkey y colaboradores), GPLv2. https://github.com/virtualagc/virtualagc
- Estado del computador del LM 15 minutos antes del descenso (`src/escenario_pdi.json`): extraído del escenario "Apollo 11 - 10 - PDI T-15m" de NASSP (Project Apollo para Orbiter), GPLv2. https://github.com/orbiternassp/NASSP. Se regenera con `python3 src/extraer_escenario.py <escenario.scn>`.
- `src/agc_web.c` se enlaza con yaAGC, así que se distribuye también bajo GPLv2, igual que el resto de `src/`. El código está en https://github.com/rvielma/apollo11-dsky y el pie del sitio lo enlaza.

## Detalle técnico

El puente implementa la API de canales que espera el motor (`ChannelOutput`, `ChannelInput`, etc.):

- Canal 010: filas del relé del display. Cada palabra trae dos dígitos de 5 bits y un bit de signo.
- Canal 011 y canal ficticio 0163: luces COMP ACTY, KEY REL, OPR ERR, RESTART y parpadeo de VERB/NOUN.
- Canal 015: teclas del DSKY, más la interrupción KEYRUPT1.
- Canal 032, bit 14: tecla PRO, activa en bajo.
- Canal 7: superbanco (FEB). Debe devolverse al CPU; si se ignora, el AGC salta a bancos equivocados y entra en bucle de reinicios por TC TRAP y NIGHT WATCHMAN.

### Nave simulada (Luminary099)

`lm_sim.js` avanza la nave cada 10 ms de tiempo AGC con lo que el computador ordenó en ese tramo:

- Canal 030: al arrancar se activan IMU OPERATE (bit 9) y "LGC al mando" (bit 10). Los canales 030-033 son activos en bajo.
- Contadores CDUX/CDUY/CDUZ (032-034): cada cambio de ángulo de gimbal llega como pulsos PCDU/MCDU (2^-15 rev). Orden del IMU: interno Y (IGA), medio Z (MGA), externo X (OGA).
- Canal ficticio 0177: torque fino de giróscopos, gira la plataforma sobre sus ejes (0,618″ por pulso).
- Canales ficticios 0174-0176: con el bit 4 del canal 012 (alineación gruesa) mueven los gimbals; si no, van a las agujas de error del FDAI.
- Canales 05 y 06: los 16 chorros RCS. El puente integra el tiempo encendido de cada chorro ciclo a ciclo, porque el DAP los dispara por milisegundos.
- Canal 031: selector de modo (bit 13 ATT HOLD, bit 14 AUTO) y palanca fuera del centro (bit 15). Los contadores RHCP/RHCY/RHCR (042-044) se cargan cuando el AGC escribe el canal 013 con los bits 8 y 9.
- Al arrancar se teclea `V48` como hacía la tripulación: N46 = 20102 y N47 = 33.300 lb. Sin masa, el autopiloto no tiene inercias y las maniobras se pasan de largo. El trim del motor de descenso (N48) se salta con `V34`, porque no hay motor simulado.
- Mientras pone en marcha la IMU, el AGC rechaza `V37` durante unos segundos (alarma 1520). Por eso el arranque carga primero el autopiloto y después pide P00.

### Descenso (escenario PDI −15 min)

- Estado inicial: la memoria borrable y los canales 030-033 del escenario de NASSP. El AGC arranca en 04000 como tras un corte de energía y su protección de reinicio retoma P63. La nave toma RN/VN (propagados desde PIPTIME), REFSMMAT, la rotación lunar WM, el radio del sitio /LAND/ y la masa MASS de esa misma memoria; el marco inercial de la simulación es el de la plataforma.
- Motor de descenso: canal 011 bit 13 enciende y bit 14 apaga; requiere el motor armado (canal 030 bit 3). Acelerador: el contador THRUST (055) se descarga cuando el canal 014 bit 4 está activo; cada pulso suma 12,53 N a un integrador (como el DECA), sobre el 10 % del acelerador manual, con tope en FTP (43.203 N), rampa de 40 kN/s y retardo de 0,2 s. Velocidad de escape 2955,9 m/s.
- PIPA: 1 cm/s por pulso, en ejes de la plataforma.
- Radar de aterrizaje: el motor yaAGC maneja la espera y la interrupción RADARUPT; el puente responde en RNRAD (046) según el canal 013. Altura oblicua a lo largo del haz (1,079 o 5,395 ft por cuenta según la escala, canal 033 bit 9), velocidades VX/VY/VZ relativas al terreno en ejes de la antena con sesgo 12288. Antena en posición 1 (24°) o 2 (vertical, la manda el canal 012 bit 13 y allí se queda). Datos buenos por debajo de 50.000 ft si el haz ve el suelo: con ventanas hacia la Luna no lo ve, por eso el giro a ventanas arriba.
- El canal 033 tiene que mostrar la antena en posición 1 antes del reinicio: el AGC la lee ahí y, si no, espera la posición 2 y da la alarma 522 con cada dato.
- Contacto: a 1,7 m se enciende CONTACT; al llegar al suelo el motor se apaga y se desarma.

Simplificaciones: Luna esférica (sin terreno), sin radar de encuentro (no hay alarmas 1201/1202), sin cardán del motor (el empuje pasa por el centro de masa), inercias constantes y aproximadas, empuje constante por chorro y bloqueo de gimbal simplificado (la plataforma es arrastrada pasados 85° de MGA).
