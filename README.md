# Apollo 11 DSKY

El software de vuelo original del Apollo 11 corriendo en el navegador. Luminary099 (Módulo Lunar) y Comanche055 (Módulo de Comando) se ejecutan ciclo a ciclo con el motor de Virtual AGC compilado a WebAssembly, con un DSKY interactivo.

Con Luminary099 el AGC vuela un LM simulado: plataforma inercial (IMU), actitud y los 16 chorros RCS. El computador lee los gimbals por sus contadores CDU, su autopiloto (DAP) mantiene la actitud, ejecuta maniobras automáticas (`V49`) y obedece la palanca de mando en ATT HOLD. La página muestra la bola de actitud (FDAI) y los chorros encendidos.

## Qué hay aquí

```
index.html              El sitio completo (≈340 KB). Motor wasm, cuerdas y simulador van embebidos.
src/agc_web.c           Puente en C entre el motor yaAGC y el navegador.
src/lm_sim.js           La nave: IMU, dinámica de actitud y chorros RCS del LM.
src/dsky_template.html  Plantilla del panel (HTML/CSS/JS) con marcadores para wasm, ROMs y simulador.
src/prueba_nave.js      Prueba en Node: el AGC lee la IMU, frena rotaciones, maniobra y obedece la palanca.
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

Carga el wasm y Luminary099 desde `index.html`, corre unos minutos de tiempo AGC en un segundo y comprueba el comportamiento de vuelo. Correrla después de tocar `lm_sim.js`, `agc_web.c` o la secuencia de arranque.

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

Simplificaciones: sin traslación (los PIPA quedan en cero), inercias constantes y aproximadas, empuje constante por chorro y bloqueo de gimbal simplificado (la plataforma es arrastrada pasados 85° de MGA).
