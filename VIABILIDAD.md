# Viabilidad del seguidor de línea en la pista con V cerradas

Fecha: 2026-09-27 · Código analizado: [carro_v10/carro_v10.ino](carro_v10/carro_v10.ino)

## 1. Conclusión

**Sí es viable, pero solo a velocidad moderada** y si se cumplen tres condiciones medibles:

1. Aproximarse a las V a **≤ 30 cm/s**.
2. Frenar y empezar a girar a **menos de ~4 cm** después del vértice.
3. Que la alimentación **no provoque reinicios** (brownout) al girar en el sitio.

Rápido, con esta geometría y este hardware, no es viable.

Los fallos de las versiones anteriores eran de **lógica, no de hardware**: la v8.8 giraba hacia el
lado contrario en la V. El carro es físicamente capaz; lo que fallaba era la decisión.

## 2. Supuestos

### Hardware
- ESP32 + puente H L298N. Motor A = rueda derecha, motor B = rueda izquierda.
- 6 sensores IR digitales en fila, **9 cm del primero al último** (1.8 cm entre sensores), HIGH = negro.
- La separación entre sensores (1.8 cm) es igual al ancho de la línea (1.8 cm), y cada sensor
  detecta un punto más angosto que eso: la línea puede caer en el hueco entre dos sensores sin que
  ninguno la vea.
- La barra de sensores está **11 cm delante del centro de las ruedas** (medido).
- **15 cm** de extremo a extremo de las ruedas (medido). Solo influye en qué tan rápido gira en el
  sitio, y eso se mide directamente con la calibración `T_180_MS`.
- Ya se observaron reinicios por bajo voltaje (brownout).

### Pista ([image.png](image.png))
Las medidas son **estimadas**, tomando como escala la línea de ~1.8 cm de ancho en la foto.

- Circuito cerrado de ~100 cm de alto y ~3 m de recorrido.
- Curvas suaves en el lado derecho y en el lado izquierdo largo.
- Un zigzag arriba con tramos de ~23-28 cm y 5 vértices agudos:

| Vértice | Ubicación | Ángulo entre ramas | Giro necesario |
|---|---|---|---|
| A | arriba a la izquierda | ~26-35° | ~150° |
| B | zigzag, derecha | ~42° | ~140° |
| C | zigzag, izquierda | ~37° | ~143° |
| D | zigzag, derecha (abajo) | ~59° | ~120° |
| E | abajo a la izquierda | ~38° | ~142° |

- No hay cruces ni ramas paralelas.

## 3. Lo que juega a favor

- **Las curvas suaves** no son problema para un seguidor PD con 6 sensores.
- **Girar en el sitio siempre encuentra la rama nueva, si gira hacia el lado correcto.** Al girar
  sobre el eje, los sensores barren un círculo de 11 cm de radio. Ese círculo corta la rama nueva
  antes que la vieja: con el eje 11 cm antes del vértice, la rama nueva aparece a unos
  `180° − 2θ` de giro (~110° con θ = 35°) y la vieja a 180°. Sigue funcionando aunque el carro se
  haya pasado un poco del vértice. Es geometría, no depende del ajuste.
- **Una V tiene una firma clara.** La rama de salida aparece como una segunda marca al costado
  antes del vértice, se junta con la línea propia y luego todo queda blanco. El lado de esa marca
  es el lado del giro. En una curva, en cambio, la línea sale por el borde de la barra.

## 4. Riesgos, del más grave al menos grave

### Riesgo 1: ver la V a tiempo
La otra rama solo se ve en un tramo corto antes del vértice:

| Vértices | Ángulo | Tramo donde se ve la otra rama |
|---|---|---|
| A, B, C, E | 35-42° | ~2.0-2.6 cm |
| D | ~59° | ~1.1 cm |

(Con la barra de 9 cm: la rama se ve desde que entra en el sensor del extremo, a 4.5 cm de la línea
propia, hasta que queda a menos de ~2.7 cm y ya no se distingue de ella.)

Cada lectura tarda 3 ms y se necesitan 2 seguidas para confirmar (antirrebote).

- A 30 cm/s, 1 cm son ~11 lecturas: alcanza.
- A 60 cm/s son ~5 lecturas: queda justo.
- Además, la marca puede caer en el hueco entre sensores. En un vértice de 35° la rama se corre
  ~1.8 cm de lado mientras es visible, justo una separación entre sensores: pasa por al menos uno.
  En D (59°) se corre más rápido, pero el tramo es más corto.

**→ La velocidad de aproximación a las V es lo que decide si funciona.**

### Riesgo 2: pasarse del vértice en B y D
En la imagen, la curva del lado derecho pasa a solo **~5-7 cm a la derecha de los vértices B y D**.
Con la barra de 9 cm, el sensor del extremo alcanza esa curva si los sensores pasan el vértice
**~4 cm en D** o **~5.5 cm en B** (llegando desde A hacia B y desde C hacia D). En ese caso la toma
como evidencia del lado contrario y gira mal, o sigue la línea equivocada.

**→ La distancia de frenado tiene que ser menor de ~4 cm.** D es el vértice más exigente.

### Riesgo 3: tramos cortos del zigzag
Con los sensores 11 cm delante del eje, al girar en un vértice el carro retoma la línea
**11-16 cm después del vértice**, y queda torcido 35-40° respecto a la rama nueva. En un tramo de
~25 cm le quedan unos 10 cm para enderezarse antes de que empiece la señal de la siguiente V. Si
llega torcido, esa señal es más débil.

**→ Manejable a baja velocidad.** Ayuda seguir girando un poco más allá del centro de la línea
(`REENG_E` negativo, por ejemplo `-3`, la línea a ~2.7 cm del centro): así queda ~14° menos torcido.

### Riesgo 4: alimentación
El L298N pierde ~2 V, y girar en el sitio con una rueda en reversa pide picos de corriente. Hay 5
giros de ese tipo por vuelta.

**→** La rampa de PWM que ya tiene el código ayuda. Si vuelve a reiniciarse, la solución es de
hardware:
- un condensador grande (≥ 1000 µF) en la entrada del L298N, o
- alimentar el ESP32 con un regulador aparte, o
- mejor aún, cambiar el L298N por un TB6612FNG (pierde mucho menos voltaje).

## 5. Estimación de tiempo de vuelta

| Concepto | Valor |
|---|---|
| Longitud de la pista | ~3 m |
| Recorrido a ~25 cm/s | ~12 s |
| 5 giros en el sitio de ~0.6-0.8 s | ~3-4 s |
| **Total por vuelta** | **~15-17 s** |

Si la competencia tiene un tiempo máximo, hay que compararlo con esto.

## 6. Pruebas para decidir (unos 30 minutos)

| # | Prueba | Cómo | Resultado que la aprueba |
|---|---|---|---|
| 1 | Velocidad real | `modo 1`: avanza derecho 2 s y se detiene | Saber cuántos cm/s son (meta en las V: ≤ 30 cm/s) |
| 2 | Frenado | `modo 2`: sigue la línea y se detiene donde decide girar en una V | Los sensores pasan el vértice < 4 cm |
| 3 | Giro aislado | `modo 3`: parado antes de un vértice, gira hacia el lado indicado | Toma la rama nueva, no la vieja |
| 4 | Ancho de detección | `modo 4`: motores apagados, registra `sens=` | Saber si hay huecos donde ningún sensor ve la línea |
| 5 | Alimentación | `modo 5`: 30 s girando en el sitio | Ningún reinicio por bajo voltaje en el log |

**Si las 5 salen bien, el proyecto es viable con este hardware.**

### Configuración por Bluetooth

El programa se sube por cable **una sola vez**. Después, el modo de prueba y los parámetros se
cambian desde una terminal serie conectada a "SeguidorV88":

- **PC:** doble clic en [terminal_carro.bat](terminal_carro.bat). Busca sola el puerto del carro,
  se reconecta sola si el carro se apaga y guarda todo en `logs\`.
- **Celular Android:** la app "Serial Bluetooth Terminal". En iPhone no funciona: iOS no se conecta
  al Bluetooth clásico del ESP32.

| Comando | Qué hace |
|---|---|
| `?` | lista de comandos |
| `ver` | muestra todos los parámetros y su valor |
| `<param> <n>` | cambia un parámetro al instante. Ej: `modo 2`, `aprox 70`, `kp 18` |
| `guardar` | guarda los parámetros en el ESP32: se mantienen al apagar |
| `fabrica` | vuelve a los valores escritos en el código |
| `go` | arranca en 3 s (tiempo para soltarlo) |
| `stop` o `x` | detiene los motores ya |

Al encender, el carro **espera `go`**. Día de carrera: `modo 0`, `auto 1` (arranca solo 3 s después
de encender), `log 0` y `guardar`.

### Cómo hacer cada prueba

Para todas: escribir `modo N` (N = número de la prueba), colocar el carro y escribir `go`. Al
terminar, `modo 0`.

**Prueba 1 - Velocidad real**
1. Pegar una cinta métrica o marcar el piso en una zona recta y lisa (no hace falta línea).
2. Poner el carro con el eje sobre la marca 0. `modo 1`, `go`.
3. A los 3 s avanza derecho 2 s (`prectams 2000`) y se detiene.
4. Medir la distancia recorrida por el eje. Velocidad = distancia / 2 s (por ejemplo 60 cm → 30 cm/s).
5. Repetir con `prectapwm 80`, que es la velocidad con la que llega a las V (`aprox`).
6. Si se desvía mucho hacia un lado, anotarlo: los dos motores no rinden igual.

**Prueba 2 - Frenado en la V**
1. Poner el carro en la pista unos 30 cm antes de una V (idealmente D, la más exigente). `modo 2`, `go`.
2. El carro sigue la línea normal. En cuanto decide girar apaga los motores y se queda quieto.
3. Medir la distancia desde la **punta del vértice** hasta la **fila de sensores**, en la dirección
   en que venía el carro.
4. En el log debe aparecer `GIRO V linea acabada con marca`. Si aparece `sin marca`, no vio venir
   la otra rama: anotarlo, es tan importante como la distancia.
5. Repetir 3-5 veces y en varias V. Vale el peor resultado.

Los motores quedan sueltos en esta prueba; en carrera la rueda en reversa frena más. La medida sale
un poco peor que la real, lo que deja margen.

**Prueba 3 - Giro aislado**
1. `modo 3` y `lado` hacia dónde está la rama nueva: `lado 1` si está a la izquierda del carro,
   `lado -1` si está a la derecha.
2. Poner el carro sobre la rama de llegada, con los sensores justo en la punta del vértice (el eje
   queda 11 cm antes).
3. `go`. A los 3 s gira en el sitio, retoma la línea y la sigue. `x` para detenerlo.
4. Aprobado si sigue por la rama nueva. Anotar en el log los ms de `REENGANCHE e=... tras xxx ms`.
5. Repetir con los sensores 2 cm y 4 cm pasados del vértice.

**Prueba 4 - Huecos entre sensores**
1. Carro quieto sobre una hoja blanca. `modo 4`, `go` (los motores no se mueven).
2. Pasar despacio una tira de la misma cinta de la pista por debajo de la barra, de un extremo al
   otro, perpendicular a la barra.
3. Mirar en el log la secuencia de `sens=`. Lo ideal es que nunca aparezca `000000` con la cinta
   debajo, por ejemplo `100000 → 110000 → 010000 → 011000...`.
4. Si aparece `000000` entre dos sensores, hay hueco. El código lo tolera, pero conviene bajar los
   sensores un poco más cerca del piso o ajustar su potenciómetro.

**Prueba 5 - Alimentación y calibración del giro**
1. Poner el carro con el **eje** (no los sensores) sobre una recta de la pista. `modo 5`, `go`.
2. Gira en el sitio sin parar. Dejarlo 30 s y detenerlo con `x`.
3. En el log aparece `T_180 = xxx ms` cada media vuelta. Los valores deben ser parecidos entre sí.
4. Aprobado si no hay ningún `REINICIO: BROWNOUT`. Si aparece, hay que arreglar la alimentación
   (Riesgo 4).
5. Poner el valor típico con `t180 <valor>` y `guardar`. Eso activa la protección contra
   devolverse por la rama vieja.

Qué hacer si alguna falla:
- **Falla la 2:** bajar `aprox` (por ejemplo `aprox 70`, `go` y medir de nuevo).
- **Falla la 5:** arreglar la alimentación (ver Riesgo 4) antes de seguir con el código.
- **Fallan la 1 y la 2 incluso a baja velocidad:** acercar la barra de sensores al eje, de 11 a
  ~6-7 cm. Así se para antes, se gira menos y se retoma la línea más cerca del vértice. A cambio,
  el carro anticipa menos las curvas.

## 7. Datos pendientes

- **Sentido en que se recorre la pista.** Cambia cómo se llega a B y D, los vértices cercanos a la
  curva derecha.
- **Tiempo máximo de vuelta**, si lo hay.
- **Resultados de las pruebas 1 y 2.** Con ellos se sabe si la v10 alcanza o qué hay que cambiar.
