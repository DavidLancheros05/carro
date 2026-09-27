//=====================================================================
//          SEGUIDOR DE LÍNEA v10 - DESDE CERO, PARA V CERRADAS
//                       ESP32 + L298N
//
// Respaldos: "carro" (v8.8) y "carro_simple" (v9.0).
//
// La pista (image.png) es un circuito cerrado con curvas suaves y
// 5 vértices agudos (35° a 60° entre las dos ramas). No tiene cruces
// ni ramas paralelas, así que aquí no hay lógica para eso.
//
//---------------------------------------------------------------------
// LO QUE DICE LA GEOMETRÍA (sensores 11 cm adelante del eje, barra de 9 cm,
// 1.8 cm entre sensores = ancho de la línea; ruedas 15 cm de extremo a extremo)
//---------------------------------------------------------------------
// 1) CÓMO SE VE UNA V AL LLEGAR. La rama de salida arranca en el vértice
//    y se abre con el ángulo θ. A una distancia d antes del vértice está
//    a d·tan(θ) de la línea propia. Con θ=35° entra en la barra unos 5-6 cm
//    antes del vértice: se ve una SEGUNDA MARCA a un costado que se acerca,
//    se funde con la propia y luego TODO SE PONE BLANCO. El lado de esa
//    marca es el lado del giro.
//
// 2) V vs CURVA. En una curva la línea se va por el BORDE (±5). En una V
//    la línea se acaba POR DENTRO de la barra (desaparece en el medio).
//
// 3) GIRAR EN EL SITIO SIEMPRE ENCUENTRA LA RAMA. Al girar sobre el eje, la
//    barra de sensores barre un círculo de 11 cm de radio. Si el giro va
//    hacia el lado correcto, ese círculo corta la rama nueva ANTES que la
//    vieja (~110° de giro con θ=35°), incluso si el carro se pasó un poco
//    del vértice. No hace falta retroceder.
//
// 4) GIRAR AL LADO EQUIVOCADO ES FATAL: se encuentra la rama VIEJA (a ~180°)
//    y el carro se devuelve. Por eso lo principal es decidir bien el lado.
//    Red de seguridad opcional: si se calibra t180, una línea que
//    aparece después de ~180° de giro se toma como la vieja y se ignora.
//
// 5) AL REENGANCHAR, el carro todavía está θ grados corto de alinearse con
//    la rama nueva (la línea se irá corriendo hacia el lado del giro). El
//    PD se encarga de eso; por eso NO se frena el giro al reenganchar.
//
//---------------------------------------------------------------------
// CÓMO FUNCIONA
//---------------------------------------------------------------------
//   - LÍNEA PROPIA = sensores negros a menos de DIST_PROPIA de donde
//     estaba la línea. Con ellos se hace el PD.
//   - EVIDENCIA = sensores negros más lejos que eso. Cada uno suma +1
//     (izquierda) o -1 (derecha) a un contador que se va olvidando solo.
//     Si hay evidencia, se baja la velocidad (viene un vértice).
//   - Si la propia se acaba:
//       con evidencia       -> V: girar en el sitio hacia la evidencia.
//       por el borde        -> curva: girar hacia ese borde.
//       por dentro sin nada -> seguir derecho T_HUECO_MS (la línea puede
//                              estar en el hueco entre dos sensores) y
//                              luego girar hacia el último lado visto.
//   - Durante el giro de V: esperar a ver blanco, y reenganchar cuando la
//     línea nueva, que entra por el lado del giro, llega al centro.
//
// Modos en el log: PROP PIVO HUEC VEE BUSQ
//
//---------------------------------------------------------------------
// CONFIGURACIÓN POR BLUETOOTH (sin cable)
//---------------------------------------------------------------------
//   Conectarse a "SeguidorV88" con una terminal serie y escribir comandos
//   (cada uno termina con Enter):
//     ?            lista de comandos
//     ver          muestra todos los parámetros y su valor
//     <param> <n>  cambia un parámetro al instante. Ej: "aprox 70", "modo 2"
//     guardar      guarda los parámetros: sobreviven al apagar
//     fabrica      vuelve a los valores del código (y borra lo guardado)
//     go           arranca en 3 s (tiempo para soltarlo)
//     stop  (o x)  detiene los motores ya (x no necesita Enter)
//   Al encender el carro espera "go", salvo que "auto" sea 1.
//   Día de carrera: "auto 1", "log 0", "guardar".
//
//---------------------------------------------------------------------
// MODOS DE PRUEBA (parámetro "modo", ver VIABILIDAD.md sección 6)
//---------------------------------------------------------------------
//   0 = carrera normal.
//   1 = VELOCIDAD: avanza derecho a "prectapwm" durante "prectams" ms y
//       se detiene. Medir la distancia recorrida.
//   2 = FRENADO: sigue la línea normal, pero en cuanto decide girar en
//       una V (o buscar) se detiene. Medir cuánto pasaron los sensores
//       del final de la línea.
//   3 = GIRO AISLADO: gira en el sitio hacia "lado" como si estuviera en
//       una V, reengancha y sigue la línea.
//   4 = SENSORES: motores apagados, registra "sens=" cada 50 ms.
//   5 = CALIBRAR: con el EJE sobre una recta, gira en el sitio y
//       registra "T_180 = xxx ms" y el valor a usar ("usar: t180 xxx").
//       Hacerlo con la batería como estará en la carrera. Un t180 menor
//       que lo medido se rechaza: haría ignorar la rama nueva.
//=====================================================================
#include "BluetoothSerial.h"
#include "Preferences.h"
#include "esp_system.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth no habilitado. Herramientas->Partition Scheme: opcion con BT/SPIFFS"
#endif
BluetoothSerial SerialBT;
Preferences     prefs;

//------------- PINES ----------------
const int   NUM_SENSORES = 6;
const byte  PINES_SENSORES[NUM_SENSORES] = {17, 16, 33, 32, 35, 34};
const float PESOS[NUM_SENSORES] = {5, 3, 1, -1, -3, -5};   // + = izquierda

const byte ENA = 25, IN1 = 27, IN2 = 14;   // motor A = rueda DERECHA
const byte ENB = 26, IN3 = 13, IN4 = 4;    // motor B = rueda IZQUIERDA

//=====================================================================
// PARÁMETROS AJUSTABLES POR BLUETOOTH (entre paréntesis, su comando)
// Los valores de aquí son los de fábrica.
//=====================================================================
//------------- EJECUCIÓN ----------------
int MODO_PRUEBA   = 0;   // (modo)  0 = carrera; 1..5 = pruebas
int AUTO_ARRANQUE = 0;   // (auto)  1 = arranca solo 3 s después de encender
int LOG_ACTIVO    = 1;   // (log)   1 = log por Bluetooth, 0 = callado (carrera)

//------------- VELOCIDADES ----------------
int BASE_SPEED       = 110;   // (base)
int VEL_MINIMA_CURVA = 85;    // (velmin)
int VEL_APROX_V      = 80;    // (aprox)   con evidencia de V: llegar despacio
int GIRO_RAPIDO      = 170;   // (girorap) PIVO: línea en el extremo
int GIRO_REVERSA     = -60;   // (girorev)
int SPEED_GIRO       = 105;   // (velgiro) giro en el sitio (V y búsqueda)

//------------- CONTROL PD ----------------
float Kp          = 20.0;     // (kp)
float Kd          = 150.0;    // (kd)
float FRENO_ERROR = 12.0;     // (frenoe) baja la velocidad con error grande
float FRENO_VEL   = 80.0;     // (frenov) baja la velocidad si el error cambia rápido

//------------- LECTURA DE LA PISTA ----------------
// Un sensor a más de esto de la línea propia es OTRA marca (1 sensor = 2).
// La línea enciende hasta 3 sensores (±2 del centro): 3.0 los deja dentro.
float DIST_PROPIA = 3.0f;                   // (distprop)
float EVID_V      = 3.0f;                   // (evidv)     evidencia para decidir el lado
float EVID_APROX  = 1.0f;                   // (evidaprox) evidencia para ir despacio
int   N_FIN_V     = 2;                      // (nfinv)     muestras en blanco = V hecha
unsigned long T_HUECO_MS      = 150;        // (thueco)    sin evidencia: seguir derecho
unsigned long T_SOLO_MARCA_MS = 150;        // (tsolomarca) solo se ve la otra marca

//------------- GIRO ----------------
int   N_BLANCO_GIRO = 5;                    // (nblanco) en la V: ver blanco antes
float REENG_E       = 1.0f;                 // (reeng)   reenganchar con |e| <= esto
unsigned long T_180_MS = 500;               // (t180)    giro de 180°; 0 = sin red
                                            //   medido 482 ms (prueba 5, 2026-09-27) + margen

//------------- PRUEBAS ----------------
unsigned long T_PRUEBA_RECTA_MS = 2000;     // (prectams)  prueba 1: tiempo andando
int           PWM_PRUEBA_RECTA  = 80;       // (prectapwm) prueba 1: 80 = aprox; luego probar 115
int           LADO_PRUEBA       = +1;       // (lado)      prueba 3: +1 izq, -1 der

//------------- FIJOS (no se ajustan por Bluetooth) ----------------
const unsigned long PERIODO_US = 3000;   // 3 ms por muestra
const int   MAX_SPEED       = 170;
const float VEL_MAX         = 0.6;
const float UMBRAL_PIVOTE   = 4.3;
const float UMBRAL_BORDE    = 4.0f;   // la línea se fue por el borde
const int   N_PEGADO       = 100;    // 0.3 s de marca sin parar = sensor pegado
const float UMBRAL_TENDENCIA = 0.02f; // |v| mínima para saber hacia dónde iba la línea
const float EVID_DECAE      = 0.98f;  // por muestra: la evidencia se olvida en ~100 ms
const float FRACCION_VIEJA  = 0.9f;   // línea vista tras 0.9·T_180 = rama vieja
const unsigned long T_ARRANQUE_MS = 3000;   // de "go" a moverse
// Subir PWM de golpe o invertir un motor que aún gira hace caer el
// voltaje y reinicia el ESP32 (brownout).
const int RAMPA_PWM       = 12;   // subida máx. de PWM cada 3 ms
const int PAUSA_INVERSION = 5;    // ciclos en 0 antes de invertir un motor

//------------- TABLA DE PARÁMETROS ----------------
// nombre = comando por Bluetooth y clave en la memoria (máx. 15 letras)
enum TipoParam { P_INT, P_FLOAT, P_ULONG };
struct Param {
  const char* nombre;
  TipoParam   tipo;
  void*       ptr;
  float       fabrica;   // se copia al arrancar, antes de leer la memoria
};
Param params[] = {
  {"modo",       P_INT,   &MODO_PRUEBA,       0},
  {"auto",       P_INT,   &AUTO_ARRANQUE,     0},
  {"log",        P_INT,   &LOG_ACTIVO,        0},
  {"base",       P_INT,   &BASE_SPEED,        0},
  {"velmin",     P_INT,   &VEL_MINIMA_CURVA,  0},
  {"aprox",      P_INT,   &VEL_APROX_V,       0},
  {"girorap",    P_INT,   &GIRO_RAPIDO,       0},
  {"girorev",    P_INT,   &GIRO_REVERSA,      0},
  {"velgiro",    P_INT,   &SPEED_GIRO,        0},
  {"kp",         P_FLOAT, &Kp,                0},
  {"kd",         P_FLOAT, &Kd,                0},
  {"frenoe",     P_FLOAT, &FRENO_ERROR,       0},
  {"frenov",     P_FLOAT, &FRENO_VEL,         0},
  {"distprop",   P_FLOAT, &DIST_PROPIA,       0},
  {"evidv",      P_FLOAT, &EVID_V,            0},
  {"evidaprox",  P_FLOAT, &EVID_APROX,        0},
  {"nfinv",      P_INT,   &N_FIN_V,           0},
  {"thueco",     P_ULONG, &T_HUECO_MS,        0},
  {"tsolomarca", P_ULONG, &T_SOLO_MARCA_MS,   0},
  {"nblanco",    P_INT,   &N_BLANCO_GIRO,     0},
  {"reeng",      P_FLOAT, &REENG_E,           0},
  {"t180",       P_ULONG, &T_180_MS,          0},
  {"prectams",   P_ULONG, &T_PRUEBA_RECTA_MS, 0},
  {"prectapwm",  P_INT,   &PWM_PRUEBA_RECTA,  0},
  {"lado",       P_INT,   &LADO_PRUEBA,       0},
};
const int NUM_PARAMS = sizeof(params) / sizeof(params[0]);

//------------- ESTADO ----------------
const int SEGUIR = 0, GIRO = 1;
int modo = SEGUIR;

bool          corriendo  = false;   // false = motores apagados, esperando "go"
unsigned long arrancarEn = 0;       // != 0: arrancar a esta hora (millis)
unsigned long inicioCorrida = 0;

bool b[NUM_SENSORES], bAnterior[NUM_SENSORES];
unsigned long ultimaMuestraUs = 0;

float ultimoError    = 0;
float velocidadError = 0;
float hist[5];
int   histIdx = 0, histLlenas = 0;

float         evidencia       = 0;   // + = hay marcas a la izquierda, - = derecha
unsigned long ultimaVezPropia = 0;
int           negroSeguido[NUM_SENSORES];   // muestras seguidas viendo marca
bool          lineaVista      = false;   // se vio la línea desde el arranque
int           muestrasBlanco  = 0;

int           ladoGiro        = 0;   // +1 izquierda, -1 derecha
bool          giroEsV         = false;
unsigned long inicioGiro      = 0;
int           blancoGiro      = 0;
bool          viendoLinea     = false;
bool          ignorandoLinea  = false;
bool          viejaSaltada    = false;

int           calibBlanco     = 0;   // prueba 5
unsigned long calibAnterior   = 0;
unsigned long calibPasada     = 0;   // duración de la pasada anterior
unsigned long calibSuma       = 0;
int           calibN          = 0;
unsigned long calibPromedio   = 0;   // T_180 medido; 0 = sin calibrar (se pierde al apagar)

int velA = 0, velB = 0;              // PWM actual (rueda derecha / izquierda)
int pausaA = 0, pausaB = 0;
const char* telModo = "----";

//=====================================================================
// LOG: toda línea empieza con "sens=XXXXXX"; los eventos se pegan al final
//=====================================================================
bool lineaAbierta = false;

void textoSensores(char *s) {
  for (int i = 0; i < NUM_SENSORES; i++) s[i] = b[i] ? '1' : '0';
  s[NUM_SENSORES] = '\0';
}

void cerrarLinea() {
  if (lineaAbierta) SerialBT.print("\r\n");
  lineaAbierta = false;
}

void logEvento(const char* fmt, ...) {
  if (!LOG_ACTIVO) return;
  char msg[140];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  char linea[180];
  if (lineaAbierta) {
    snprintf(linea, sizeof(linea), " | %s", msg);
  } else {
    char sens[NUM_SENSORES + 1];
    textoSensores(sens);
    snprintf(linea, sizeof(linea), "sens=%s | %s", sens, msg);
    lineaAbierta = true;
  }
  SerialBT.print(linea);
}

// Respuesta a un comando: se muestra siempre, aunque el log esté apagado
void responder(const char* fmt, ...) {
  char msg[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  cerrarLinea();
  SerialBT.print(msg);
  SerialBT.print("\r\n");
  Serial.println(msg);   // también por USB, para diagnosticar con el cable
}

void logEstado(float e) {
  if (!LOG_ACTIVO) return;
  static unsigned long tLog = 0;
  if (millis() - tLog < 50) return;
  tLog = millis();

  char sens[NUM_SENSORES + 1];
  textoSensores(sens);
  const char* dir = (velB > velA) ? "->DER" : (velA > velB) ? "->IZQ" : "recto";

  cerrarLinea();
  char linea[180];
  snprintf(linea, sizeof(linea),
           "sens=%s e=%.2f v=%.3f ev=%+.1f | modo=%s IZQ=%+d DER=%+d %s",
           sens, e, velocidadError, evidencia, telModo, velB, velA, dir);
  SerialBT.print(linea);
  lineaAbierta = true;
}

//=====================================================================
// MOTORES (con rampa y pausa antes de invertir)
//=====================================================================
int rampa(int actual, int objetivo, int &pausa) {
  if (pausa > 0) { pausa--; return 0; }
  bool invierte = (actual > 0 && objetivo < 0) || (actual < 0 && objetivo > 0);
  if (invierte) objetivo = 0;                 // primero frenar
  int nuevo;
  if (abs(objetivo) <= abs(actual))
    nuevo = objetivo;                         // bajar no pide corriente: inmediato
  else
    nuevo = actual + constrain(objetivo - actual, -RAMPA_PWM, RAMPA_PWM);
  if (invierte && nuevo == 0) pausa = PAUSA_INVERSION;
  return nuevo;
}

void motores(int derecha, int izquierda) {
  velA = rampa(velA, constrain(derecha,   -MAX_SPEED, MAX_SPEED), pausaA);
  velB = rampa(velB, constrain(izquierda, -MAX_SPEED, MAX_SPEED), pausaB);

  analogWrite(ENA, abs(velA));
  digitalWrite(IN1, velA >= 0 ? HIGH : LOW);
  digitalWrite(IN2, velA >= 0 ? LOW  : HIGH);

  analogWrite(ENB, abs(velB));
  digitalWrite(IN3, velB >= 0 ? LOW  : HIGH);
  digitalWrite(IN4, velB >= 0 ? HIGH : LOW);
}

// lado > 0 = girar a la IZQUIERDA en el sitio
void pivotarHacia(int lado, int velocidad) {
  if (lado > 0) motores(velocidad, -velocidad);
  else          motores(-velocidad, velocidad);
}

//=====================================================================
// CONTROL PD
//=====================================================================
void registrarError(float e) {
  hist[histIdx] = e;
  histIdx = (histIdx + 1) % 5;
  if (histLlenas < 5) histLlenas++;
  if (histLlenas == 5) {
    float velCruda = (e - hist[histIdx]) / 4.0f;   // vs. la muestra más vieja
    velocidadError = 0.7f * velocidadError + 0.3f * velCruda;
    velocidadError = constrain(velocidadError, -VEL_MAX, VEL_MAX);
  }
}

void resetVelocidad() {
  histIdx = histLlenas = 0;
  velocidadError = 0;
}

void aplicarControl(float e, const char* nombreModo) {
  if (e > UMBRAL_PIVOTE) {
    motores(GIRO_RAPIDO, GIRO_REVERSA);
    telModo = nombreModo ? nombreModo : "PIVO";
    return;
  }
  if (e < -UMBRAL_PIVOTE) {
    motores(GIRO_REVERSA, GIRO_RAPIDO);
    telModo = nombreModo ? nombreModo : "PIVO";
    return;
  }
  float exceso = max(fabs(e) - 1.0f, 0.0f);
  int velBase = BASE_SPEED - (int)(FRENO_ERROR * exceso)
                           - (int)(FRENO_VEL * fabs(velocidadError));
  velBase = max(velBase, VEL_MINIMA_CURVA);
  if (fabs(evidencia) >= EVID_APROX) velBase = min(velBase, VEL_APROX_V);

  int correccion = (int)(Kp * e + Kd * velocidadError);
  motores(velBase + correccion, velBase - correccion);
  telModo = nombreModo ? nombreModo : "PROP";
}

//=====================================================================
// SENSORES (antirrebote: dos lecturas seguidas iguales)
//=====================================================================
void leerSensores() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    bool lectura = (digitalRead(PINES_SENSORES[i]) == HIGH);
    b[i] = lectura && bAnterior[i];
    bAnterior[i] = lectura;
  }
}

//=====================================================================
// ARRANCAR / DETENER
//=====================================================================
void detener(const char* motivo) {
  corriendo  = false;
  arrancarEn = 0;
  motores(0, 0);
  responder("DETENIDO: %s", motivo);
}

void empezarGiro(int lado, bool esV, const char* motivo);

void iniciarCorrida() {
  corriendo       = true;
  arrancarEn      = 0;
  inicioCorrida   = millis();
  modo            = SEGUIR;
  ultimoError     = 0;
  evidencia       = 0;
  muestrasBlanco  = 0;
  ultimaVezPropia = millis();
  lineaVista      = false;
  for (int i = 0; i < NUM_SENSORES; i++) negroSeguido[i] = 0;
  calibBlanco     = 0;
  calibAnterior   = 0;
  calibPasada     = 0;
  if (MODO_PRUEBA == 5) { calibSuma = 0; calibN = 0; calibPromedio = 0; }
  resetVelocidad();
  responder("ARRANCA (modo %d)", MODO_PRUEBA);
  bool hay = false;
  for (int i = 0; i < NUM_SENSORES; i++) if (b[i]) hay = true;
  if (!hay && (MODO_PRUEBA == 0 || MODO_PRUEBA == 2))
    responder("OJO: ningun sensor ve la linea (sens=000000). Los sensores van SOBRE la linea");
  if (MODO_PRUEBA == 3) empezarGiro(LADO_PRUEBA >= 0 ? +1 : -1, true, "PRUEBA GIRO AISLADO");
}

//=====================================================================
// TRANSICIONES
//=====================================================================
void empezarGiro(int lado, bool esV, const char* motivo) {
  modo           = GIRO;
  ladoGiro       = lado;
  giroEsV        = esV;
  inicioGiro     = millis();
  blancoGiro     = 0;
  viendoLinea    = false;
  ignorandoLinea = false;
  viejaSaltada   = false;
  resetVelocidad();
  logEvento("%s %s lado=%+d ev=%+.1f", esV ? "GIRO V" : "BUSCAR", motivo, lado, evidencia);
  if (MODO_PRUEBA == 2) {           // prueba de frenado: quedarse donde decidió girar
    detener("PRUEBA FRENADO, medir la distancia");
    return;
  }
  pivotarHacia(lado, SPEED_GIRO);
}

void reenganchar(float p) {
  unsigned long t = millis() - inicioGiro;
  modo            = SEGUIR;
  ultimoError     = p;
  evidencia       = 0;
  ultimaVezPropia = millis();
  muestrasBlanco  = 0;
  resetVelocidad();
  registrarError(p);
  aplicarControl(p, nullptr);
  logEvento("REENGANCHE e=%.2f tras %lu ms", p, t);
}

//=====================================================================
// MODO SEGUIR
//=====================================================================
void seguir() {
  unsigned long ms = millis();

  // Separar sensores negros en: línea propia / otras marcas (evidencia)
  float ref = ultimoError;
  float sumaPropia = 0;
  int   nPropia = 0, nNegros = 0;
  float ev = 0;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (!b[i]) { negroSeguido[i] = 0; continue; }
    nNegros++;
    float d = PESOS[i] - ref;
    if (fabs(d) <= DIST_PROPIA) { sumaPropia += PESOS[i]; nPropia++; negroSeguido[i] = 0; continue; }
    // Marca al costado. La rama de una V pasa por un sensor en < 0.1 s; un
    // sensor negro sin parar es otra cosa (potenciómetro muy sensible, piso
    // oscuro): no cuenta, y lo que ya sumó se borra.
    if (negroSeguido[i] < N_PEGADO) negroSeguido[i]++;
    if (negroSeguido[i] == N_PEGADO - 1) {
      evidencia = 0;
      logEvento("SENSOR %d PEGADO EN NEGRO: no cuenta como marca", i + 1);
    }
    if (negroSeguido[i] >= N_PEGADO - 1) continue;
    ev += (d > 0) ? 1.0f : -1.0f;
  }
  evidencia = evidencia * EVID_DECAE + ev;
  muestrasBlanco = (nNegros == 0) ? muestrasBlanco + 1 : 0;

  // --- Se ve la propia: PD normal ---
  if (nPropia > 0) {
    lineaVista = true;
    float e = sumaPropia / nPropia;
    ultimoError     = e;
    ultimaVezPropia = ms;
    registrarError(e);
    aplicarControl(e, nullptr);
    logEstado(e);
    return;
  }

  // --- La propia no se ve ---
  unsigned long sinPropia = ms - ultimaVezPropia;
  int ladoEvid = (evidencia >= 0) ? +1 : -1;

  // V: hubo otra marca y ahora se acabó todo (o solo queda la otra marca)
  if (fabs(evidencia) >= EVID_V &&
      (muestrasBlanco >= N_FIN_V || sinPropia >= T_SOLO_MARCA_MS)) {
    empezarGiro(ladoEvid, true, "linea acabada con marca");
    return;
  }

  // Curva: la línea se fue por el borde
  if (fabs(ultimoError) >= UMBRAL_BORDE && nNegros == 0) {
    empezarGiro(ultimoError > 0 ? +1 : -1, false, "salio por el borde");
    return;
  }

  // Se perdió por dentro: está en el hueco entre dos sensores (la prueba 4
  // mostró que cada sensor ve un punto angosto: pasa en cada cambio de
  // sensor). Está al lado del último sensor, hacia donde se venía moviendo.
  if (sinPropia < T_HUECO_MS) {
    float e;
    if      (velocidadError >  UMBRAL_TENDENCIA) e = ultimoError + 1.0f;
    else if (velocidadError < -UMBRAL_TENDENCIA) e = ultimoError - 1.0f;
    // Sin tendencia: corregir hacia el último sensor visto, nunca derecho.
    // Así la línea sale del hueco y cae en un sensor; si se fuera derecho
    // podría quedarse en el hueco más de T_HUECO_MS y parecer una V.
    else    e = ultimoError;
    e = constrain(e, -UMBRAL_BORDE, UMBRAL_BORDE);
    aplicarControl(e, "HUEC");
    logEstado(e);
    return;
  }

  // Nunca se vio la línea desde el arranque: no es una V, el carro se
  // puso mal (los sensores van 11 cm delante del eje, sobre la línea).
  if (!lineaVista) {
    detener("no veo la linea desde el arranque: poner los SENSORES sobre ella");
    return;
  }

  // Se acabó sin aviso: V sin marca previa. Mejor apuesta: el lado de la
  // poca evidencia que haya, o el último lado donde se vio la línea.
  int lado = (fabs(evidencia) > 0.3f) ? ladoEvid : (ultimoError >= 0 ? +1 : -1);
  empezarGiro(lado, true, "linea acabada sin marca");
}

//=====================================================================
// MODO GIRO (en el sitio): V o búsqueda
//=====================================================================
void girar() {
  unsigned long t = millis() - inicioGiro;
  int   nNegros = 0;
  float suma = 0;
  for (int i = 0; i < NUM_SENSORES; i++)
    if (b[i]) { nNegros++; suma += PESOS[i]; }

  int blancoRequerido = giroEsV ? N_BLANCO_GIRO : 0;

  if (nNegros == 0) {
    blancoGiro++;
    viendoLinea    = false;
    ignorandoLinea = false;
  } else if (blancoGiro >= blancoRequerido) {
    float p = suma / nNegros;
    if (!viendoLinea) {          // empieza a entrar una línea a la barra
      viendoLinea = true;
      if (giroEsV && T_180_MS > 0 && !viejaSaltada &&
          t > (unsigned long)(FRACCION_VIEJA * T_180_MS)) {
        viejaSaltada   = true;
        ignorandoLinea = true;
        logEvento("IGNORA RAMA VIEJA tras %lu ms", t);
      }
    }
    // La línea entra por el lado del giro y viaja al centro: tomarla ahí
    if (!ignorandoLinea && p * ladoGiro <= REENG_E) {
      reenganchar(p);
      return;
    }
  }

  pivotarHacia(ladoGiro, SPEED_GIRO);
  telModo = giroEsV ? "VEE" : "BUSQ";
  logEstado(5.0f * ladoGiro);
}

//=====================================================================
// PRUEBAS
//=====================================================================
// 1: avanza derecho T_PRUEBA_RECTA_MS y se detiene.
void pruebaVelocidad() {
  if (millis() - inicioCorrida < T_PRUEBA_RECTA_MS) {
    motores(PWM_PRUEBA_RECTA, PWM_PRUEBA_RECTA);
    return;
  }
  char msg[80];
  snprintf(msg, sizeof(msg), "PRUEBA VELOCIDAD tras %lu ms a PWM %d, medir la distancia",
           T_PRUEBA_RECTA_MS, PWM_PRUEBA_RECTA);
  detener(msg);
}

// 4: motores apagados, muestra lo que ven los sensores. Solo cuando
// cambian (o cada 1 s): mandar menos datos por Bluetooth, porque mientras
// el carro envía mucho se han perdido comandos.
void pruebaSensores() {
  static unsigned long tLog = 0;
  static char anterior[NUM_SENSORES + 1] = "";
  motores(0, 0);
  char sens[NUM_SENSORES + 1];
  textoSensores(sens);
  bool cambio = strcmp(sens, anterior) != 0;
  if (!cambio && millis() - tLog < 1000) return;
  if (cambio && millis() - tLog < 20) return;
  tLog = millis();
  strcpy(anterior, sens);
  responder("sens=%s", sens);
}

// 5: gira en el sitio y mide el tiempo entre dos pasadas por la recta
// (cada pasada = 180° de giro). Si el eje no está justo sobre la línea,
// las pasadas se alternan largo/corto, pero cada par suma una vuelta: por
// eso T_180 = promedio de las dos últimas pasadas.
void calibrar() {
  pivotarHacia(+1, SPEED_GIRO);

  bool hayLinea = false;
  for (int i = 0; i < NUM_SENSORES; i++) if (b[i]) hayLinea = true;
  if (!hayLinea) { calibBlanco++; return; }

  if (calibBlanco >= 20) {         // la línea vuelve a entrar tras 60 ms de blanco
    unsigned long ms = millis();
    if (calibAnterior != 0) {
      unsigned long pasada = ms - calibAnterior;
      if (calibPasada != 0) {
        unsigned long t180 = (calibPasada + pasada) / 2;
        calibSuma += t180;
        calibN++;
        calibPromedio = calibSuma / calibN;
        // Un poco más que lo medido: si la batería baja, gira más lento
        unsigned long sugerido = ((unsigned long)(calibPromedio * 1.04f) + 5) / 10 * 10;
        responder("pasada %lu ms | T_180 = %lu ms | promedio %lu (%d) -> usar: t180 %lu",
                  pasada, t180, calibPromedio, calibN, sugerido);
      } else {
        responder("pasada %lu ms", pasada);
      }
      calibPasada = pasada;
    }
    calibAnterior = ms;
  }
  calibBlanco = 0;
}

//=====================================================================
// PARÁMETROS: mostrar, cambiar, guardar
//=====================================================================
float leerParam(const Param &p) {
  switch (p.tipo) {
    case P_INT:   return *(int*)p.ptr;
    case P_FLOAT: return *(float*)p.ptr;
    default:      return *(unsigned long*)p.ptr;
  }
}

void escribirParam(const Param &p, float v) {
  switch (p.tipo) {
    case P_INT:   *(int*)p.ptr = (int)lroundf(v); break;
    case P_FLOAT: *(float*)p.ptr = v; break;
    default:      *(unsigned long*)p.ptr = (unsigned long)max(0L, lroundf(v)); break;
  }
}

void mostrarParam(const Param &p) {
  if (p.tipo == P_FLOAT) responder("  %-11s %.2f", p.nombre, leerParam(p));
  else                   responder("  %-11s %ld",  p.nombre, (long)leerParam(p));
}

void mostrarTodo() {
  responder("--- parametros (%s) ---", corriendo ? "corriendo" : "detenido");
  for (int i = 0; i < NUM_PARAMS; i++) mostrarParam(params[i]);
}

void cargarGuardados() {
  prefs.begin("seguidor", true);
  // Si un parámetro nunca se guardó, getFloat devuelve el valor actual
  for (int i = 0; i < NUM_PARAMS; i++)
    escribirParam(params[i], prefs.getFloat(params[i].nombre, leerParam(params[i])));
  prefs.end();
}

void guardarTodo() {
  prefs.begin("seguidor", false);
  for (int i = 0; i < NUM_PARAMS; i++)
    prefs.putFloat(params[i].nombre, leerParam(params[i]));
  prefs.end();
  responder("GUARDADO: se mantiene al apagar");
}

void volverAFabrica() {
  prefs.begin("seguidor", false);
  prefs.clear();
  prefs.end();
  for (int i = 0; i < NUM_PARAMS; i++) escribirParam(params[i], params[i].fabrica);
  responder("VALORES DE FABRICA (lo guardado se borro)");
}

void ayuda() {
  responder("Comandos:");
  responder("  ver           muestra los parametros");
  responder("  <param> <n>   cambia un parametro. Ej: aprox 70 | modo 2");
  responder("  guardar       guarda (se mantiene al apagar)");
  responder("  fabrica       vuelve a los valores del codigo");
  responder("  go            arranca en 3 s");
  responder("  stop / x      detiene ya");
  responder("Modos: 0 carrera, 1 velocidad, 2 frenado, 3 giro, 4 sensores, 5 calibrar");
}

//=====================================================================
// COMANDOS POR BLUETOOTH
//=====================================================================
int buscarParam(const char* nombre) {
  for (int i = 0; i < NUM_PARAMS; i++)
    if (!strcasecmp(nombre, params[i].nombre)) return i;
  return -1;
}

void ejecutarComando(char* linea) {
  char* cmd = strtok(linea, " \t=");
  if (!cmd) return;
  char* arg = strtok(nullptr, " \t=");

  if (!strcasecmp(cmd, "x") || !strcasecmp(cmd, "stop")) { detener("comando stop"); return; }
  if (!strcasecmp(cmd, "go")) {
    if (corriendo) { responder("Ya esta corriendo (stop para detener)"); return; }
    arrancarEn = millis() + T_ARRANQUE_MS;
    responder("Arranca en %lu s, modo %d...", T_ARRANQUE_MS / 1000, MODO_PRUEBA);
    return;
  }
  if (!strcasecmp(cmd, "?") || !strcasecmp(cmd, "ayuda")) { ayuda(); return; }
  if (!strcasecmp(cmd, "ver"))     { mostrarTodo();    return; }
  if (!strcasecmp(cmd, "guardar")) { guardarTodo();    return; }
  if (!strcasecmp(cmd, "fabrica")) { volverAFabrica(); return; }

  int i = buscarParam(cmd);
  // "modo5" sin espacio = "modo 5" (sin romper nombres con números como "t180")
  if (i < 0 && !arg) {
    char* num = cmd;
    while (*num && !isdigit((unsigned char)*num) && *num != '-' && *num != '.') num++;
    if (num != cmd && *num) {
      char nombre[16];
      int largo = min((int)(num - cmd), (int)sizeof(nombre) - 1);
      memcpy(nombre, cmd, largo);
      nombre[largo] = '\0';
      i = buscarParam(nombre);
      if (i >= 0) arg = num;
    }
  }
  if (i < 0) { responder("No entiendo \"%s\". Escribe ? para ver los comandos", cmd); return; }

  Param &p = params[i];
  if (!arg) { mostrarParam(p); return; }
  char* fin;
  float v = strtof(arg, &fin);
  if (fin == arg) { responder("Valor no valido: %s", arg); return; }
  if (!strcasecmp(p.nombre, "modo") && (v < 0 || v > 5)) { responder("modo va de 0 a 5"); return; }
  if (!strcasecmp(p.nombre, "t180") && v > 0) {
    // Un t180 menor que el real hace ignorar la rama NUEVA y devolverse
    if (calibPromedio > 0 && v < calibPromedio) {
      responder("t180 %.0f es menor que lo medido (%lu ms): ignoraria la rama nueva.", v, calibPromedio);
      responder("  Usa t180 %lu o mas. No se cambio.", calibPromedio);
      return;
    }
    if (calibPromedio == 0)
      responder("  OJO: sin calibrar en este encendido. Usar el valor 'usar: t180' de la prueba 5");
  }
  escribirParam(p, v);
  mostrarParam(p);
  responder("  (activo ya; 'guardar' para que quede al apagar)");
}

void leerComandos() {
  static char buf[64];
  static int  n = 0;
  static bool saltarLinea = false;
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = '\0'; ejecutarComando(buf); n = 0; }
      saltarLinea = false;
    } else if (saltarLinea && c != 'x' && c != 'X') {
      // resto de una línea que empezó con x: se descarta
    } else if ((n == 0 || saltarLinea) && (c == 'x' || c == 'X')) {
      // x al inicio de línea detiene YA, sin esperar el Enter (se han
      // perdido comandos mientras el carro envía el log)
      detener("comando stop");
      saltarLinea = true;
    } else if (n < (int)sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

//=====================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[USB] arrancando...");
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(PINES_SENSORES[i], INPUT);
    bAnterior[i] = false;
  }
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motores(0, 0);

  // Valores de fábrica = los del código; luego se aplican los guardados
  for (int i = 0; i < NUM_PARAMS; i++) params[i].fabrica = leerParam(params[i]);
  cargarGuardados();

  Serial.println("[USB] encendiendo Bluetooth...");
  bool btOk = SerialBT.begin("SeguidorV88");   // mismo nombre: el PC ya lo tiene emparejado
  Serial.println(btOk ? "[USB] Bluetooth OK: buscar 'SeguidorV88'"
                      : "[USB] ERROR: no se pudo encender el Bluetooth");
  responder("=== SEGUIDOR DE LINEA - v10 (V cerradas) ===");
  const char* motivo;
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  motivo = "ENCENDIDO"; break;
    case ESP_RST_BROWNOUT: motivo = "BROWNOUT (bajo voltaje!)"; break;
    case ESP_RST_PANIC:    motivo = "PANIC (error de programa)"; break;
    case ESP_RST_EXT:      motivo = "BOTON RESET"; break;
    default:               motivo = "OTRO"; break;
  }
  responder("REINICIO: %s", motivo);
  responder("modo=%d auto=%d log=%d  (? = comandos)", MODO_PRUEBA, AUTO_ARRANQUE, LOG_ACTIVO);

  if (AUTO_ARRANQUE) {
    arrancarEn = millis() + T_ARRANQUE_MS;
    responder("Arranque automatico en %lu s", T_ARRANQUE_MS / 1000);
  } else {
    responder("Esperando 'go'");
  }
}

void loop() {
  leerComandos();

  unsigned long ahora = micros();
  if (ahora - ultimaMuestraUs < PERIODO_US) return;
  ultimaMuestraUs = ahora;

  leerSensores();

  if (arrancarEn != 0 && (long)(millis() - arrancarEn) >= 0) iniciarCorrida();
  if (!corriendo) { motores(0, 0); return; }

  switch (MODO_PRUEBA) {
    case 1:  pruebaVelocidad(); break;
    case 4:  pruebaSensores();  break;
    case 5:  calibrar();        break;
    default:                     // 0 carrera, 2 frenado, 3 giro aislado
      if (modo == GIRO) girar();
      else              seguir();
  }
}
