//=====================================================================
//              SEGUIDOR DE LÍNEA - VERSIÓN SIMPLE v9.0
//                       ESP32 + L298N
//
// Reescritura simple de la v8.8 (que quedó como respaldo en "carro").
// Pocas ideas, en este orden:
//
//   1) SEGUIR:  PD sobre la línea propia = el grupo de sensores más
//               cercano a donde se vio la línea la última vez.
//   2) MARCA:   cualquier otra marca al costado se anota (de qué lado y
//               cuándo). Mientras haya marca reciente se va despacio,
//               porque puede venir el vértice de una V.
//   3) HUECO:   si la propia deja de verse, se mantiene el rumbo
//               T_HUECO_MS (puede estar en el hueco entre dos sensores).
//   4) GIRO V:  si sigue sin verse y hubo marca reciente, la línea se
//               acabó en una V: girar en el sitio hacia el lado de la marca.
//   5) BUSCAR:  si no hubo marca, girar hacia donde se vio la línea por
//               última vez.
//   6) CRUCE:   4+ sensores seguidos CENTRADOS -> seguir derecho.
//               4+ sensores seguidos DESCENTRADOS -> V: girar hacia ese lado.
//
// Hardware:
//   - 6 sensores IR digitales, HIGH = negro, 11 cm adelante del eje.
//   - L298N: motor A = rueda DERECHA, motor B = rueda IZQUIERDA.
//
// Log (Bluetooth "SeguidorV88"): toda línea empieza con "sens=XXXXXX";
// los eventos se pegan al final de la línea de estado.
//   Modos: PROP PIVO HUEC CRUZ VEE BUSQ
//=====================================================================
#include "BluetoothSerial.h"
#include "esp_system.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth no habilitado. Herramientas->Partition Scheme: opcion con BT/SPIFFS"
#endif
BluetoothSerial SerialBT;

#define DEBUG 1   // 1 = log por Bluetooth, 0 = apagado (carrera)

//------------- PINES ----------------
const int   NUM_SENSORES = 6;
const byte  PINES_SENSORES[NUM_SENSORES] = {17, 16, 33, 32, 35, 34};
const float PESOS[NUM_SENSORES] = {5, 3, 1, -1, -3, -5};   // + = izquierda

const byte ENA = 25, IN1 = 27, IN2 = 14;   // motor A = rueda DERECHA
const byte ENB = 26, IN3 = 13, IN4 = 4;    // motor B = rueda IZQUIERDA

//------------- VELOCIDADES ----------------
const int BASE_SPEED       = 115;
const int MAX_SPEED        = 170;
const int VEL_MINIMA_CURVA = 90;
const int VEL_APROX_V      = 80;    // con marca al costado: llegar despacio a la V
const int GIRO_RAPIDO      = 170;   // pivote con la línea en el extremo
const int GIRO_REVERSA     = -60;
const int SPEED_VEE        = 105;   // giro en el sitio en la V
const int SPEED_BUSQUEDA   = 100;

//------------- CONTROL PD ----------------
const unsigned long PERIODO_US = 3000;   // 3 ms por muestra
float Kp = 20.0;
float Kd = 150.0;
const float FRENO_ERROR      = 12.0;  // baja la velocidad con error grande
const float FRENO_VEL        = 80.0;  // baja la velocidad si el error cambia rápido
const float VEL_MAX          = 0.6;
const float UMBRAL_PIVOTE    = 4.3;
const float UMBRAL_SALTO     = 2.5;   // más lejos que esto de la propia = otra marca
const float AMORT_REENGANCHE = 0.6f;  // frena el giro al reencontrar la línea

//------------- TIEMPOS Y UMBRALES ----------------
const unsigned long T_HUECO_MS    = 250;   // mantener rumbo sin ver la línea
const unsigned long T_HUECO_V_MS  = 60;    // ...pero si hubo marca al costado (V), girar casi ya
const unsigned long T_MARCA_MS    = 1000;  // cuánto dura una marca "reciente"
const int           N_MARCA       = 3;     // muestras seguidas para aceptar una marca
const unsigned long T_CRUCE_MS    = 400;   // máximo cruzando un cruce
const int           N_ANCHO       = 2;     // muestras seguidas de banda ancha
const unsigned long T_GIRO_MIN_MS = 60;    // giro mínimo en la V
const unsigned long T_GIRO_MAX_MS = 2200;  // si no aparece la rama -> buscar
const float TOL_REENGANCHE        = 3.0f;  // |pos| para dar la línea por encontrada
const float UMBRAL_CENTRO_CRUCE   = 1.8f;  // banda ancha centrada = cruce

//------------- LÍMITE DE CORRIENTE ----------------
// Subir PWM de golpe o invertir un motor que aún gira hace caer el
// voltaje y reinicia el ESP32 (brownout).
const int RAMPA_PWM       = 12;   // subida máx. de PWM cada 3 ms
const int PAUSA_INVERSION = 5;    // ciclos en 0 antes de invertir un motor

//------------- ESTADO ----------------
const int SEGUIR = 0, GIRO_V = 1, BUSCAR = 2;
int modo = SEGUIR;

bool  b[NUM_SENSORES], bAnterior[NUM_SENSORES];
unsigned long ultimaMuestraUs = 0;

float ultimoError    = 0;
float velocidadError = 0;
float hist[5];
int   histIdx = 0, histLlenas = 0;

unsigned long ultimaVezLinea = 0;   // última vez que se vio la propia
int           ladoMarca = 0;        // +1 = marca a la izquierda, -1 derecha
unsigned long tMarca    = 0;
int           contMarca = 0;
int           contAncho = 0;
bool          enCruce   = false;
unsigned long inicioCruce = 0;

int           ladoGiro   = 0;       // GIRO_V y BUSCAR: +1 izquierda, -1 derecha
unsigned long inicioGiro = 0;

int velA = 0, velB = 0;             // PWM actual (rueda derecha / izquierda)
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
#if DEBUG
  if (lineaAbierta) SerialBT.print("\r\n");
  lineaAbierta = false;
#endif
}

void logEvento(const char* fmt, ...) {
#if DEBUG
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
#endif
}

bool marcaReciente() {
  return ladoMarca != 0 && millis() - tMarca < T_MARCA_MS;
}

void logEstado(float e) {
#if DEBUG
  static unsigned long tLog = 0;
  if (millis() - tLog < 50) return;
  tLog = millis();

  char sens[NUM_SENSORES + 1];
  textoSensores(sens);
  const char* dir = (velB > velA) ? "->DER" : (velA > velB) ? "->IZQ" : "recto";

  cerrarLinea();
  char linea[180];
  snprintf(linea, sizeof(linea),
           "sens=%s e=%.2f v=%.3f marca=%+d | modo=%s IZQ=%+d DER=%+d %s",
           sens, e, velocidadError, marcaReciente() ? ladoMarca : 0,
           telModo, velB, velA, dir);
  SerialBT.print(linea);
  lineaAbierta = true;
#endif
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
  if (marcaReciente()) velBase = min(velBase, VEL_APROX_V);

  int correccion = (int)(Kp * e + Kd * velocidadError);
  motores(velBase + correccion, velBase - correccion);
  telModo = nombreModo ? nombreModo : "PROP";
}

//=====================================================================
// SENSORES: lectura con antirrebote y agrupación
//=====================================================================
void leerSensores() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    bool lectura = (digitalRead(PINES_SENSORES[i]) == HIGH);
    b[i] = lectura && bAnterior[i];
    bAnterior[i] = lectura;
  }
}

// Grupos de sensores negros contiguos: posición (centroide) y ancho.
int leerGrupos(float pos[3], int ancho[3]) {
  int n = 0, i = 0;
  while (i < NUM_SENSORES && n < 3) {
    if (!b[i]) { i++; continue; }
    float suma = 0; int cant = 0;
    while (i < NUM_SENSORES && b[i]) { suma += PESOS[i]; cant++; i++; }
    pos[n] = suma / cant;
    ancho[n] = cant;
    n++;
  }
  return n;
}

//=====================================================================
// TRANSICIONES
//=====================================================================
void empezarGiro(int lado, const char* motivo) {
  modo = GIRO_V;
  ladoGiro = lado;
  inicioGiro = millis();
  ladoMarca = 0;
  resetVelocidad();
  logEvento("GIRO V %s lado=%+d", motivo, lado);
}

void empezarBusqueda(int lado, const char* motivo) {
  modo = BUSCAR;
  ladoGiro = lado;
  inicioGiro = millis();
  resetVelocidad();
  logEvento("BUSCAR %s lado=%+d", motivo, lado);
}

void reenganchar(float p, const char* motivo) {
  unsigned long t = millis() - inicioGiro;
  modo = SEGUIR;
  ultimoError = p;
  ultimaVezLinea = millis();
  ladoMarca = 0;
  enCruce = false;
  contAncho = contMarca = 0;
  resetVelocidad();
  registrarError(p);
  velocidadError = -AMORT_REENGANCHE * ladoGiro;   // frena el giro
  aplicarControl(p, nullptr);
  logEvento("%s e=%.2f tras %lu ms", motivo, p, t);
}

// Línea angosta cerca del centro (y no del lado contrario al giro)
int buscarLinea(float pos[], int ancho[], int n, int lado) {
  int mejor = -1;
  for (int i = 0; i < n; i++) {
    if (ancho[i] >= 4) continue;
    if (fabs(pos[i]) > TOL_REENGANCHE) continue;
    if (pos[i] * lado < -1.0f) continue;
    if (mejor < 0 || fabs(pos[i]) < fabs(pos[mejor])) mejor = i;
  }
  return mejor;
}

//=====================================================================
// MODOS
//=====================================================================
void seguir(float pos[], int ancho[], int n) {
  unsigned long ms = millis();

  // --- 1) Banda ancha (4+ sensores): cruce o V ---
  int iAncho = -1;
  for (int i = 0; i < n; i++) if (ancho[i] >= 4) iAncho = i;
  contAncho = (iAncho >= 0) ? contAncho + 1 : 0;

  if (iAncho >= 0 && contAncho >= N_ANCHO) {
    float c = pos[iAncho];
    if (fabs(c) > UMBRAL_CENTRO_CRUCE) {           // descentrada: V
      empezarGiro(c > 0 ? +1 : -1, "banda ancha");
      return;
    }
    if (!enCruce) { enCruce = true; inicioCruce = ms; logEvento("CRUCE"); }
    if (ms - inicioCruce < T_CRUCE_MS) {           // centrada: seguir derecho
      ultimaVezLinea = ms;
      float e = constrain(ultimoError, -2.0f, 2.0f);
      aplicarControl(e, "CRUZ");
      logEstado(e);
      return;
    }
    empezarBusqueda(ultimoError >= 0 ? +1 : -1, "cruce muy largo");
    return;
  }
  if (iAncho < 0) enCruce = false;

  // --- 2) Línea propia: el grupo angosto más cercano al último error ---
  int iPropia = -1;
  float dist = 1e9;
  for (int i = 0; i < n; i++) {
    if (ancho[i] >= 4) continue;
    float d = fabs(pos[i] - ultimoError);
    if (d < dist) { dist = d; iPropia = i; }
  }
  if (dist > UMBRAL_SALTO) iPropia = -1;           // lo que se ve está lejos: es otra marca

  // --- 3) Otras marcas al costado: anotar de qué lado están ---
  float ref = (iPropia >= 0) ? pos[iPropia] : ultimoError;
  int ladoVisto = 0;
  for (int i = 0; i < n; i++) {
    if (i == iPropia || ancho[i] >= 4) continue;
    ladoVisto = (pos[i] > ref) ? +1 : -1;
  }
  contMarca = ladoVisto ? contMarca + 1 : 0;
  if (ladoVisto && contMarca >= N_MARCA) {
    if (!marcaReciente() || ladoVisto != ladoMarca)
      logEvento("MARCA lado=%+d", ladoVisto);
    ladoMarca = ladoVisto;
    tMarca = ms;
  }

  // --- 4) Se ve la propia: PD normal ---
  if (iPropia >= 0) {
    float e = pos[iPropia];
    ultimoError = e;
    registrarError(e);
    ultimaVezLinea = ms;
    aplicarControl(e, nullptr);
    logEstado(e);
    return;
  }

  // --- 5) No se ve la propia: mantener rumbo un rato (hueco entre sensores) ---
  unsigned long tHueco = marcaReciente() ? T_HUECO_V_MS : T_HUECO_MS;
  if (ms - ultimaVezLinea < tHueco) {
    // Centrada o con una marca a la vista: derecho. Si no, seguir corrigiendo.
    float e = (n > 0 || fabs(ultimoError) <= 1.0f) ? 0 : ultimoError;
    aplicarControl(e, "HUEC");
    logEstado(e);
    return;
  }

  // --- 6) La línea se acabó: ¿V (hubo marca) o perdida? ---
  if (marcaReciente()) empezarGiro(ladoMarca, "linea acabada con marca");
  else                 empezarBusqueda(ultimoError >= 0 ? +1 : -1, "linea perdida");
}

void girarV(float pos[], int ancho[], int n) {
  unsigned long t = millis() - inicioGiro;
  if (t >= T_GIRO_MIN_MS) {
    int i = buscarLinea(pos, ancho, n, ladoGiro);
    if (i >= 0) { reenganchar(pos[i], "REENGANCHE V"); return; }
  }
  if (t > T_GIRO_MAX_MS) { empezarBusqueda(ladoGiro, "V sin rama"); return; }
  pivotarHacia(ladoGiro, SPEED_VEE);
  telModo = "VEE";
  logEstado(5.0f * ladoGiro);
}

void buscar(float pos[], int ancho[], int n) {
  int i = buscarLinea(pos, ancho, n, ladoGiro);
  if (i >= 0) { reenganchar(pos[i], "REENGANCHE"); return; }
  pivotarHacia(ladoGiro, SPEED_BUSQUEDA);
  telModo = "BUSQ";
  logEstado(5.0f * ladoGiro);
}

//=====================================================================
void setup() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(PINES_SENSORES[i], INPUT);
    bAnterior[i] = false;
  }
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ultimaVezLinea = millis();

#if DEBUG
  SerialBT.begin("SeguidorV88");   // mismo nombre: el PC ya lo tiene emparejado
  logEvento("=== SEGUIDOR DE LINEA - v9.0 SIMPLE ===");  cerrarLinea();
  logEvento("Modos: PROP PIVO HUEC CRUZ VEE BUSQ");     cerrarLinea();
  const char* motivo;
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  motivo = "ENCENDIDO"; break;
    case ESP_RST_BROWNOUT: motivo = "BROWNOUT (bajo voltaje!)"; break;
    case ESP_RST_PANIC:    motivo = "PANIC (error de programa)"; break;
    case ESP_RST_EXT:      motivo = "BOTON RESET"; break;
    default:               motivo = "OTRO"; break;
  }
  logEvento("REINICIO: %s", motivo);  cerrarLinea();
#endif
}

void loop() {
  unsigned long ahora = micros();
  if (ahora - ultimaMuestraUs < PERIODO_US) return;
  ultimaMuestraUs = ahora;

  leerSensores();
  float pos[3];
  int   ancho[3];
  int   n = leerGrupos(pos, ancho);

  if      (modo == GIRO_V) girarV(pos, ancho, n);
  else if (modo == BUSCAR) buscar(pos, ancho, n);
  else                     seguir(pos, ancho, n);
}
