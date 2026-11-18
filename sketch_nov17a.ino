// =============================================================================
//  Identificación y control de un motor DC (JGA25-370, 34:1) — ESP32 + L298N
//  Encoder de cuadratura leído por HARDWARE (PCNT) + Servidor Web + ThingSpeak
//
//  Requiere: Arduino-ESP32 core v3.x (ESP-IDF 5.x). Sin librerías externas:
//  WiFi.h, WebServer.h, HTTPClient.h y driver/pulse_cnt.h vienen con el core.
//
//  PINES (SIN CAMBIOS): ENCODER_A=18, ENCODER_B=19, ENA_PIN=25, IN1_PIN=26, IN2_PIN=27
//
//  Protocolo serial (115200 baudios):
//    'S' -> Prueba de escalón 100%.  "#INICIO,N,Ts,errores" + N líneas
//           "t_s,rpm,rev" + "#FIN"       <-- compatible con Validacion_Motor_DC.m
//    'B' -> Barrido de escalones 20-100% PWM. "#BARRIDO,M" + M líneas
//           "pwm,duty_pct,voltaje_V,rpm_estable" + "#FIN"
//    'P' -> Excitación PRBS. "#PRBS,N,Ts,duty_bajo,duty_alto" + N líneas
//           "t_s,rpm,rev,duty" + "#FIN"
//    'X' -> Parada de emergencia inmediata (también disponible desde la web).
// =============================================================================

#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "esp_timer.h"
#include "driver/pulse_cnt.h"

// ============================= EDITA AQUÍ ===================================
// ---- WiFi (modo estación). Si falla, el ESP32 levanta un punto de acceso ----
const char* WIFI_SSID       = "Yola36";
const char* WIFI_PASSWORD   = "36558142";
const char* AP_SSID         = "MotorDC_ESP32";     // red de respaldo
const char* AP_PASSWORD     = "motordc123";        // mínimo 8 caracteres

// ---- ThingSpeak --------------------------------------------------------
unsigned long THINGSPEAK_CHANNEL_ID = 3585696;             // tu Channel ID
const char*   THINGSPEAK_WRITE_KEY  = "01DKMH87H2RUMUT7";  // tu Write API Key

// ---- Voltaje efectivo medido en bornes del motor bajo carga (PWM = 255) ----
float V_alimentacion_motor = 4.25f;   // V
// =============================================================================

// ---------------------------- Pines (NO CAMBIAR) ------------------------------
#define ENCODER_A  18
#define ENCODER_B  19
#define ENA_PIN    25
#define IN1_PIN    26
#define IN2_PIN    27

// ---------------------------- Motor y encoder --------------------------------
const float PPR         = 11.0f;
const float REDUCCION   = 34.0f;
const float CUENTAS_REV = PPR * 4.0f * REDUCCION;   // 1496 cuentas/vuelta (x4)
const int   SENTIDO     = -1;                       // giro adelante -> valores positivos

// ---------------------------- PWM ---------------------------------------------
const uint32_t PWM_FREQ     = 5000;
const uint8_t  PWM_RES      = 8;
const uint8_t  DUTY_ESCALON = 255;

// ---------------------------- Muestreo de alta velocidad ----------------------
uint32_t       TS_US     = 2000;        // 2 ms por defecto (ajustable 1-5 ms vía /config)
const uint32_t TS_US_MIN = 1000;
const uint32_t TS_US_MAX = 5000;

const uint16_t N_PRE_ESCALON  = 50;     // 0.1 s antes del escalón (idéntico al original)
const uint16_t N_POST_ESCALON = 1000;   // 2.0 s después del escalón

#define MAX_MUESTRAS 5200                // 5100 muestras PRBS + margen; reduce uso de DRAM
struct Muestra {
  int32_t t_us;    // tiempo relativo al inicio de la captura (µs)
  int32_t cuenta;  // posición absoluta del encoder (cuentas x4, PCNT + overflow)
  uint8_t pwm;     // PWM aplicado en el instante de la muestra (0-255)
};
static Muestra buf[MAX_MUESTRAS];
volatile uint16_t idxMuestra        = 0;
volatile uint16_t nMuestrasObjetivo = 0;
volatile bool     capturando        = false;   // la ISR está llenando buf[]
volatile uint8_t  pwmDuranteCaptura = 0;
int64_t tInicioCaptura_us = 0;

// Metadatos de la última captura de alta velocidad (Escalón o PRBS)
enum TipoUltimaCaptura { CAP_NINGUNA, CAP_ESCALON, CAP_PRBS };
volatile TipoUltimaCaptura ultimaCapturaTipo = CAP_NINGUNA;
uint16_t ultimaCapturaN        = 0;
int32_t  ultimaCapturaOffsetUs = 0;
uint32_t ultimaCapturaTsUs     = 2000;
uint32_t ultimaCapturaId       = 0;

hw_timer_t *timerMuestreo = nullptr;

// ---------------------------- PCNT (contador de pulsos por hardware) ----------
pcnt_unit_handle_t    pcntUnidad = nullptr;
pcnt_channel_handle_t pcntCanalA = nullptr;
pcnt_channel_handle_t pcntCanalB = nullptr;
volatile int64_t overflowPCNT = 0;   // acumula los +-32768 del contador de 16 bits

// ---------------------------- WiFi / Web ---------------------------------------
const uint32_t WIFI_TIMEOUT_MS = 10000;
WebServer server(80);

// ---------------------------- ThingSpeak (estado) ------------------------------
const char* THINGSPEAK_HOST  = "api.thingspeak.com";
bool        thingspeakActivo    = true;
uint32_t    thingspeakPeriodoMs = 15000;   // ThingSpeak free: mínimo 15 s entre updates
uint32_t    tsUltimoEnvio       = 0;
volatile bool thingspeakForzar  = false;

// ---------------------------- Gemelo digital (parámetros calibrados) ----------
struct ParamGemelo { float J, B, Ka, Km, R; };
ParamGemelo gemelo = { 0.0003965f, 0.002402f, 0.23351629f, 0.23351629f, 3.4f };
float omegaGemelo_rads  = 0.0f;
float omegaReal_rads    = 0.0f;
float errorResiduo_rads = 0.0f;

// ---------------------------- Telemetría de baja velocidad --------------------
uint32_t tTelemetriaAnterior_ms      = 0;
int32_t  cuentaTelemetriaAnterior    = 0;
const uint32_t PERIODO_TELEMETRIA_MS = 150;

// ---------------------------- Estado general / modo manual ---------------------
volatile bool    ocupado = false;              // true durante Escalón, Barrido o PRBS
volatile uint8_t pwmManualActual = 0;
volatile bool    direccionManualAdelante = true;
enum EstadoMotor { REPOSO, MANUAL, CAPTURA_ESCALON, CAPTURA_BARRIDO, CAPTURA_PRBS };
volatile EstadoMotor estadoActual = REPOSO;

// =============================================================================
//                              PCNT: configuración
// =============================================================================
static bool IRAM_ATTR onPCNTAlcance(pcnt_unit_handle_t unidad,
                                     const pcnt_watch_event_data_t *datos,
                                     void *contexto) {
  if (datos->watch_point_value == 32767)       overflowPCNT += 32768;
  else if (datos->watch_point_value == -32768) overflowPCNT -= 32768;
  return false;
}

static inline int32_t leerCuentaPCNT() {
  int cuentaBruta = 0;
  pcnt_unit_get_count(pcntUnidad, &cuentaBruta);
  return (int32_t)(overflowPCNT + cuentaBruta);
}

void configurarPCNT() {
  pcnt_unit_config_t configUnidad = {};
  configUnidad.low_limit  = -32768;
  configUnidad.high_limit =  32767;
  ESP_ERROR_CHECK(pcnt_new_unit(&configUnidad, &pcntUnidad));

  pcnt_glitch_filter_config_t filtro = {};
  filtro.max_glitch_ns = 1000;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcntUnidad, &filtro));

  pcnt_chan_config_t cfgA = {};
  cfgA.edge_gpio_num  = ENCODER_A;
  cfgA.level_gpio_num = ENCODER_B;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnidad, &cfgA, &pcntCanalA));

  pcnt_chan_config_t cfgB = {};
  cfgB.edge_gpio_num  = ENCODER_B;
  cfgB.level_gpio_num = ENCODER_A;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnidad, &cfgB, &pcntCanalB));

  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntCanalA,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntCanalA,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntCanalB,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntCanalB,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcntUnidad, 32767));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcntUnidad, -32768));

  pcnt_event_callbacks_t cbs = {};
  cbs.on_reach = &onPCNTAlcance;
  ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcntUnidad, &cbs, nullptr));

  ESP_ERROR_CHECK(pcnt_unit_enable(pcntUnidad));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcntUnidad));
  ESP_ERROR_CHECK(pcnt_unit_start(pcntUnidad));
}

// =============================================================================
//                              Motor: utilidades
// =============================================================================
void aplicarPWM(int16_t valor) {
  valor = constrain(valor, -255, 255);
  if (valor > 0) {
    digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, (uint8_t)valor);
  } else if (valor < 0) {
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH);
    ledcWrite(ENA_PIN, (uint8_t)(-valor));
  } else {
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, 0);
  }
}

// =============================================================================
//                    ISR del temporizador de muestreo (cada Ts)
// =============================================================================
void IRAM_ATTR isrMuestreo() {
  if (!capturando) return;
  uint16_t k = idxMuestra;
  if (k >= nMuestrasObjetivo) { capturando = false; return; }
  buf[k].t_us   = (int32_t)(esp_timer_get_time() - tInicioCaptura_us);
  buf[k].cuenta = leerCuentaPCNT();
  buf[k].pwm    = pwmDuranteCaptura;
  idxMuestra = k + 1;
}

// =============================================================================
//               Motor de captura de alta velocidad (Escalón y PRBS)
// =============================================================================
void iniciarCapturaAltaVelocidad(uint16_t nMuestras) {
  idxMuestra = 0;
  nMuestrasObjetivo = nMuestras;
  tInicioCaptura_us = esp_timer_get_time();
  capturando = true;
  timerRestart(timerMuestreo);
  timerAlarm(timerMuestreo, TS_US, true, 0);
  timerStart(timerMuestreo);
}

void esperarFinCaptura() {
  while (capturando && idxMuestra < nMuestrasObjetivo) { yield(); }
  timerStop(timerMuestreo);
  capturando = false;
}

float calcularRPM(uint16_t k, uint16_t n) {
  if (n < 2) return 0.0f;
  int32_t dC, dT_us;
  if (k == 0)            { dC = buf[1].cuenta   - buf[0].cuenta;   dT_us = buf[1].t_us   - buf[0].t_us; }
  else if (k == n - 1)   { dC = buf[k].cuenta   - buf[k-1].cuenta; dT_us = buf[k].t_us   - buf[k-1].t_us; }
  else                   { dC = buf[k+1].cuenta - buf[k-1].cuenta; dT_us = buf[k+1].t_us - buf[k-1].t_us; }
  if (dT_us <= 0) return 0.0f;
  return SENTIDO * ((float)dC / CUENTAS_REV) / ((float)dT_us * 1e-6f) * 60.0f;
}

// ---------------------------- Prueba de escalón (100 %) -----------------------
void enviarCapturaEscalon(int32_t offsetEscalon_us) {
  const uint16_t N = N_PRE_ESCALON + N_POST_ESCALON;
  Serial.printf("#INICIO,%u,%.6f,%lu\n", N, TS_US * 1e-6f, 0UL);
  for (uint16_t k = 0; k < N; k++) {
    float t_s = (float)(buf[k].t_us - offsetEscalon_us) * 1e-6f;
    float rpm = calcularRPM(k, N);
    float rev = SENTIDO * (float)buf[k].cuenta / CUENTAS_REV;
    Serial.printf("%.6f,%.3f,%.5f\n", t_s, rpm, rev);
  }
  Serial.println("#FIN");
}

void ejecutarEscalon() {
  ocupado = true;
  aplicarPWM(0);
  delay(500);
  pwmDuranteCaptura = 0;

  const uint16_t N = N_PRE_ESCALON + N_POST_ESCALON;
  iniciarCapturaAltaVelocidad(N);
  while (idxMuestra < N_PRE_ESCALON) { yield(); }

  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  ledcWrite(ENA_PIN, DUTY_ESCALON);
  pwmDuranteCaptura = DUTY_ESCALON;
  int32_t offsetEscalon_us = (int32_t)(esp_timer_get_time() - tInicioCaptura_us);

  esperarFinCaptura();
  aplicarPWM(0);

  ultimaCapturaTipo     = CAP_ESCALON;
  ultimaCapturaN        = N;
  ultimaCapturaOffsetUs = offsetEscalon_us;
  ultimaCapturaTsUs     = TS_US;
  ultimaCapturaId++;

  enviarCapturaEscalon(offsetEscalon_us);

  // Sincronizar telemetría de baja velocidad antes de liberar ocupado
  cuentaTelemetriaAnterior = leerCuentaPCNT();
  tTelemetriaAnterior_ms   = millis();
  omegaReal_rads           = 0.0f;
  omegaGemelo_rads         = 0.0f;
  errorResiduo_rads        = 0.0f;
  ocupado = false;
}

// ---------------------------- PRBS ---------------------------------------------
uint8_t lfsrPRBS = 0xB4;
bool siguienteBitPRBS() {
  uint8_t bit = ((lfsrPRBS >> 7) ^ (lfsrPRBS >> 5) ^ (lfsrPRBS >> 4) ^ (lfsrPRBS >> 3)) & 1;
  lfsrPRBS = (uint8_t)((lfsrPRBS << 1) | bit);
  return bit;
}

const uint8_t  DUTY_PRBS_BAJO        = 102;
const uint8_t  DUTY_PRBS_ALTO        = 217;
const uint16_t MUESTRAS_POR_BIT_PRBS = 10;
const uint16_t BITS_PRBS             = 255 * 2;
const uint16_t N_PRBS                = BITS_PRBS * MUESTRAS_POR_BIT_PRBS;

void enviarCapturaPRBS() {
  Serial.printf("#PRBS,%u,%.6f,%u,%u\n", N_PRBS, TS_US * 1e-6f, DUTY_PRBS_BAJO, DUTY_PRBS_ALTO);
  for (uint16_t k = 0; k < N_PRBS; k++) {
    float t_s = (float)buf[k].t_us * 1e-6f;
    float rpm = calcularRPM(k, N_PRBS);
    float rev = SENTIDO * (float)buf[k].cuenta / CUENTAS_REV;
    Serial.printf("%.6f,%.3f,%.5f,%u\n", t_s, rpm, rev, buf[k].pwm);
  }
  Serial.println("#FIN");
}

void ejecutarPRBS() {
  if (N_PRBS > MAX_MUESTRAS) { Serial.println("#ERROR PRBS excede el buffer disponible"); return; }
  ocupado = true;
  aplicarPWM(0);
  delay(500);
  lfsrPRBS = 0xB4;

  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  pwmDuranteCaptura = DUTY_PRBS_BAJO;
  ledcWrite(ENA_PIN, DUTY_PRBS_BAJO);

  iniciarCapturaAltaVelocidad(N_PRBS);

  uint16_t ultimoBloque = 0;
  while (idxMuestra < N_PRBS) {
    uint16_t bloqueActual = idxMuestra / MUESTRAS_POR_BIT_PRBS;
    if (bloqueActual != ultimoBloque) {
      ultimoBloque = bloqueActual;
      uint8_t duty = siguienteBitPRBS() ? DUTY_PRBS_ALTO : DUTY_PRBS_BAJO;
      pwmDuranteCaptura = duty;
      ledcWrite(ENA_PIN, duty);
    }
    yield();
  }
  esperarFinCaptura();
  aplicarPWM(0);

  ultimaCapturaTipo     = CAP_PRBS;
  ultimaCapturaN        = N_PRBS;
  ultimaCapturaOffsetUs = 0;
  ultimaCapturaTsUs     = TS_US;
  ultimaCapturaId++;

  enviarCapturaPRBS();

  // Sincronizar telemetría de baja velocidad antes de liberar ocupado
  cuentaTelemetriaAnterior = leerCuentaPCNT();
  tTelemetriaAnterior_ms   = millis();
  omegaReal_rads           = 0.0f;
  omegaGemelo_rads         = 0.0f;
  errorResiduo_rads        = 0.0f;
  ocupado = false;
}

// ---------------------------- Barrido de escalones (20-100 %) -----------------
const uint8_t  NIVELES_BARRIDO[]   = {20, 30, 40, 50, 60, 70, 80, 90, 100};
const uint8_t  N_NIVELES_BARRIDO   = sizeof(NIVELES_BARRIDO) / sizeof(NIVELES_BARRIDO[0]);
const uint32_t T_MAX_ASENTAMIENTO_MS      = 3000;
const uint32_t T_MIN_ANTES_DE_EVALUAR_MS  = 800;
const uint8_t  VENTANA_ASENTAMIENTO       = 15;
const float    UMBRAL_REL_ASENTAMIENTO    = 0.03f;

float medirRPMInstantanea(int32_t &cuentaAnterior, uint32_t &tAnteriorUs) {
  int32_t cuentaActual = leerCuentaPCNT();
  uint32_t tActualUs = micros();
  int32_t dC = cuentaActual - cuentaAnterior;
  uint32_t dT = tActualUs - tAnteriorUs;
  float rpm = (dT > 0) ? SENTIDO * ((float)dC / CUENTAS_REV) / ((float)dT * 1e-6f) * 60.0f : 0.0f;
  cuentaAnterior = cuentaActual;
  tAnteriorUs = tActualUs;
  return rpm;
}

void ejecutarBarrido() {
  ocupado = true;
  aplicarPWM(0);
  delay(500);
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);

  float rpmEstable[N_NIVELES_BARRIDO];
  int32_t cuentaAnt = leerCuentaPCNT();
  uint32_t tAnt = micros();

  for (uint8_t i = 0; i < N_NIVELES_BARRIDO; i++) {
    uint8_t duty = (uint8_t)roundf(255.0f * NIVELES_BARRIDO[i] / 100.0f);
    ledcWrite(ENA_PIN, duty);
    pwmDuranteCaptura = duty;

    float ventana[VENTANA_ASENTAMIENTO] = {0};
    uint16_t nv = 0;
    uint32_t tInicioNivel = millis();
    float rpmFinal = 0.0f;

    while (millis() - tInicioNivel < T_MAX_ASENTAMIENTO_MS) {
      delay(20);
      float rpm = medirRPMInstantanea(cuentaAnt, tAnt);
      ventana[nv % VENTANA_ASENTAMIENTO] = rpm;
      nv++;
      rpmFinal = rpm;
      if (nv >= VENTANA_ASENTAMIENTO && (millis() - tInicioNivel) > T_MIN_ANTES_DE_EVALUAR_MS) {
        float suma = 0, suma2 = 0;
        for (uint8_t j = 0; j < VENTANA_ASENTAMIENTO; j++) { suma += ventana[j]; suma2 += ventana[j]*ventana[j]; }
        float media = suma / VENTANA_ASENTAMIENTO;
        float var   = suma2 / VENTANA_ASENTAMIENTO - media*media;
        float desv  = (var > 0) ? sqrtf(var) : 0.0f;
        rpmFinal = media;
        if (fabsf(media) < 1.0f || desv / fabsf(media) < UMBRAL_REL_ASENTAMIENTO) break;
      }
      yield();
    }
    rpmEstable[i] = rpmFinal;
  }
  aplicarPWM(0);

  Serial.printf("#BARRIDO,%u\n", (unsigned)N_NIVELES_BARRIDO);
  for (uint8_t i = 0; i < N_NIVELES_BARRIDO; i++) {
    uint8_t duty = (uint8_t)roundf(255.0f * NIVELES_BARRIDO[i] / 100.0f);
    float V = V_alimentacion_motor * duty / 255.0f;
    Serial.printf("%u,%u,%.4f,%.3f\n", duty, NIVELES_BARRIDO[i], V, rpmEstable[i]);
  }
  Serial.println("#FIN");
  delay(150);  // deja que el rotor frene por completo tras el escalón del 100 %

  // Sincronizar telemetría de baja velocidad antes de liberar ocupado
  cuentaTelemetriaAnterior = leerCuentaPCNT();
  tTelemetriaAnterior_ms   = millis();
  omegaReal_rads           = 0.0f;
  omegaGemelo_rads         = 0.0f;
  errorResiduo_rads        = 0.0f;
  ocupado = false;
}

// =============================================================================
//                    Telemetría de baja velocidad + Gemelo digital
// =============================================================================
void enviarThingSpeak(float omega, float omegaEst, float error, float indicadorFriccion) {
  if (WiFi.status() != WL_CONNECTED) return;
  if (THINGSPEAK_CHANNEL_ID == 0 || strlen(THINGSPEAK_WRITE_KEY) < 8) return;
  HTTPClient http;
  http.setTimeout(1500);
  String url = String("http://") + THINGSPEAK_HOST + "/update?api_key=" + THINGSPEAK_WRITE_KEY +
               "&field1=" + String(omega, 4) + "&field2=" + String(omegaEst, 4) +
               "&field3=" + String(error, 4) + "&field4=" + String(indicadorFriccion, 2);
  http.begin(url);
  http.GET();
  http.end();
}

void actualizarTelemetria() {
  uint32_t ahora = millis();
  if (ahora - tTelemetriaAnterior_ms < PERIODO_TELEMETRIA_MS) return;
  float dt_s = (ahora - tTelemetriaAnterior_ms) / 1000.0f;
  tTelemetriaAnterior_ms = ahora;

  int32_t cuentaActual = leerCuentaPCNT();
  int32_t dC = cuentaActual - cuentaTelemetriaAnterior;
  cuentaTelemetriaAnterior = cuentaActual;
  float rpmInst = (dt_s > 0) ? SENTIDO * ((float)dC / CUENTAS_REV) / dt_s * 60.0f : 0.0f;
  omegaReal_rads = rpmInst * 2.0f * PI / 60.0f;

  // Gemelo digital: discretización exacta de 1er orden (ZOH) para evitar
  // inestabilidad numérica cuando dt_s (150 ms) > tau_m (~21.5 ms).
  uint8_t pwmActual = ocupado ? pwmDuranteCaptura : pwmManualActual;
  float signoV = (ocupado ? true : direccionManualAdelante) ? 1.0f : -1.0f;
  float V = V_alimentacion_motor * (pwmActual / 255.0f) * signoV;
  float denom   = gemelo.R * gemelo.B + gemelo.Ka * gemelo.Km;
  float tau_m   = (denom > 1e-9f) ? (gemelo.J * gemelo.R / denom) : 0.0f;
  float omegaSS = (denom > 1e-9f) ? (gemelo.Km * V / denom) : 0.0f;
  if (tau_m > 1e-6f) {
    float alpha = 1.0f - expf(-dt_s / tau_m);
    omegaGemelo_rads += alpha * (omegaSS - omegaGemelo_rads);
  }

  errorResiduo_rads = omegaReal_rads - omegaGemelo_rads;

  if (thingspeakActivo || thingspeakForzar) {
    if (thingspeakForzar || (ahora - tsUltimoEnvio > thingspeakPeriodoMs)) {
      float magRef = fmaxf(fabsf(omegaGemelo_rads), fabsf(omegaReal_rads));
      float indicador = (pwmActual > 0 && magRef > 0.5f)
                       ? constrain(100.0f * fabsf(errorResiduo_rads) / magRef, 0.0f, 100.0f) : 0.0f;
      enviarThingSpeak(omegaReal_rads, omegaGemelo_rads, errorResiduo_rads, indicador);
      tsUltimoEnvio = ahora;
      thingspeakForzar = false;
    }
  }
}

// =============================================================================
//                        Servidor Web (Soft UI / Neumorphism)
// =============================================================================
const char PAGINA_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Motor DC JGA25-370 • Gemelo Digital</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
  :root{
    --bg:#eef2f6;
    --card:#f6f9fc;
    --text:#1e293b;
    --muted:#64748b;
    --blue:#0088ff;
    --blue-dark:#0062cc;
    --green:#10b981;
    --amber:#f59e0b;
    --coral:#ff5a5f;
    --red:#ef4444;
    --sh-out:8px 8px 18px rgba(163,177,198,0.45), -8px -8px 18px rgba(255,255,255,0.92);
    --sh-in:inset 3px 3px 7px rgba(163,177,198,0.42), inset -3px -3px 7px rgba(255,255,255,0.92);
  }
  *{box-sizing:border-box;margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;}
  body{background:var(--bg);color:var(--text);padding:16px;min-height:100vh;}
  .wrap{max-width:1280px;margin:0 auto;display:flex;flex-direction:column;gap:16px;}

  .topbar{
    background:var(--card);border-radius:24px;padding:12px 18px;box-shadow:var(--sh-out);
    display:flex;flex-wrap:wrap;align-items:center;justify-content:space-between;gap:12px;
    border:1px solid rgba(255,255,255,0.85);
  }
  .brand{display:flex;align-items:center;gap:10px;}
  .brand-dot{width:12px;height:12px;border-radius:50%;background:var(--blue);box-shadow:0 0 10px var(--blue);}
  .brand h1{font-size:1.05rem;font-weight:800;letter-spacing:-0.2px;color:#0f172a;}
  .brand small{display:block;font-size:0.72rem;color:var(--muted);font-weight:600;}

  .nav-pill{
    display:flex;align-items:center;gap:10px;padding:7px 16px;border-radius:999px;
    background:var(--bg);border:2px solid var(--blue);
    box-shadow:0 4px 14px rgba(0,136,255,0.18), -3px -3px 8px rgba(255,255,255,0.85);
    flex-wrap:wrap;justify-content:center;
  }
  .chip{font-size:0.76rem;font-weight:700;padding:4px 10px;border-radius:999px;background:#fff;color:var(--text);box-shadow:2px 2px 6px rgba(163,177,198,0.35);}
  .chip-ok{color:#059669;background:#ecfdf5;}
  .chip-run{color:#0284c7;background:#e0f2fe;}
  .chip-busy{color:#d97706;background:#fef3c7;animation:pulse 1.2s infinite;}
  @keyframes pulse{0%,100%{opacity:1;}50%{opacity:0.55;}}

  .btn-paro{
    background:linear-gradient(135deg,#ff5252,#dc2626);color:#fff;border:none;
    border-radius:999px;padding:10px 20px;font-weight:800;font-size:0.82rem;letter-spacing:0.4px;
    cursor:pointer;box-shadow:0 6px 16px rgba(220,38,38,0.35);transition:transform 0.12s,box-shadow 0.12s;
  }
  .btn-paro:hover{transform:translateY(-1px);box-shadow:0 8px 20px rgba(220,38,38,0.45);}
  .btn-paro:active{transform:scale(0.97);}

  .grid-top{display:grid;grid-template-columns:340px 1fr;gap:16px;}
  .grid-bot{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;}
  @media(max-width:980px){.grid-top{grid-template-columns:1fr;}.grid-bot{grid-template-columns:1fr;}}

  .card{
    background:var(--card);border-radius:22px;padding:18px;box-shadow:var(--sh-out);
    border:1px solid rgba(255,255,255,0.85);display:flex;flex-direction:column;justify-content:space-between;
  }
  .card-head{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px;gap:8px;flex-wrap:wrap;}
  .pill-tag{
    background:linear-gradient(135deg,#0099ff,#0066dd);color:#fff;font-size:0.72rem;
    font-weight:700;padding:5px 13px;border-radius:999px;letter-spacing:0.2px;
    box-shadow:0 3px 8px rgba(0,136,255,0.28);
  }
  .sub-note{font-size:0.74rem;color:var(--muted);font-weight:600;}

  .hero-blue{
    background:linear-gradient(135deg,#009dff 0%,#0062d6 100%);color:#fff;
    box-shadow:10px 12px 24px rgba(0,114,230,0.32), -6px -6px 16px rgba(255,255,255,0.85);
    border:none;
  }
  .hero-badge{
    background:rgba(255,255,255,0.2);backdrop-filter:blur(4px);color:#fff;
    font-size:0.72rem;font-weight:700;padding:5px 12px;border-radius:999px;
  }
  .hero-main{margin:10px 0 16px 0;}
  .hero-label{font-size:0.78rem;opacity:0.88;font-weight:600;text-transform:uppercase;letter-spacing:0.5px;}
  .hero-rpm{font-size:2.65rem;font-weight:800;line-height:1.05;margin:4px 0;}
  .hero-rpm span{font-size:1rem;font-weight:600;opacity:0.9;}
  .hero-rads{
    display:inline-block;background:rgba(255,255,255,0.18);padding:4px 12px;
    border-radius:999px;font-size:0.88rem;font-weight:700;margin-top:4px;
  }
  .hero-subgrid{display:grid;grid-template-columns:1fr 1fr;gap:10px;}
  .hero-mini{
    background:#ffffff;color:var(--text);border-radius:16px;padding:10px 12px;
    display:flex;align-items:center;gap:10px;box-shadow:0 4px 12px rgba(0,45,110,0.16);
  }
  .mini-ico{
    width:32px;height:32px;border-radius:50%;display:flex;align-items:center;justify-content:center;
    font-weight:800;font-size:0.95rem;flex-shrink:0;
  }
  .ico-v{background:#e0f2fe;color:#0284c7;}
  .ico-dir{background:#dcfce7;color:#059669;}
  .mini-val{font-size:0.98rem;font-weight:800;color:#0f172a;line-height:1.1;}
  .mini-lbl{font-size:0.68rem;color:var(--muted);font-weight:600;}

  .legend-row{display:flex;align-items:center;gap:10px;font-size:0.75rem;font-weight:700;flex-wrap:wrap;}
  .lg-item{display:flex;align-items:center;gap:5px;color:var(--muted);}
  .lg-dot{width:10px;height:10px;border-radius:50%;}
  .mode-switch{
    display:inline-flex;background:var(--bg);padding:3px;border-radius:999px;box-shadow:var(--sh-in);gap:3px;
  }
  .mode-btn{
    border:none;background:transparent;color:var(--muted);font-size:0.7rem;font-weight:700;
    padding:4px 10px;border-radius:999px;cursor:pointer;transition:all 0.15s;
  }
  .mode-btn.active{background:var(--blue);color:#fff;box-shadow:0 2px 6px rgba(0,136,255,0.35);}
  .cap-banner{
    display:none;background:#e0f2fe;color:#0369a1;font-size:0.74rem;font-weight:700;
    padding:6px 12px;border-radius:10px;margin-bottom:8px;justify-content:space-between;align-items:center;
  }
  .chart-box{
    position:relative;height:215px;width:100%;background:#ffffff;border-radius:16px;
    padding:10px;box-shadow:var(--sh-in);
  }
  .chart-offline{
    display:none;position:absolute;inset:0;align-items:center;justify-content:center;
    text-align:center;padding:16px;color:var(--muted);font-size:0.82rem;font-weight:600;
  }

  .gauges-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;}
  @media(max-width:720px){.gauges-grid{grid-template-columns:1fr;}}
  .gauge-item{
    background:var(--bg);border-radius:18px;padding:12px 16px;box-shadow:var(--sh-in);
    display:flex;align-items:center;gap:16px;
  }
  .ring-wrap{position:relative;width:82px;height:82px;flex-shrink:0;}
  .ring-wrap svg{width:82px;height:82px;transform:rotate(-90deg);}
  .ring-bg{fill:none;stroke:#dde5ef;stroke-width:8;}
  .ring-fg{fill:none;stroke-width:8;stroke-linecap:round;transition:stroke-dashoffset 0.22s ease,stroke 0.22s ease;}
  .ring-txt{
    position:absolute;inset:0;display:flex;align-items:center;justify-content:center;
    font-size:0.95rem;font-weight:800;color:#0f172a;
  }
  .g-info h4{font-size:0.9rem;font-weight:800;color:#0f172a;margin-bottom:2px;}
  .g-info p{font-size:0.75rem;color:var(--muted);font-weight:600;}
  .g-sub{display:inline-block;margin-top:5px;font-size:0.72rem;font-weight:700;padding:2px 8px;border-radius:999px;background:#fff;color:var(--text);}

  .slider-well{background:var(--bg);border-radius:16px;padding:14px;box-shadow:var(--sh-in);margin-bottom:12px;}
  .slider-top{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;font-size:0.82rem;font-weight:700;}
  .pwm-badge{background:var(--blue);color:#fff;padding:3px 10px;border-radius:999px;font-size:0.78rem;}
  input[type=range]{
    -webkit-appearance:none;width:100%;height:10px;border-radius:999px;background:#dce4ee;
    box-shadow:inset 2px 2px 5px rgba(163,177,198,0.6);outline:none;
  }
  input[type=range]::-webkit-slider-thumb{
    -webkit-appearance:none;width:22px;height:22px;border-radius:50%;
    background:linear-gradient(135deg,#0099ff,#0066dd);cursor:pointer;
    border:3px solid #fff;box-shadow:0 3px 8px rgba(0,102,221,0.45);
  }
  .scale-marks{display:flex;justify-content:space-between;font-size:0.68rem;color:var(--muted);font-weight:700;margin-top:6px;}

  .btn-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;}
  .btn-grid-3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;}
  .btn-neu{
    background:var(--card);color:var(--text);border:1px solid rgba(255,255,255,0.9);
    border-radius:12px;padding:10px 10px;font-size:0.78rem;font-weight:700;cursor:pointer;
    box-shadow:4px 4px 10px rgba(163,177,198,0.42), -4px -4px 10px rgba(255,255,255,0.9);
    transition:all 0.14s ease;text-align:center;
  }
  .btn-neu:hover:not(:disabled){color:var(--blue);transform:translateY(-1px);}
  .btn-neu:active:not(:disabled){box-shadow:var(--sh-in);transform:translateY(0);}
  .btn-neu:disabled{opacity:0.5;cursor:not-allowed;}
  .btn-primary{background:linear-gradient(135deg,#0095ff,#006be6);color:#fff;border:none;}
  .btn-primary:hover:not(:disabled){color:#fff;filter:brightness(1.05);}
  .btn-green{color:#059669;}
  .btn-coral{color:#e11d48;}

  .divider{height:1px;background:#e2e8f0;margin:12px 0;}

  .diag-list{display:flex;flex-direction:column;gap:9px;}
  .diag-row{
    background:var(--bg);border-radius:12px;padding:9px 12px;box-shadow:var(--sh-in);
    display:flex;justify-content:space-between;align-items:center;font-size:0.8rem;
  }
  .diag-k{color:var(--muted);font-weight:600;}
  .diag-v{font-weight:800;color:#0f172a;font-variant-numeric:tabular-nums;}
</style>
</head>
<body>
<div class="wrap">

  <header class="topbar">
    <div class="brand">
      <div class="brand-dot"></div>
      <div>
        <h1>Motor DC JGA25-370 • Gemelo Digital</h1>
        <small>ESP32 PCNT Hardware x4 + Observador EDO</small>
      </div>
    </div>

    <div class="nav-pill">
      <span class="chip" id="wifiInfo">WiFi: --</span>
      <span class="chip chip-ok" id="estadoBadge">REPOSO</span>
    </div>

    <button class="btn-paro" onclick="paroEmergencia()">■ PARADA DE EMERGENCIA</button>
  </header>

  <section class="grid-top">
    <div class="card hero-blue">
      <div class="card-head">
        <span class="hero-badge">Telemetría en Vivo</span>
        <span style="font-size:0.74rem;font-weight:700;opacity:0.9;">250 ms</span>
      </div>

      <div class="hero-main">
        <div class="hero-label">Velocidad Angular Real</div>
        <div class="hero-rpm"><span id="rpmVal" style="font-size:2.65rem;opacity:1;">0.0</span> <span>RPM</span></div>
        <div class="hero-rads"><span id="omegaRealVal">0.000</span> rad/s</div>
      </div>

      <div class="hero-subgrid">
        <div class="hero-mini">
          <div class="mini-ico ico-v">V</div>
          <div>
            <div class="mini-val" id="voltajeVal">0.00 V</div>
            <div class="mini-lbl">Voltaje Bornes</div>
          </div>
        </div>
        <div class="hero-mini">
          <div class="mini-ico ico-dir" id="dirIco">•</div>
          <div>
            <div class="mini-val" id="dirVal">Detenido</div>
            <div class="mini-lbl">Sentido Giro</div>
          </div>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-head">
        <span class="pill-tag" id="chartTitulo">Velocidad Real vs Gemelo Digital (rad/s)</span>
        <div class="legend-row">
          <span class="lg-item"><span class="lg-dot" style="background:#0088ff;"></span>ω Real</span>
          <span class="lg-item"><span class="lg-dot" style="background:#ff5a5f;"></span>ω Gemelo</span>
          <div class="mode-switch">
            <button class="mode-btn active" id="btnModoVivo" onclick="cambiarModoGrafica('vivo')">En Vivo</button>
            <button class="mode-btn" id="btnModoCap" onclick="cambiarModoGrafica('captura')">Última Captura</button>
          </div>
          <button class="btn-neu" style="padding:4px 10px;font-size:0.7rem;" onclick="limpiarGrafica()">Limpiar</button>
        </div>
      </div>
      <div class="cap-banner" id="capBanner">
        <span id="capBannerTxt">Sin captura registrada</span>
        <span style="opacity:0.8;">Curva de alta velocidad</span>
      </div>
      <div class="chart-box">
        <canvas id="graficaVelocidad"></canvas>
        <div class="chart-offline" id="chartOfflineMsg">
          Gráfica en tiempo real no disponible sin conexión a Internet (Chart.js CDN).<br>
          Los indicadores numéricos y controles funcionan normalmente.
        </div>
      </div>
    </div>
  </section>

  <section class="card">
    <div class="card-head">
      <span class="pill-tag">Indicadores de Operación &amp; Residuos</span>
      <span class="sub-note">Supervisión continua del modelo calibrado</span>
    </div>
    <div class="gauges-grid">
      <div class="gauge-item">
        <div class="ring-wrap">
          <svg viewBox="0 0 100 100">
            <circle class="ring-bg" cx="50" cy="50" r="42"></circle>
            <circle class="ring-fg" id="ringPwm" cx="50" cy="50" r="42" stroke="#0088ff"
                    stroke-dasharray="263.89" stroke-dashoffset="263.89"></circle>
          </svg>
          <div class="ring-txt" id="txtPwmPct">0%</div>
        </div>
        <div class="g-info">
          <h4>PWM Aplicado</h4>
          <p>Ciclo de trabajo puente H</p>
          <span class="g-sub" id="subPwmRaw">0 / 255</span>
        </div>
      </div>

      <div class="gauge-item">
        <div class="ring-wrap">
          <svg viewBox="0 0 100 100">
            <circle class="ring-bg" cx="50" cy="50" r="42"></circle>
            <circle class="ring-fg" id="ringErr" cx="50" cy="50" r="42" stroke="#10b981"
                    stroke-dasharray="263.89" stroke-dashoffset="263.89"></circle>
          </svg>
          <div class="ring-txt" id="txtErrPct">0%</div>
        </div>
        <div class="g-info">
          <h4>Residuo Gemelo</h4>
          <p>Error relativo |ω - ω_est|</p>
          <span class="g-sub" id="subResiduo">0.000 rad/s</span>
        </div>
      </div>

      <div class="gauge-item">
        <div class="ring-wrap">
          <svg viewBox="0 0 100 100">
            <circle class="ring-bg" cx="50" cy="50" r="42"></circle>
            <circle class="ring-fg" id="ringFric" cx="50" cy="50" r="42" stroke="#ff5a5f"
                    stroke-dasharray="263.89" stroke-dashoffset="263.89"></circle>
          </svg>
          <div class="ring-txt" id="txtFricPct">0%</div>
        </div>
        <div class="g-info">
          <h4>Fricción / Carga</h4>
          <p>Caída de velocidad por carga</p>
          <span class="g-sub" id="subFricEst">Nominal</span>
        </div>
      </div>
    </div>
  </section>

  <section class="grid-bot">
    <div class="card">
      <div class="card-head">
        <span class="pill-tag">Control Manual</span>
        <span class="sub-note">Lazo abierto</span>
      </div>
      <div class="slider-well">
        <div class="slider-top">
          <span>Consigna PWM</span>
          <span class="pwm-badge" id="sliderBadge">0 (0%)</span>
        </div>
        <input type="range" id="sliderPwm" min="-255" max="255" value="0"
               oninput="onSliderInput(this.value)" onchange="enviarPWM(this.value)">
        <div class="scale-marks">
          <span>-255 (Rev)</span><span>0</span><span>+255 (Ade)</span>
        </div>
      </div>
      <div class="btn-grid">
        <button class="btn-neu ctrl-lock" onclick="invertirGiro()">⇄ Invertir Giro</button>
        <button class="btn-neu btn-coral ctrl-lock" onclick="setManual(0)">■ Freno (0)</button>
        <button class="btn-neu ctrl-lock" onclick="setManual(-150)">↺ Reversa (-150)</button>
        <button class="btn-neu btn-primary ctrl-lock" onclick="setManual(150)">↻ Adelante (+150)</button>
      </div>
    </div>

    <div class="card">
      <div class="card-head">
        <span class="pill-tag">Identificación &amp; Nube</span>
        <span class="chip" id="tsBadge">TS: ON</span>
      </div>

      <div>
        <div class="sub-note" style="margin-bottom:8px;">Captura de alta velocidad (Serial MATLAB + Gráfica)</div>
        <div class="btn-grid-3">
          <button class="btn-neu btn-primary ctrl-lock" onclick="dispararPrueba('/escalon','ESCALÓN 100%',true)">Escalón 100%</button>
          <button class="btn-neu ctrl-lock" onclick="dispararPrueba('/barrido','BARRIDO 20-100%',false)">Barrido 20-100%</button>
          <button class="btn-neu ctrl-lock" onclick="dispararPrueba('/prbs','PRBS',true)">PRBS</button>
        </div>

        <div class="divider"></div>

        <div class="sub-note" style="margin-bottom:8px;">Telemetría ThingSpeak (cada 15 s)</div>
        <div class="btn-grid-3">
          <button class="btn-neu btn-green" onclick="cmdThingSpeak('on')">Activar</button>
          <button class="btn-neu btn-coral" onclick="cmdThingSpeak('off')">Desactivar</button>
          <button class="btn-neu" onclick="cmdThingSpeak('forzar')">Forzar envío</button>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-head">
        <span class="pill-tag">Diagnóstico del Sistema</span>
        <span class="sub-note">Hardware ESP32</span>
      </div>
      <div class="diag-list">
        <div class="diag-row">
          <span class="diag-k">Posición Angular</span>
          <span class="diag-v"><span id="revVal">0.000</span> rev</span>
        </div>
        <div class="diag-row">
          <span class="diag-k">Cuentas Encoder (PCNT)</span>
          <span class="diag-v" id="pcntVal">0</span>
        </div>
        <div class="diag-row">
          <span class="diag-k">Tiempo Actividad (Uptime)</span>
          <span class="diag-v" id="uptimeVal">00:00:00</span>
        </div>
        <div class="diag-row">
          <span class="diag-k">Memoria RAM Libre</span>
          <span class="diag-v" id="heapVal">-- KB</span>
        </div>
      </div>
    </div>
  </section>

</div>

<script>
(() => {
  const CIRC = 263.89;
  const MAX_PUNTOS = 80;
  let chartVel = null;
  let modoGrafica = 'vivo'; // 'vivo' | 'captura'
  let esperandoFinPrueba = false;
  let descargarCapturaAlFinal = false;
  let ultimaCapturaIdVista = 0;
  let ultimoEnvioSlider = 0;

  // Historial en vivo separado para poder alternar sin perderlo
  const histVivo = { labels: [], real: [], gem: [] };

  const initChart = () => {
    if (typeof Chart === 'undefined') {
      document.getElementById('graficaVelocidad').style.display = 'none';
      document.getElementById('chartOfflineMsg').style.display = 'flex';
      return;
    }
    const ctx = document.getElementById('graficaVelocidad').getContext('2d');
    const grad = ctx.createLinearGradient(0, 0, 0, 200);
    grad.addColorStop(0, 'rgba(0, 136, 255, 0.34)');
    grad.addColorStop(1, 'rgba(0, 136, 255, 0.0)');

    chartVel = new Chart(ctx, {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'ω Real (rad/s)',
            data: [],
            borderColor: '#0088ff',
            backgroundColor: grad,
            borderWidth: 2.5,
            fill: true,
            tension: 0.28,
            pointRadius: 0
          },
          {
            label: 'ω Gemelo (rad/s)',
            data: [],
            borderColor: '#ff5a5f',
            borderWidth: 2.2,
            borderDash: [5, 4],
            fill: false,
            tension: 0.28,
            pointRadius: 0
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: { mode: 'index', intersect: false },
        plugins: { legend: { display: false } },
        scales: {
          x: {
            ticks: { maxTicksLimit: 7, color: '#64748b', font: { size: 10 } },
            grid: { display: false }
          },
          y: {
            ticks: { color: '#64748b', font: { size: 10 } },
            grid: { color: 'rgba(148,163,184,0.18)' }
          }
        }
      }
    });
  };

  const aplicarDatosGrafica = (labels, realArr, gemArr) => {
    if (!chartVel) return;
    chartVel.data.labels = labels;
    chartVel.data.datasets[0].data = realArr;
    chartVel.data.datasets[1].data = gemArr;
    chartVel.update('none');
  };

  window.cambiarModoGrafica = (modo) => {
    modoGrafica = modo;
    document.getElementById('btnModoVivo').classList.toggle('active', modo === 'vivo');
    document.getElementById('btnModoCap').classList.toggle('active', modo === 'captura');

    if (modo === 'vivo') {
      document.getElementById('capBanner').style.display = 'none';
      document.getElementById('chartTitulo').innerText = 'Velocidad Real vs Gemelo Digital (rad/s)';
      aplicarDatosGrafica(histVivo.labels, histVivo.real, histVivo.gem);
    } else {
      cargarUltimaCaptura();
    }
  };

  window.limpiarGrafica = () => {
    if (modoGrafica === 'vivo') {
      histVivo.labels = [];
      histVivo.real = [];
      histVivo.gem = [];
      aplicarDatosGrafica([], [], []);
    } else {
      window.cambiarModoGrafica('vivo');
    }
  };

  const cargarUltimaCaptura = async () => {
    try {
      const r = await fetch('/ultima_captura', { cache: 'no-store' });
      if (!r.ok) return;
      const cap = await r.json();
      const banner = document.getElementById('capBanner');
      const bannerTxt = document.getElementById('capBannerTxt');

      if (!cap.disponible || !cap.muestras || cap.muestras.length === 0) {
        banner.style.display = 'flex';
        bannerTxt.innerText = 'Aún no hay captura de Escalón o PRBS almacenada';
        aplicarDatosGrafica([], [], []);
        return;
      }

      modoGrafica = 'captura';
      document.getElementById('btnModoVivo').classList.remove('active');
      document.getElementById('btnModoCap').classList.add('active');

      const nombreTipo = (cap.tipo === 'escalon') ? 'ESCALÓN 100%' : 'PRBS';
      document.getElementById('chartTitulo').innerText = 'Respuesta ' + nombreTipo + ' (rad/s vs ms)';
      banner.style.display = 'flex';
      bannerTxt.innerText = nombreTipo + ' • Ts = ' + Number(cap.ts_ms).toFixed(2) + ' ms • ' + cap.muestras.length + ' muestras';

      const labels = cap.muestras.map(m => Number(m.t_ms).toFixed(0) + ' ms');
      const realArr = cap.muestras.map(m => Number(m.omegaReal));
      const gemArr = cap.muestras.map(m => Number(m.omegaGemelo));
      aplicarDatosGrafica(labels, realArr, gemArr);
    } catch (e) {}
  };

  const setGauge = (ringId, txtId, pct) => {
    const p = Math.max(0, Math.min(100, pct));
    const offset = CIRC * (1 - p / 100);
    document.getElementById(ringId).style.strokeDashoffset = offset.toFixed(2);
    document.getElementById(txtId).innerText = Math.round(p) + '%';
  };

  const formatUptime = (ms) => {
    const s = Math.floor(ms / 1000);
    const hh = String(Math.floor(s / 3600)).padStart(2, '0');
    const mm = String(Math.floor((s % 3600) / 60)).padStart(2, '0');
    const ss = String(s % 60).padStart(2, '0');
    return hh + ':' + mm + ':' + ss;
  };

  const bloquearControles = (lock) => {
    document.querySelectorAll('.ctrl-lock').forEach(b => b.disabled = lock);
    document.getElementById('sliderPwm').disabled = lock;
  };

  window.onSliderInput = (v) => {
    const val = parseInt(v, 10) || 0;
    const pct = Math.round(Math.abs(val) * 100 / 255);
    document.getElementById('sliderBadge').innerText = val + ' (' + pct + '%)';
    const ahora = Date.now();
    if (ahora - ultimoEnvioSlider > 65) {
      ultimoEnvioSlider = ahora;
      fetch('/pwm?valor=' + val).catch(() => {});
    }
  };

  window.enviarPWM = (v) => {
    const val = parseInt(v, 10) || 0;
    const pct = Math.round(Math.abs(val) * 100 / 255);
    document.getElementById('sliderBadge').innerText = val + ' (' + pct + '%)';
    fetch('/pwm?valor=' + val).catch(() => {});
  };

  window.setManual = (val) => {
    const sl = document.getElementById('sliderPwm');
    sl.value = val;
    window.enviarPWM(val);
  };

  window.invertirGiro = () => {
    const sl = document.getElementById('sliderPwm');
    const actual = parseInt(sl.value, 10) || 0;
    const nuevo = (actual === 0) ? -150 : -actual;
    sl.value = nuevo;
    window.enviarPWM(nuevo);
  };

  window.paroEmergencia = () => {
    esperandoFinPrueba = false;
    descargarCapturaAlFinal = false;
    bloquearControles(false);
    const sl = document.getElementById('sliderPwm');
    sl.value = 0;
    document.getElementById('sliderBadge').innerText = '0 (0%)';
    fetch('/paro').catch(() => {});
  };

  window.dispararPrueba = (ruta, etiqueta, tieneCurvaAltaVel) => {
    if (esperandoFinPrueba) return;
    esperandoFinPrueba = true;
    descargarCapturaAlFinal = !!tieneCurvaAltaVel;
    bloquearControles(true);
    const badge = document.getElementById('estadoBadge');
    badge.innerText = 'CAPTURANDO: ' + etiqueta;
    badge.className = 'chip chip-busy';

    fetch(ruta)
      .then(r => {
        if (r.status === 409) throw new Error('Ocupado');
        return r.text();
      })
      .catch(() => {
        esperandoFinPrueba = false;
        descargarCapturaAlFinal = false;
        bloquearControles(false);
      });
  };

  window.cmdThingSpeak = (modo) => {
    fetch('/thingspeak?modo=' + modo).catch(() => {});
  };

  const cicloEstado = async () => {
    let siguienteMs = 250;

    const ctrl = new AbortController();
    const timerAbort = setTimeout(() => ctrl.abort(), 450);

    try {
      const resp = await fetch('/estado', { signal: ctrl.signal, cache: 'no-store' });
      clearTimeout(timerAbort);
      if (!resp.ok) throw new Error('HTTP ' + resp.status);
      const d = await resp.json();

      document.getElementById('wifiInfo').innerText = d.wifiModo + ' • ' + d.ip;

      const badge = document.getElementById('estadoBadge');
      const esBusy = !!d.ocupado || (d.estado !== 'reposo' && d.estado !== 'manual');
      bloquearControles(esBusy);

      if (esBusy) {
        badge.innerText = 'CAPTURANDO (' + d.estado.toUpperCase() + ')';
        badge.className = 'chip chip-busy';
        siguienteMs = 600;
      } else {
        if (d.estado === 'manual') {
          badge.innerText = 'MANUAL';
          badge.className = 'chip chip-run';
        } else {
          badge.innerText = 'REPOSO';
          badge.className = 'chip chip-ok';
        }

        // Si acaba de terminar una prueba de Escalón o PRBS (Web o Serial), cargar curva
        const nuevoIdCap = Number(d.ultimaCapturaId || 0);
        if (esperandoFinPrueba) {
          esperandoFinPrueba = false;
          if (descargarCapturaAlFinal) {
            descargarCapturaAlFinal = false;
            ultimaCapturaIdVista = nuevoIdCap;
            cargarUltimaCaptura();
          }
        } else if (nuevoIdCap > 0 && nuevoIdCap !== ultimaCapturaIdVista) {
          ultimaCapturaIdVista = nuevoIdCap;
          cargarUltimaCaptura();
        }
      }

      document.getElementById('rpmVal').innerText = Number(d.rpm).toFixed(1);
      document.getElementById('omegaRealVal').innerText = Number(d.omegaReal).toFixed(3);
      const signV = d.voltajeEstimado > 0 ? '+' : '';
      document.getElementById('voltajeVal').innerText = signV + Number(d.voltajeEstimado).toFixed(2) + ' V';

      const dirVal = document.getElementById('dirVal');
      const dirIco = document.getElementById('dirIco');
      if (d.sentido > 0) {
        dirVal.innerText = 'Adelante';
        dirIco.innerText = '↻';
        dirIco.style.background = '#dcfce7';
        dirIco.style.color = '#059669';
      } else if (d.sentido < 0) {
        dirVal.innerText = 'Reversa';
        dirIco.innerText = '↺';
        dirIco.style.background = '#fef3c7';
        dirIco.style.color = '#d97706';
      } else {
        dirVal.innerText = 'Detenido';
        dirIco.innerText = '•';
        dirIco.style.background = '#f1f5f9';
        dirIco.style.color = '#64748b';
      }

      const pwmPct = (d.pwm * 100) / 255;
      setGauge('ringPwm', 'txtPwmPct', pwmPct);
      document.getElementById('subPwmRaw').innerText = d.pwm + ' / 255';

      const errPct = Number(d.residuoPct || 0);
      setGauge('ringErr', 'txtErrPct', errPct);
      document.getElementById('ringErr').style.stroke = errPct > 20 ? '#f59e0b' : '#10b981';
      document.getElementById('subResiduo').innerText = Number(d.residuo).toFixed(3) + ' rad/s';

      const fricPct = Number(d.indicadorFriccion || 0);
      setGauge('ringFric', 'txtFricPct', fricPct);
      document.getElementById('subFricEst').innerText =
        fricPct > 25 ? 'Carga / Fricción alta' : (fricPct > 10 ? 'Carga moderada' : 'Nominal');

      const tsB = document.getElementById('tsBadge');
      if (d.thingspeak) {
        tsB.innerText = 'TS: ACTIVO';
        tsB.className = 'chip chip-ok';
      } else {
        tsB.innerText = 'TS: INACTIVO';
        tsB.className = 'chip';
      }

      document.getElementById('revVal').innerText = Number(d.revoluciones).toFixed(3);
      document.getElementById('pcntVal').innerText = d.cuentasPCNT;
      document.getElementById('uptimeVal').innerText = formatUptime(d.uptimeMs || 0);
      document.getElementById('heapVal').innerText = (d.heapLibre / 1024).toFixed(1) + ' KB';

      // Acumular historial en vivo
      const ahora = new Date();
      const tLbl = ahora.toLocaleTimeString('es-CO', { hour12: false, minute: '2-digit', second: '2-digit' });
      histVivo.labels.push(tLbl);
      histVivo.real.push(Number(d.omegaReal));
      histVivo.gem.push(Number(d.omegaGemelo));
      if (histVivo.labels.length > MAX_PUNTOS) {
        histVivo.labels.shift();
        histVivo.real.shift();
        histVivo.gem.shift();
      }

      if (chartVel && modoGrafica === 'vivo') {
        aplicarDatosGrafica(histVivo.labels, histVivo.real, histVivo.gem);
      }
    } catch (e) {
      clearTimeout(timerAbort);
      const badge = document.getElementById('estadoBadge');
      badge.innerText = 'CAPTURANDO (ALTA VELOCIDAD)';
      badge.className = 'chip chip-busy';
      bloquearControles(true);
      siguienteMs = 650;
    }

    setTimeout(cicloEstado, siguienteMs);
  };

  window.addEventListener('DOMContentLoaded', () => {
    initChart();
    cicloEstado();
  });
})();
</script>
</body>
</html>
)=====";

String nombreEstado() {
  switch (estadoActual) {
    case REPOSO:          return "reposo";
    case MANUAL:          return "manual";
    case CAPTURA_ESCALON: return "escalon";
    case CAPTURA_BARRIDO: return "barrido";
    case CAPTURA_PRBS:    return "prbs";
  }
  return "?";
}

void handleRaiz() { server.send_P(200, "text/html", PAGINA_HTML); }

void handleEstado() {
  int32_t cuentas = leerCuentaPCNT();
  float rev = SENTIDO * (float)cuentas / CUENTAS_REV;
  float rpm = omegaReal_rads * 60.0f / (2.0f * PI);
  uint8_t pwmAct = ocupado ? pwmDuranteCaptura : pwmManualActual;
  int8_t sentido = (pwmAct == 0) ? 0 : ((ocupado ? true : direccionManualAdelante) ? 1 : -1);
  float voltajeEst = V_alimentacion_motor * (pwmAct / 255.0f) * (float)sentido;

  float magReal = fabsf(omegaReal_rads);
  float magGem  = fabsf(omegaGemelo_rads);
  float magRef  = fmaxf(magGem, magReal);

  float indicadorFriccion = 0.0f;
  if (pwmAct > 0 && magGem > 0.5f) {
    float caida = magGem - magReal;
    indicadorFriccion = (caida > 0.0f) ? constrain(100.0f * caida / magGem, 0.0f, 100.0f) : 0.0f;
  }

  float residuoRelPct = (magRef > 0.5f)
                      ? constrain(100.0f * fabsf(errorResiduo_rads) / magRef, 0.0f, 100.0f)
                      : 0.0f;

  String ipStr = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  const char* wifiModo = (WiFi.getMode() == WIFI_AP) ? "AP" : ((WiFi.status() == WL_CONNECTED) ? "STA" : "OFF");

  char json[560];
  snprintf(json, sizeof(json),
    "{\"estado\":\"%s\",\"ocupado\":%s,\"rpm\":%.1f,\"omegaReal\":%.3f,\"omegaGemelo\":%.3f,"
    "\"residuo\":%.3f,\"residuoPct\":%.1f,\"indicadorFriccion\":%.1f,\"voltajeEstimado\":%.2f,"
    "\"revoluciones\":%.3f,\"cuentasPCNT\":%ld,\"pwm\":%u,\"sentido\":%d,"
    "\"thingspeak\":%s,\"heapLibre\":%lu,\"uptimeMs\":%lu,\"ip\":\"%s\",\"wifiModo\":\"%s\","
    "\"ultimaCapturaId\":%lu}",
    nombreEstado().c_str(),
    ocupado ? "true" : "false",
    rpm,
    omegaReal_rads,
    omegaGemelo_rads,
    errorResiduo_rads,
    residuoRelPct,
    indicadorFriccion,
    voltajeEst,
    rev,
    (long)cuentas,
    (unsigned)pwmAct,
    (int)sentido,
    thingspeakActivo ? "true" : "false",
    (unsigned long)ESP.getFreeHeap(),
    (unsigned long)millis(),
    ipStr.c_str(),
    wifiModo,
    (unsigned long)ultimaCapturaId
  );
  server.send(200, "application/json", json);
}

void handleUltimaCaptura() {
  if (ocupado) {
    server.send(409, "application/json", "{\"error\":\"Ocupado\"}");
    return;
  }
  if (ultimaCapturaTipo == CAP_NINGUNA || ultimaCapturaN < 2) {
    server.send(200, "application/json", "{\"disponible\":false,\"tipo\":\"ninguna\",\"ts_ms\":0,\"muestras\":[]}");
    return;
  }

  const char* tipoStr = (ultimaCapturaTipo == CAP_ESCALON) ? "escalon" : "prbs";
  uint16_t N = ultimaCapturaN;
  if (N > MAX_MUESTRAS) N = MAX_MUESTRAS;
  const uint16_t M = (N > 120) ? 120 : N;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  char chunk[1024];
  int pos = snprintf(chunk, sizeof(chunk),
    "{\"disponible\":true,\"id\":%lu,\"tipo\":\"%s\",\"ts_ms\":%.3f,\"muestras\":[",
    (unsigned long)ultimaCapturaId, tipoStr, (float)ultimaCapturaTsUs * 1e-3f);

  float wGem = 0.0f;
  const float denom = (gemelo.R * gemelo.B + gemelo.Ka * gemelo.Km);
  const float tau_m = (denom > 1e-9f) ? (gemelo.J * gemelo.R / denom) : 0.0f;

  uint16_t idxSub    = 0;
  uint16_t objetivoK = 0;

  for (uint16_t k = 0; k < N; k++) {
    // Integración muestra a muestra de la EDO de 1er orden del gemelo digital
    if (k > 0) {
      float dt_s = (float)(buf[k].t_us - buf[k - 1].t_us) * 1e-6f;
      if (dt_s > 0.0f && tau_m > 1e-6f && denom > 1e-9f) {
        float V = V_alimentacion_motor * ((float)buf[k].pwm / 255.0f);
        float omegaSS = gemelo.Km * V / denom;
        float paso = dt_s / tau_m;
        if (paso > 1.0f) paso = 1.0f;
        wGem += paso * (omegaSS - wGem);
      }
    }

    if (k == objetivoK && idxSub < M) {
      float t_ms = (ultimaCapturaTipo == CAP_ESCALON)
                   ? (float)(buf[k].t_us - ultimaCapturaOffsetUs) * 1e-3f
                   : (float)buf[k].t_us * 1e-3f;
      float wReal = calcularRPM(k, N) * 2.0f * PI / 60.0f;

      pos += snprintf(chunk + pos, sizeof(chunk) - pos,
        "%s{\"t_ms\":%.2f,\"omegaReal\":%.3f,\"omegaGemelo\":%.3f}",
        (idxSub > 0) ? "," : "", t_ms, wReal, wGem);

      if (pos > (int)(sizeof(chunk) - 160)) {
        server.sendContent(chunk);
        pos = 0;
      }

      idxSub++;
      if (idxSub < M) {
        objetivoK = (uint32_t)idxSub * (N - 1) / (M - 1);
      }
    }
  }

  pos += snprintf(chunk + pos, sizeof(chunk) - pos, "]}");
  server.sendContent(chunk);
  server.sendContent("");
}

void handleEscalon() {
  if (ocupado) { server.send(409, "text/plain", "Ocupado"); return; }
  server.send(200, "text/plain", "OK, iniciando prueba de escalon");
  estadoActual = CAPTURA_ESCALON;
  ejecutarEscalon();
  estadoActual = REPOSO;
}

void handleBarrido() {
  if (ocupado) { server.send(409, "text/plain", "Ocupado"); return; }
  server.send(200, "text/plain", "OK, iniciando barrido 20-100%");
  estadoActual = CAPTURA_BARRIDO;
  ejecutarBarrido();
  estadoActual = REPOSO;
}

void handlePRBS() {
  if (ocupado) { server.send(409, "text/plain", "Ocupado"); return; }
  server.send(200, "text/plain", "OK, iniciando PRBS");
  estadoActual = CAPTURA_PRBS;
  ejecutarPRBS();
  estadoActual = REPOSO;
}

void handlePwm() {
  if (ocupado) { server.send(409, "text/plain", "Ocupado"); return; }
  if (!server.hasArg("valor")) { server.send(400, "text/plain", "Falta 'valor'"); return; }
  int v = server.arg("valor").toInt();
  v = constrain(v, -255, 255);
  pwmManualActual = (uint8_t)abs(v);
  direccionManualAdelante = (v >= 0);
  estadoActual = (v == 0) ? REPOSO : MANUAL;
  aplicarPWM(v);
  server.send(200, "text/plain", "OK");
}

void handleParo() {
  pwmManualActual = 0;
  aplicarPWM(0);
  estadoActual = REPOSO;
  server.send(200, "text/plain", "Parada de emergencia OK");
}

void handleThingspeak() {
  String modo = server.arg("modo");
  if (modo == "on") thingspeakActivo = true;
  else if (modo == "off") thingspeakActivo = false;
  else if (modo == "forzar") thingspeakForzar = true;
  server.send(200, "text/plain", "OK");
}

void handleConfig() {
  if (server.hasArg("ts_us")) {
    uint32_t nuevo = (uint32_t)server.arg("ts_us").toInt();
    nuevo = constrain(nuevo, TS_US_MIN, TS_US_MAX);
    TS_US = nuevo;
  }
  server.send(200, "text/plain", "Ts = " + String(TS_US) + " us");
}

void handleNotFound() { server.send(404, "text/plain", "No encontrado"); }

void configurarServidorWeb() {
  server.on("/",               handleRaiz);
  server.on("/estado",         handleEstado);
  server.on("/ultima_captura", handleUltimaCaptura);
  server.on("/escalon",        handleEscalon);
  server.on("/barrido",        handleBarrido);
  server.on("/prbs",           handlePRBS);
  server.on("/pwm",            handlePwm);
  server.on("/paro",           handleParo);
  server.on("/thingspeak",     handleThingspeak);
  server.on("/config",         handleConfig);
  server.onNotFound(handleNotFound);
  server.begin();
}

// =============================================================================
//                              WiFi: STA con fallback a AP
// =============================================================================
void conectarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Conectando a %s", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nConectado. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nNo se pudo conectar al WiFi. Iniciando punto de acceso de respaldo...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("AP activo: SSID=%s  IP=%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }
}

// =============================================================================
//                              setup() / loop()
// =============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  if (!ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES)) {
    Serial.println("#ERROR configurando PWM en ENA_PIN");
  }
  aplicarPWM(0);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  configurarPCNT();

  timerMuestreo = timerBegin(1000000);           // base de 1 MHz (1 tick = 1 us)
  timerAttachInterrupt(timerMuestreo, &isrMuestreo);
  timerAlarm(timerMuestreo, TS_US, true, 0);
  timerStop(timerMuestreo);

  conectarWiFi();
  configurarServidorWeb();

  cuentaTelemetriaAnterior = leerCuentaPCNT();
  tTelemetriaAnterior_ms   = millis();

  Serial.println("#LISTO  ('S'=escalon, 'B'=barrido, 'P'=PRBS, 'X'=parada)");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (!ocupado) {
      if (c == 'S' || c == 's') { estadoActual = CAPTURA_ESCALON; ejecutarEscalon(); estadoActual = REPOSO; }
      else if (c == 'B' || c == 'b') { estadoActual = CAPTURA_BARRIDO; ejecutarBarrido(); estadoActual = REPOSO; }
      else if (c == 'P' || c == 'p') { estadoActual = CAPTURA_PRBS; ejecutarPRBS(); estadoActual = REPOSO; }
    }
    if (c == 'X' || c == 'x') { pwmManualActual = 0; aplicarPWM(0); estadoActual = REPOSO; }
  }

  // Mientras ocupado == true (captura en curso) NO se atienden peticiones web:
  // así no se introduce jitter en el temporizador de hardware del muestreo.
  if (!ocupado) {
    server.handleClient();
    actualizarTelemetria();
  }
}