/*
 * ============================================================
 *  PerturbMax v3.1 — EMC Stress Tester
 *  Canal NFC complet regândit: IRF7470PBF + TC4420 + rezonanță
 * ============================================================
 *  Platform  : ESP32 DevKit V1 (Xtensa LX6, 240MHz, dual-core)
 *  Canal LF  : IBT-2 BTS7960B (12V/43A peak, PWM 20kHz)
 *  Canal NFC : TC4420 + IRF7470PBF + antenă rezonantă (12V, 5 moduri)
 *  Canal HF  : LEDC Timer1 GPIO33 (1kHz–100kHz, hardware pur)
 *  Core 0    : Secventa atacuri, butoane, LF loop
 *  Core 1    : NFC burst/wideband cu timing precis (dedicat)
 *
 * ============================================================
 *  PINOUT COMPLET v3.1
 * ============================================================
 *
 *  CANAL LF — IBT-2 BTS7960B:
 *  ┌────────────────┬────────────────────────────────────────┐
 *  │ ESP32 GPIO 25  │ IBT-2 RPWM (PWM 20kHz, directie +)    │   RPWM is at GPIO 6
 *  │ ESP32 GPIO 26  │ IBT-2 LPWM (PWM 20kHz, directie -)    │   LPWM is at GPIO 7
 *  │ +5V via R4/R5  │ IBT-2 R_EN + L_EN (enable permanent)  │   EN/INH is at GPIO 4   -> ENABLE/DISABLE THE CHIP
 *  │ Sursa 12V      │ IBT-2 VCC_M (din J1 XT60)             │
 *  │ GND comun      │ IBT-2 GND                              │
 *  │ IBT-2 OUT1     │ Bobina LF terminal A (J3 pin 1)        │
 *  │ IBT-2 OUT2     │ Bobina LF terminal B (J3 pin 2)        │
 *  │ D2 SB1060      │ Paralel bobina (Anod→OUT2, Catod→OUT1) │
 *  └────────────────┴────────────────────────────────────────┘
 *
 *  CANAL NFC — UPGRADE MAJOR v3.1:
 *  ┌────────────────┬────────────────────────────────────────┐
 *  │ ESP32 GPIO 32  │ R1 (100Ω) → TC4420 IN                  │    TC4420 IN is at GPIO 35
 *  │ TC4420 VDD     │ +12V (aceeasi sursa 12V)               │
 *  │ TC4420 GND     │ GND comun                              │
 *  │ TC4420 OUT     │ R2 (10Ω) → IRF7470PBF Gate            │
 *  │ IRF7470 Source │ GND comun                              │
 *  │ IRF7470 Drain  │ C10(10nF DC-block) → Antena NFC A     │
 *  │ Antena NFC B   │ +12V sursa (J4 pin 2)                  │
 *  │ Antena NFC     │ 8 ture, φ8cm, 1.37µH                   │
 *  │ C1 100pF C0G   │ Paralel antena (OBLIGATORIU, rezonanta)│
 *  │ D1 SS24        │ Flyback clamp: Anod=Drain/Catod=+12V  │
 *  │ D4 SS14        │ Freewheeling: Anod=Source/Catod=Drain  │
 *  └────────────────┴────────────────────────────────────────┘
 *
 *  CANAL HF — GPIO33 LEDC:
 *  ┌────────────────┬────────────────────────────────────────┐
 *  │ ESP32 GPIO 33  │ R3 (10Ω) → HF output terminal         │     HF output terminal at GPIO 21
 *  └────────────────┴────────────────────────────────────────┘
 *
 *  CONTROL:
 *  │ ESP32 GPIO 27  │ SW1 START → GND (INPUT_PULLUP)        │      SW START IS AT GPIO 15
 *  │ ESP32 GPIO 14  │ SW2 CANCEL → GND (INPUT_PULLUP)       │      SW CANCEL IS AT GPIO 14
 *  │ ESP32 GPIO 2   │ LED1 Status (via R6 330Ω la GND)      │      LED STATUS IS AT GPIO 2
 *
 * ============================================================
 *  DE CE IRF7470PBF + TC4420 SCHIMBA TOT:
 *
 *  BSS138 (v2.0):    200mA max → curent insuficient la 13.56MHz
 *  IRLZ44N (v3.0):   47A, RDS(on)=28mΩ — Coss ridicat, pierderi mari la 13.56MHz
 *  IRF7470PBF (v3.1): 18A, Qg=11nC, Coss=100pF — RF-friendly, pierderi minime
 *  TC4420COA gate driver: 6A peak, rise time <25ns → IRF7470PBF complet saturat
 *
 *  Cu circuit rezonant C1=100pF, L_antena=1.37µH (Q≈15–25):
 *    Frecventa rezonanta: 1/(2π√(1.37µH × 100pF)) = 13.59MHz ✓
 *    La 12V/R_coil≈0.8Ω → curent masiv in tancul LC
 *
 * ============================================================
 *  MODURI NFC v3.1 (5 moduri distincte):
 *  F: PURE      — carrier pur 13.56MHz, putere maxima
 *  G: SWEEP     — sweep 13.40→13.72MHz (detune rezonanta)
 *  H: BURST_100K — 13.56MHz ON/OFF la 100kHz (ASK 100kHz)
 *  I: BURST_200K — 13.56MHz ON/OFF la ~200kHz (ASK rapid)
 *  J: WIDEBAND  — cicleaza 7 frecvente:
 *                 13.0/13.2/13.35/13.56/13.7/13.9/14.1MHz
 * ============================================================
 *
 *  33 ATACURI A→Z + 27-33 (~33 secunde total la SPEED_DIV=3):
 *  LF [A-E] + NFC [F-J] + HF [K-P] + COMBO [Q-T] + STRESS [U-Y] + EMC_AVANSAT [Z,27-33]
 * ============================================================
 */

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// =================== PINOUT ==========================
#define LF_RPWM         25   // IBT-2 RPWM
#define LF_LPWM         26   // IBT-2 LPWM
#define NFC_PIN         32   // R1 (100Ω) → TC4420 IN
#define HF_PIN          33   // R3 (10Ω) → HF output
#define BUTTON_START    27   // SW1 → GND, INPUT_PULLUP
#define BUTTON_CANCEL   14   // SW2 → GND, INPUT_PULLUP
#define LED_PIN          2   // LED1 via R6 330Ω, activ HIGH

// =================== LEDC NFC (Timer0, Ch0) ==========
#define NFC_LEDC_CH    LEDC_CHANNEL_0
#define NFC_LEDC_TIMER LEDC_TIMER_0
#define NFC_LEDC_MODE  LEDC_HIGH_SPEED_MODE
#define NFC_FREQ_HZ    13560000UL
#define NFC_RES        LEDC_TIMER_2_BIT    // duty=2 → 50% din 4 counts

// =================== LEDC HF (Timer1, Ch1) ===========
#define HF_LEDC_CH     LEDC_CHANNEL_1
#define HF_LEDC_TIMER  LEDC_TIMER_1
#define HF_LEDC_MODE   LEDC_HIGH_SPEED_MODE
#define HF_RES         LEDC_TIMER_8_BIT    // duty=128 → 50%

// =================== LEDC LF PWM (Timer2, Ch2+3) =====
#define LF_RPWM_CH     LEDC_CHANNEL_2
#define LF_LPWM_CH     LEDC_CHANNEL_3
#define LF_PWM_TIMER   LEDC_TIMER_2
#define LF_PWM_MODE    LEDC_HIGH_SPEED_MODE
#define LF_PWM_FREQ    20000UL             // 20kHz carrier IBT-2

// =================== NIVELE PUTERE LF ================
#define PWR_LOW        64    // 25%  (64/255)
#define PWR_MEDIUM     128   // 50%  (128/255)
#define PWR_HIGH       192   // 75%  (192/255)
#define PWR_MAX        255   // 100% (255/255)

// =================== NFC SWEEP PARAMS ================
#define NFC_SWEEP_START  13400000UL
#define NFC_SWEEP_END    13720000UL
static const uint32_t NFC_WIDE_FREQS[] = {
  13000000UL, 13200000UL, 13350000UL,
  13560000UL,
  13700000UL, 13900000UL, 14100000UL
};
#define NFC_WIDE_COUNT  7

// =================== CONFIGURARE =====================
#define SERIAL_BAUD    115200
#define DEBOUNCE_MS    50
#define SUCCESS_BLINK  5

// ─── VITEZA SECVENTA ──────────────────────────────────
// SPEED_DIV = 1 → durate originale (~99s total)
// SPEED_DIV = 2 → durate injumatatite (~50s total)
// SPEED_DIV = 3 → durate la 1/3 (~33s total)
// SPEED_DIV = 5 → ultra-rapid (~20s total)
#define SPEED_DIV       3   // <─ MODIFICA AICI (3 = ~33s total, optim)
#define ATK_DUR_MIN   150   // durata minima per atac (ms) — sub 150ms efectul e neglijabil
#define BETWEEN_ATK_MS (400 / SPEED_DIV)   // pauza intre atacuri scalata automat
// LED circuit: GPIO2 HIGH = LED ON (activ HIGH, via R6 330Ω la GND)
// LED_ACTIVE = HIGH (nivel logic pentru LED aprins)
// LED_IDLE   = LOW  (nivel logic pentru LED stins)
#define LED_ACTIVE     HIGH
#define LED_IDLE       LOW

// =================== TIPURI ==========================
enum HWChannel {
  LF_ONLY,         // IBT-2 bidirectional PWM
  NFC_PURE,        // IRF7470PBF + TC4420, carrier pur 13.56MHz
  NFC_SWEEP_V,     // Sweep 13.40MHz→13.72MHz (detune)
  NFC_BURST_100K,  // Burst ON/OFF 100kHz (Core 1)
  NFC_BURST_200K,  // Burst ON/OFF ~200kHz (Core 1)
  NFC_WIDEBAND,    // Wideband 13.0→14.1MHz, 7 frecvente (Core 1)
  HF_ONLY,         // LEDC Timer1 GPIO33
  LF_NFC,          // LF + NFC simultan
  LF_HF,           // LF + HF simultan
  NFC_HF,          // NFC + HF simultan
  LF_NFC_HF,       // TRIPLU: LF + NFC + HF
  LF_SWEEP,        // LF sweep 50Hz→1kHz
  LF_NFC_SWEEP,    // LF + NFC sweep simultan
  // --- NOU v3.1: moduri EMC avansate ---
  NFC_ASK_1K,      // 13.56MHz ASK 100% adancime @ 1kHz  (500µs ON/OFF) — confuzie Manchester
  NFC_ASK_2K,      // 13.56MHz ASK 100% adancime @ 2kHz  (250µs ON/OFF)
  NFC_ASK_5K,      // 13.56MHz ASK 100% adancime @ 5kHz  (100µs ON/OFF)
  NFC_ASK_10K,     // 13.56MHz ASK 100% adancime @ 10kHz (50µs ON/OFF)
  NFC_BURST_50HZ,  // Burst cu rampa: 5ms ON / 15ms OFF @ 50Hz  — depaseste watchdog cititor
  NFC_BURST_100HZ, // Burst cu rampa: 2ms ON / 8ms OFF  @ 100Hz
  NFC_BURST_200HZ, // Burst cu rampa: 1ms ON / 4ms OFF  @ 200Hz
  NFC_SWEEP_FULL   // Sweep fin 13.0-14.0MHz, pas 20kHz (51 pasi) — prinde toate off-tune
};

struct Attack {
  const char* name;
  uint32_t    lfFreqHz;
  uint32_t    hfFreqHz;
  int         durationMs;
  uint8_t     lfPower;
  HWChannel   channel;
};

const Attack ATTACKS[] = {
  // ─── LF (IBT-2, putere graduata) ───────────────────
  { "LF 50Hz   LOW  (25%)",        50,      0,  1000, PWR_LOW,    LF_ONLY       },  // A
  { "LF 125Hz  MED  (50%)",       125,      0,  1000, PWR_MEDIUM, LF_ONLY       },  // B
  { "LF 247Hz  MED  (50%)",       247,      0,   600, PWR_MEDIUM, LF_ONLY       },  // C
  { "LF 500Hz  HIGH (75%)",       500,      0,   400, PWR_HIGH,   LF_ONLY       },  // D
  { "LF 1kHz   HIGH (75%)",      1000,      0,   200, PWR_HIGH,   LF_ONLY       },  // E

  // ─── NFC (IRF7470PBF+TC4420, 5 moduri distincte) ───
  { "NFC PURE  13.56MHz",           0,      0,  2000, 0,          NFC_PURE      },  // F
  { "NFC SWEEP 13.40->13.72MHz",    0,      0,  3000, 0,          NFC_SWEEP_V   },  // G
  { "NFC BURST 100kHz ASK",         0,      0,  2000, 0,          NFC_BURST_100K},  // H
  { "NFC BURST 200kHz rapid",       0,      0,  2000, 0,          NFC_BURST_200K},  // I
  { "NFC WIDEBAND 13.0-14.1MHz",    0,      0,  3000, 0,          NFC_WIDEBAND  },  // J

  // ─── HF GPIO LEDC (hardware pur) ───────────────────
  { "HF GPIO  1kHz",                0,   1000,   500, 0,          HF_ONLY       },  // K
  { "HF GPIO  5kHz",                0,   5000,   400, 0,          HF_ONLY       },  // L
  { "HF GPIO  10kHz",               0,  10000,   300, 0,          HF_ONLY       },  // M
  { "HF GPIO  25kHz",               0,  25000,   200, 0,          HF_ONLY       },  // N
  { "HF GPIO  50kHz",               0,  50000,   150, 0,          HF_ONLY       },  // O
  { "HF GPIO 100kHz",               0, 100000,   100, 0,          HF_ONLY       },  // P

  // ─── COMBO (canale simultane) ───────────────────────
  { "COMBO LF125 MED + NFC PURE", 125,      0,  2000, PWR_MEDIUM, LF_NFC        },  // Q
  { "COMBO LF500 HIGH + HF10k",   500,  10000,  1000, PWR_HIGH,   LF_HF         },  // R
  { "COMBO NFC PURE + HF50k",       0,  50000,  1500, 0,          NFC_HF        },  // S
  { "TRIPLU LF HIGH+NFC+HF5k",   125,   5000,  2000, PWR_HIGH,   LF_NFC_HF     },  // T

  // ─── STRESS TESTS ───────────────────────────────────
  { "SWEEP LF 50->1kHz MED",        0,      0,  5000, PWR_MEDIUM, LF_SWEEP      },  // U
  { "SWEEP LF 50->1kHz MAX",        0,      0,  5000, PWR_MAX,    LF_SWEEP      },  // V
  { "SUSTAINED 125Hz MAX 10s",     125,     0, 10000, PWR_MAX,    LF_ONLY       },  // W
  { "COMBO LF125MAX+NFC SWEEP",    125,     0,  5000, PWR_MAX,    LF_NFC_SWEEP  },  // X
  { "FULL STRESS MAX+NFC+HF100k", 125, 100000,  5000, PWR_MAX,    LF_NFC_HF     },  // Y

  // ─── NOU v3.1: EMC AVANSAT ──────────────────────────
  { "NFC ASK 100% 1kHz  (500µs)", 0,      0,  3000, 0,          NFC_ASK_1K    },  // Z
  { "NFC ASK 100% 2kHz  (250µs)", 0,      0,  3000, 0,          NFC_ASK_2K    },  // 27
  { "NFC ASK 100% 5kHz  (100µs)", 0,      0,  3000, 0,          NFC_ASK_5K    },  // 28
  { "NFC ASK 100% 10kHz (50µs)",  0,      0,  3000, 0,          NFC_ASK_10K   },  // 29
  { "BURST+RAMP 50Hz  5ms/15ms",  0,      0,  5000, 0,          NFC_BURST_50HZ  },  // 30
  { "BURST+RAMP 100Hz 2ms/8ms",   0,      0,  5000, 0,          NFC_BURST_100HZ },  // 31
  { "BURST+RAMP 200Hz 1ms/4ms",   0,      0,  5000, 0,          NFC_BURST_200HZ },  // 32
  { "SWEEP FIN 13.0-14.0MHz 20k", 0,      0,  5000, 0,          NFC_SWEEP_FULL  },  // 33
};
const int NUM_ATTACKS = sizeof(ATTACKS) / sizeof(ATTACKS[0]);

// =================== FORWARD DECLARATIONS ============
void nfcLedcInit();
void hfLedcInit(uint32_t freqHz);
void lfPwmInit();
void nfcStart();
void nfcStop();
void hfStart(uint32_t freqHz);
void hfStop();
void lfPositive(uint8_t p);
void lfNegative(uint8_t p);
void lfStop();
void stopCore1NFC();
void waitButtonRelease(int pin);
void nfcRampStart();
bool generateNFCBurstRepeat(int onMs, int offMs, int durationMs);
bool generateNFCSweepFull(int durationMs);
void printBanner();
void setReady();

// =================== CORE 1 NFC TASK =================
// Mode: 0=stop, 1=burst_100k, 2=burst_200k, 3=wideband, 4=nfc_sweep_combo
volatile uint8_t  core1_nfc_mode  = 0;
volatile bool     core1_nfc_stop  = false;
TaskHandle_t      core1Handle     = NULL;

void IRAM_ATTR nfcCore1Task(void* param) {
  int wideIdx = 0;

  while (!core1_nfc_stop) {
    switch (core1_nfc_mode) {

      case 1: // BURST 100kHz — ON 5µs / OFF 5µs = perioda 10µs = 100kHz
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(5);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(5);
        break;

      case 2: // BURST ~200kHz — ON 3µs / OFF 2µs = perioda 5µs ≈ 200kHz
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(3);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(2);
        break;

      case 3: // WIDEBAND — cicleaza 7 frecvente (13.0/13.2/13.35/13.56/13.7/13.9/14.1MHz)
        ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_WIDE_FREQS[wideIdx]);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(400);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(50);
        wideIdx = (wideIdx + 1) % NFC_WIDE_COUNT;
        break;

      case 4: // NFC SWEEP continuu — 13.40→13.72MHz, pas 40kHz, 500µs/pas
        {
          uint32_t f = NFC_SWEEP_START;
          while (f <= NFC_SWEEP_END && !core1_nfc_stop) {
            ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, f);
            ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
            ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
            ets_delay_us(500);
            f += 40000;
          }
          // Reset frecventa la 13.56MHz dupa un ciclu complet
          ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
        }
        break;

      // ─── ASK 1kHz: 500µs ON / 500µs OFF ─────────────────
      case 5:
        ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 1);   // rampa: 25%
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(1);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);   // full: 50%
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(500);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);   // OFF
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(500);
        break;

      // ─── ASK 2kHz: 250µs ON / 250µs OFF ─────────────────
      case 6:
        ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 1);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(1);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(250);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(250);
        break;

      // ─── ASK 5kHz: 100µs ON / 100µs OFF ─────────────────
      case 7:
        ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 1);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(1);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(100);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(100);
        break;

      // ─── ASK 10kHz: 50µs ON / 50µs OFF ──────────────────
      case 8:
        ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 1);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(1);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(50);
        ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
        ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
        ets_delay_us(50);
        break;

      default:
        vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  // Cleanup la iesire
  ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
  ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
  ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
  vTaskDelete(NULL);
}

void startCore1NFC(uint8_t mode) {
  core1_nfc_stop = false;
  core1_nfc_mode = mode;
  core1Handle    = NULL;
  xTaskCreatePinnedToCore(
    nfcCore1Task, "nfc_core1",
    4096, NULL, 2,
    &core1Handle, 1  // pinned to Core 1
  );
}

void stopCore1NFC() {
  core1_nfc_stop = true;
  core1_nfc_mode = 0;
  // Asteapta task-ul sa se opreasca (max 200ms)
  for (int i = 0; i < 200; i++) {
    if (core1Handle == NULL || eTaskGetState(core1Handle) == eDeleted) break;
    delay(1);
  }
  core1Handle = NULL;  // FIX: reseteaza handle-ul dupa delete (evita acces la memorie eliberata)
  nfcStop();
}

// =================== STARE GLOBALA ===================
enum State { READY, ATTACKING, SUCCESS, CANCELLED };
volatile State currentState  = READY;
volatile bool  cancelRequest = false;

// =================== SETUP ===========================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  pinMode(NFC_PIN,       OUTPUT);
  pinMode(HF_PIN,        OUTPUT);
  pinMode(BUTTON_START,  INPUT_PULLUP);
  pinMode(BUTTON_CANCEL, INPUT_PULLUP);
  pinMode(LED_PIN,       OUTPUT);
  digitalWrite(LED_PIN, LED_IDLE);

  // IMPORTANT: Initializeaza LEDC INAINTE de apelarea functiilor stop
  nfcLedcInit();
  hfLedcInit(1000);
  lfPwmInit();

  // Acum e sigur sa oprim toate canalele
  nfcStop();
  hfStop();
  lfStop();

  printBanner();
  setReady();
}

// =================== LOOP ============================
void loop() {
  if (digitalRead(BUTTON_CANCEL) == LOW) {
    delay(DEBOUNCE_MS);
    if (digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true;
      waitButtonRelease(BUTTON_CANCEL);
    }
  }
  if (digitalRead(BUTTON_START) == LOW) {
    delay(DEBOUNCE_MS);
    if (digitalRead(BUTTON_START) == LOW) {
      waitButtonRelease(BUTTON_START);
      if (currentState != ATTACKING) runFullSequence();
    }
  }
}

// =================== LEDC INIT =======================
void nfcLedcInit() {
  ledc_timer_config_t t = {
    .speed_mode      = NFC_LEDC_MODE,
    .duty_resolution = NFC_RES,
    .timer_num       = NFC_LEDC_TIMER,
    .freq_hz         = NFC_FREQ_HZ,
    .clk_cfg         = LEDC_AUTO_CLK
  };
  ledc_timer_config(&t);
  ledc_channel_config_t ch = {
    .gpio_num   = NFC_PIN,
    .speed_mode = NFC_LEDC_MODE,
    .channel    = NFC_LEDC_CH,
    .timer_sel  = NFC_LEDC_TIMER,
    .duty       = 0, .hpoint = 0
  };
  ledc_channel_config(&ch);
}

void hfLedcInit(uint32_t freqHz) {
  ledc_timer_config_t t = {
    .speed_mode      = HF_LEDC_MODE,
    .duty_resolution = HF_RES,
    .timer_num       = HF_LEDC_TIMER,
    .freq_hz         = freqHz,
    .clk_cfg         = LEDC_AUTO_CLK
  };
  ledc_timer_config(&t);
  ledc_channel_config_t ch = {
    .gpio_num   = HF_PIN,
    .speed_mode = HF_LEDC_MODE,
    .channel    = HF_LEDC_CH,
    .timer_sel  = HF_LEDC_TIMER,
    .duty       = 0, .hpoint = 0
  };
  ledc_channel_config(&ch);
}

void lfPwmInit() {
  ledc_timer_config_t t = {
    .speed_mode      = LF_PWM_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num       = LF_PWM_TIMER,
    .freq_hz         = LF_PWM_FREQ,
    .clk_cfg         = LEDC_AUTO_CLK
  };
  ledc_timer_config(&t);
  ledc_channel_config_t rpwm = {
    .gpio_num   = LF_RPWM,
    .speed_mode = LF_PWM_MODE,
    .channel    = LF_RPWM_CH,
    .timer_sel  = LF_PWM_TIMER,
    .duty       = 0, .hpoint = 0
  };
  ledc_channel_config(&rpwm);
  ledc_channel_config_t lpwm = {
    .gpio_num   = LF_LPWM,
    .speed_mode = LF_PWM_MODE,
    .channel    = LF_LPWM_CH,
    .timer_sel  = LF_PWM_TIMER,
    .duty       = 0, .hpoint = 0
  };
  ledc_channel_config(&lpwm);
}

// =================== NFC CONTROL =====================
void nfcStart() {
  ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
  ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
  ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
}

void nfcStop() {
  ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
  ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
  ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
  digitalWrite(NFC_PIN, LOW);
}

// =================== HF CONTROL ======================
void hfStart(uint32_t freqHz) {
  ledc_set_freq(HF_LEDC_MODE, HF_LEDC_TIMER, freqHz);
  ledc_set_duty(HF_LEDC_MODE, HF_LEDC_CH, 128);  // 50% duty
  ledc_update_duty(HF_LEDC_MODE, HF_LEDC_CH);
}
void hfStop() {
  ledc_set_duty(HF_LEDC_MODE, HF_LEDC_CH, 0);
  ledc_update_duty(HF_LEDC_MODE, HF_LEDC_CH);
  digitalWrite(HF_PIN, LOW);
}

// =================== LF CONTROL (IBT-2 PWM) ==========
void lfPositive(uint8_t p) {
  ledc_set_duty(LF_PWM_MODE, LF_RPWM_CH, p);
  ledc_update_duty(LF_PWM_MODE, LF_RPWM_CH);
  ledc_set_duty(LF_PWM_MODE, LF_LPWM_CH, 0);
  ledc_update_duty(LF_PWM_MODE, LF_LPWM_CH);
}
void lfNegative(uint8_t p) {
  ledc_set_duty(LF_PWM_MODE, LF_RPWM_CH, 0);
  ledc_update_duty(LF_PWM_MODE, LF_RPWM_CH);
  ledc_set_duty(LF_PWM_MODE, LF_LPWM_CH, p);
  ledc_update_duty(LF_PWM_MODE, LF_LPWM_CH);
}
void lfStop() {
  ledc_set_duty(LF_PWM_MODE, LF_RPWM_CH, 0);
  ledc_update_duty(LF_PWM_MODE, LF_RPWM_CH);
  ledc_set_duty(LF_PWM_MODE, LF_LPWM_CH, 0);
  ledc_update_duty(LF_PWM_MODE, LF_LPWM_CH);
}

// =================== GENERATOR LF ====================
bool generateLF(uint32_t freqHz, int durationMs, uint8_t power) {
  if (freqHz == 0) { delay(durationMs); return !cancelRequest; }
  long halfUs = 500000L / (long)freqHz;
  unsigned long startMs = millis();
  while ((long)(millis() - startMs) < durationMs) {
    if (cancelRequest || digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true; lfStop(); return false;
    }
    lfPositive(power);
    delayMicroseconds(halfUs);
    lfNegative(power);
    delayMicroseconds(halfUs);
  }
  lfStop();
  return true;
}

// =================== GENERATOR LF SWEEP ==============
bool generateLFSweep(int durationMs, uint8_t power) {
  const uint32_t F0 = 50, F1 = 1000;
  unsigned long startMs = millis();
  unsigned long elapsed;
  while (true) {
    elapsed = millis() - startMs;
    if ((long)elapsed >= durationMs) break;
    if (cancelRequest || digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true; lfStop(); return false;
    }
    uint32_t f = F0 + (uint32_t)((F1 - F0) * elapsed / (unsigned long)durationMs);
    if (f < F0) f = F0;
    if (f > F1) f = F1;
    long halfUs = 500000L / (long)f;
    lfPositive(power);
    delayMicroseconds(halfUs);
    lfNegative(power);
    delayMicroseconds(halfUs);
  }
  lfStop();
  return true;
}

// =================== GENERATOR NFC PURE ==============
bool generateNFCPure(int durationMs) {
  nfcStart();
  unsigned long startMs = millis();
  while ((long)(millis() - startMs) < durationMs) {
    if (cancelRequest || digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true; nfcStop(); return false;
    }
    delay(10);
  }
  nfcStop();
  return true;
}

// =================== GENERATOR NFC SWEEP =============
bool generateNFCSweep(int durationMs) {
  unsigned long startMs = millis();
  while ((long)(millis() - startMs) < durationMs) {
    if (cancelRequest || digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true; nfcStop();
      ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
      return false;
    }
    // Un ciclu complet de sweep la fiecare iteratie
    for (uint32_t f = NFC_SWEEP_START; f <= NFC_SWEEP_END; f += 40000) {
      if (cancelRequest) break;
      ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, f);
      ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
      ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
      delayMicroseconds(600);
    }
  }
  nfcStop();
  ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
  return true;
}

// =================== GENERATOR NFC CORE1 ============
bool generateNFCCore1(uint8_t mode, int durationMs) {
  startCore1NFC(mode);
  unsigned long startMs = millis();
  while ((long)(millis() - startMs) < durationMs) {
    if (cancelRequest || digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true;
      stopCore1NFC();
      return false;
    }
    delay(10);
  }
  stopCore1NFC();
  return true;
}

// =================== GENERATOR HF ====================
bool generateHF(uint32_t freqHz, int durationMs) {
  hfStart(freqHz);
  unsigned long startMs = millis();
  while ((long)(millis() - startMs) < durationMs) {
    if (cancelRequest || digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true; hfStop(); return false;
    }
    delay(10);
  }
  hfStop();
  return true;
}

// =================== GENERATOARE COMBO ===============
bool generateLF_NFC(uint32_t lfHz, int durationMs, uint8_t power) {
  nfcStart();
  bool ok = generateLF(lfHz, durationMs, power);
  nfcStop();
  return ok;
}
bool generateLF_HF(uint32_t lfHz, uint32_t hfHz, int durationMs, uint8_t power) {
  hfStart(hfHz);
  bool ok = generateLF(lfHz, durationMs, power);
  hfStop();
  return ok;
}
bool generateNFC_HF(uint32_t hfHz, int durationMs) {
  nfcStart();
  bool ok = generateHF(hfHz, durationMs);
  nfcStop();
  return ok;
}
bool generateLF_NFC_HF(uint32_t lfHz, uint32_t hfHz, int durationMs, uint8_t power) {
  nfcStart();
  hfStart(hfHz);
  bool ok = generateLF(lfHz, durationMs, power);
  nfcStop();
  hfStop();
  return ok;
}
// LF + NFC Sweep: NFC sweep pe Core 1, LF pe Core 0
bool generateLF_NFCSweep(uint32_t lfHz, int durationMs, uint8_t power) {
  startCore1NFC(4); // mode 4 = sweep continuu
  bool ok = generateLF(lfHz, durationMs, power);
  stopCore1NFC();
  return ok;
}

// =================== PULSE SHAPING (RAMPA) ===========
// Soft-start in 2 pasi: 25% duty (1 ciclu) → 50% duty (full)
// Simuleaza rampa de ridicare ~100-300ns → AGC-ul cititorului
// nu apuca sa compenseze saturatia inainte de primul burst
void nfcRampStart() {
  ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
  ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 1);  // 25% — pas 1
  ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
  ets_delay_us(1);                                // ~1µs = 13 cicli purtator
  ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);  // 50% — pas 2 (full)
  ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
}

// =================== BURST REPEAT 50/100/200Hz =======
// Fiecare burst are rampa de ridicare (nfcRampStart).
// Intervalul OFF permite watchdog-ului cititorului sa se
// reseteze partial → urmatorul burst il gaseste vulnerabil.
bool generateNFCBurstRepeat(int onMs, int offMs, int durationMs) {
  unsigned long startMs = millis();
  while ((long)(millis() - startMs) < durationMs) {
    if (cancelRequest || digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true; nfcStop(); return false;
    }
    // Burst ON cu rampa de ridicare
    nfcRampStart();
    delay(onMs);
    // Burst OFF
    ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 0);
    ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
    delay(offMs);
  }
  nfcStop();
  return true;
}

// =================== SWEEP FIN 13.0-14.0MHz ==========
// 51 pasi de 20kHz, 300µs/pas → ciclu complet ~15.3ms
// Acopera toate cititoarele NFC cu rezonanta off-tune
// (toleranta cristal ±200ppm, variatie temperatura, imbatranire)
bool generateNFCSweepFull(int durationMs) {
  const uint32_t F_START = 13000000UL;
  const uint32_t F_STOP  = 14000000UL;
  const uint32_t F_STEP  =    20000UL;  // pas 20kHz → 51 puncte
  unsigned long startMs = millis();
  while ((long)(millis() - startMs) < durationMs) {
    if (cancelRequest || digitalRead(BUTTON_CANCEL) == LOW) {
      cancelRequest = true;
      nfcStop();
      ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
      return false;
    }
    for (uint32_t f = F_START; f <= F_STOP; f += F_STEP) {
      if (cancelRequest) break;
      ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, f);
      ledc_set_duty(NFC_LEDC_MODE, NFC_LEDC_CH, 2);
      ledc_update_duty(NFC_LEDC_MODE, NFC_LEDC_CH);
      ets_delay_us(300);
    }
  }
  nfcStop();
  ledc_set_freq(NFC_LEDC_MODE, NFC_LEDC_TIMER, NFC_FREQ_HZ);
  return true;
}

// =================== SECVENTA COMPLETA ===============
void runFullSequence() {
  Serial.printf("\n======== PERTURBMAX v3.1 — SECVENTA COMPLETA (%d atacuri) ========\n", NUM_ATTACKS);
  currentState  = ATTACKING;
  cancelRequest = false;
  digitalWrite(LED_PIN, LED_ACTIVE);  // LED aprins pe durata secventei

  for (int i = 0; i < NUM_ATTACKS; i++) {
    if (cancelRequest) { setCancelled(); return; }

    const Attack& atk = ATTACKS[i];
    char labelBuf[5];
    if (i < 26) snprintf(labelBuf, 5, "%c", 'A' + i);
    else        snprintf(labelBuf, 5, "%d", i + 1);
    uint8_t pct = (atk.lfPower > 0) ? (uint8_t)((uint32_t)atk.lfPower * 100 / 255) : 0;

    // Durata scalata cu SPEED_DIV, respectand minimul ATK_DUR_MIN
    int dur = atk.durationMs / SPEED_DIV;
    if (dur < ATK_DUR_MIN) dur = ATK_DUR_MIN;

    Serial.printf("[%s/%d] %-38s | %s | %dms",
      labelBuf, NUM_ATTACKS, atk.name,
      channelName(atk.channel), dur);
    if (atk.lfPower > 0)
      Serial.printf(" | LF_PWR=%d%%", pct);
    Serial.println();

    bool ok = false;
    switch (atk.channel) {
      case LF_ONLY:       ok = generateLF(atk.lfFreqHz, dur, atk.lfPower);                        break;
      case NFC_PURE:      ok = generateNFCPure(dur);                                               break;
      case NFC_SWEEP_V:   ok = generateNFCSweep(dur);                                              break;
      case NFC_BURST_100K:ok = generateNFCCore1(1, dur);                                           break;
      case NFC_BURST_200K:ok = generateNFCCore1(2, dur);                                           break;
      case NFC_WIDEBAND:  ok = generateNFCCore1(3, dur);                                           break;
      case HF_ONLY:       ok = generateHF(atk.hfFreqHz, dur);                                     break;
      case LF_NFC:        ok = generateLF_NFC(atk.lfFreqHz, dur, atk.lfPower);                    break;
      case LF_HF:         ok = generateLF_HF(atk.lfFreqHz, atk.hfFreqHz, dur, atk.lfPower);      break;
      case NFC_HF:        ok = generateNFC_HF(atk.hfFreqHz, dur);                                 break;
      case LF_NFC_HF:     ok = generateLF_NFC_HF(atk.lfFreqHz, atk.hfFreqHz, dur, atk.lfPower);  break;
      case LF_SWEEP:      ok = generateLFSweep(dur, atk.lfPower);                                  break;
      case LF_NFC_SWEEP:  ok = generateLF_NFCSweep(atk.lfFreqHz, dur, atk.lfPower);               break;
      // --- NOU v3.1 ---
      case NFC_ASK_1K:    ok = generateNFCCore1(5, dur);  break;
      case NFC_ASK_2K:    ok = generateNFCCore1(6, dur);  break;
      case NFC_ASK_5K:    ok = generateNFCCore1(7, dur);  break;
      case NFC_ASK_10K:   ok = generateNFCCore1(8, dur);  break;
      case NFC_BURST_50HZ:  ok = generateNFCBurstRepeat(5,  15, dur); break;
      case NFC_BURST_100HZ: ok = generateNFCBurstRepeat(2,   8, dur); break;
      case NFC_BURST_200HZ: ok = generateNFCBurstRepeat(1,   4, dur); break;
      case NFC_SWEEP_FULL:  ok = generateNFCSweepFull(dur);            break;
    }

    if (!ok) { setCancelled(); return; }
    if (i < NUM_ATTACKS - 1) delay(BETWEEN_ATK_MS);
  }
  setSuccess();
}

// =================== STARI ===========================
void setReady() {
  currentState  = READY;
  cancelRequest = false;
  lfStop(); nfcStop(); hfStop();
  Serial.printf("\n[READY] Apasa START — secventa completa %d atacuri A-Z+27-33 (~%ds).\n",
    NUM_ATTACKS, (NUM_ATTACKS * 3) / SPEED_DIV);
  // 3 clipiri LED la intrarea in starea READY
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LED_ACTIVE); delay(200);
    digitalWrite(LED_PIN, LED_IDLE);   delay(200);
  }
}
void setSuccess() {
  currentState = SUCCESS;
  lfStop(); nfcStop(); hfStop();
  Serial.printf("\n[SUCCESS] Toate cele %d atacuri finalizate.\n", NUM_ATTACKS);
  // SUCCESS_BLINK clipiri rapide
  for (int i = 0; i < SUCCESS_BLINK; i++) {
    digitalWrite(LED_PIN, LED_ACTIVE); delay(100);
    digitalWrite(LED_PIN, LED_IDLE);   delay(100);
  }
  delay(2000);
  setReady();
}
void setCancelled() {
  lfStop(); nfcStop(); hfStop(); stopCore1NFC();
  currentState  = CANCELLED;
  cancelRequest = false;
  Serial.println("\n[CANCEL] Oprit de utilizator.");
  // 1 puls lung LED = semnalizare anulare
  digitalWrite(LED_PIN, LED_ACTIVE); delay(800);
  digitalWrite(LED_PIN, LED_IDLE);   delay(400);
  setReady();
}

// =================== HELPERS =========================
void waitButtonRelease(int pin) {
  while (digitalRead(pin) == LOW) delay(10);
  delay(DEBOUNCE_MS);
}
const char* channelName(HWChannel ch) {
  switch(ch) {
    case LF_ONLY:        return "LF        (IBT-2 PWM)      ";
    case NFC_PURE:       return "NFC PURE  (IRF7470 13.56M) ";
    case NFC_SWEEP_V:    return "NFC SWEEP (13.40->13.72MHz)";
    case NFC_BURST_100K: return "NFC BURST (100kHz ASK)     ";
    case NFC_BURST_200K: return "NFC BURST (~200kHz rapid)  ";
    case NFC_WIDEBAND:   return "NFC WIDE  (13.0->14.1MHz)  ";
    case HF_ONLY:        return "HF        (LEDC GPIO33)    ";
    case LF_NFC:         return "LF+NFC    (COMBO)          ";
    case LF_HF:          return "LF+HF     (COMBO)          ";
    case NFC_HF:         return "NFC+HF    (COMBO)          ";
    case LF_NFC_HF:      return "TRIPLU    (LF+NFC+HF)      ";
    case LF_SWEEP:       return "LF SWEEP  (50Hz->1kHz)     ";
    case LF_NFC_SWEEP:   return "LF+NFCSweep (DUAL SWEEP)   ";
    case NFC_ASK_1K:     return "NFC ASK 100% 1kHz (Core1)  ";
    case NFC_ASK_2K:     return "NFC ASK 100% 2kHz (Core1)  ";
    case NFC_ASK_5K:     return "NFC ASK 100% 5kHz (Core1)  ";
    case NFC_ASK_10K:    return "NFC ASK 100% 10kHz (Core1) ";
    case NFC_BURST_50HZ: return "BURST+RAMP 50Hz  5ms/15ms  ";
    case NFC_BURST_100HZ:return "BURST+RAMP 100Hz 2ms/8ms   ";
    case NFC_BURST_200HZ:return "BURST+RAMP 200Hz 1ms/4ms   ";
    case NFC_SWEEP_FULL: return "SWEEP FIN 13.0-14.0MHz 20k ";
    default:             return "???                        ";
  }
}
void printBanner() {
  Serial.println("=============================================================");
  Serial.println("  PerturbMax v3.1 — EMC Stress Tester");
  Serial.println("  Canal LF  : IBT-2 BTS7960B (12V/43A peak, PWM 20kHz)");
  Serial.println("  Canal NFC : TC4420COA + IRF7470PBF (12V, 5 moduri)");
  Serial.println("              PURE / SWEEP 13.40-13.72MHz / BURST 100k+~200k");
  Serial.println("              WIDEBAND 13.0-14.1MHz, 7 frecvente (Core 1)");
  Serial.println("  Canal HF  : LEDC Timer1, GPIO33 (1kHz-100kHz, 0% CPU)");
  Serial.println("=============================================================");
  Serial.printf( "  Atacuri: %d total (LF:5, NFC:13, HF:6, Combo:4, Stress:5)\n",
    NUM_ATTACKS);
  Serial.printf( "  Durata estimata: ~%d secunde (SPEED_DIV=%d)\n", (NUM_ATTACKS * 3) / SPEED_DIV, SPEED_DIV);
  Serial.println("  START  (GPIO27) = secventa completa A-Z+27-33");
  Serial.println("  CANCEL (GPIO14) = oprire imediata (toate canalele + Core 1)");
  Serial.println("=============================================================");
}
