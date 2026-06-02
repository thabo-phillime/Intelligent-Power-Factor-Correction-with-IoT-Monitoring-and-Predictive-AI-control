#include <LiquidCrystal_I2C.h>
#include "pfc_model.h"

// ---------------- Pin Definitions ----------------
#define V_ADC_PIN   36
#define I_ADC_PIN   39
#define RELAY1_PIN  4
#define RELAY2_PIN  5
#define RELAY3_PIN  16
#define RELAY4_PIN  17
#define LED_RELAY1  14
#define LED_RELAY2  15
#define LED_RELAY3  18
#define LED_RELAY4  19
#define DEBUG_LED   13
#define BTN_INC_PIN 25
#define BTN_DEC_PIN 26

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- Globals ----------------
uint16_t sim_angle_tenths = 500;   // Start 50.0°
const uint16_t ANGLE_STEP_TENTHS = 50; // 5.0° per press
const uint16_t ANGLE_MIN = 0;
const uint16_t ANGLE_MAX = 900;

uint32_t last_inc = 0, last_dec = 0;
const uint32_t DEBOUNCE_MS = 120;

// Cosine table
const int16_t cos1000[181] = {
  1000,1000,999,999,998,996,995,993,990,988,985,982,978,974,970,966,961,956,951,946,
  940,934,927,921,914,906,899,891,883,875,866,857,848,839,829,819,809,799,788,777,
  766,755,743,731,719,707,695,682,669,656,643,629,616,602,588,574,559,545,530,515,
  500,485,469,454,438,423,407,391,375,358,342,326,309,292,276,259,242,225,208,191,
  174,156,139,122,105,87,70,52,35,17,0,-17,-35,-52,-70,-87,-105,-122,-139,-156,
  -174,-191,-208,-225,-242,-259,-276,-292,-309,-326,-342,-358,-375,-391,-407,-423,
  -438,-454,-469,-485,-500,-515,-530,-545,-559,-574,-588,-602,-616,-629,-643,-656,
  -669,-682,-695,-707,-719,-731,-743,-755,-766,-777,-788,-799,-809,-819,-829,-839,
  -848,-857,-866,-875,-883,-891,-899,-906,-914,-921,-927,-934,-940,-946,-951,-956,
  -961,-966,-970,-974,-978,-982,-985,-988,-990,-993,-995,-996,-998,-999,-999,-1000,-1000
};

// ---------------- Core Functions ----------------
int16_t pf_from_angle(int16_t tenths) {
  tenths = abs(tenths);
  if (tenths > 1800) tenths = 1800;
  uint16_t deg = tenths / 10;
  uint16_t frac = tenths % 10;
  int16_t c0 = cos1000[deg];
  int16_t c1 = cos1000[deg + 1];
  return c0 + ((c1 - c0) * frac) / 10;
}

// fallback
float compute_qvar_test(float pf) {
  float q = (1.0 - pf) * 1000.0;  // scales with angle
  if (q < 0) q = 0;
  return q;
}

// Model prediction 
float predict_qvar(uint16_t v_adc, uint16_t i_adc, uint16_t angle_tenths) {
  int16_t features[3];
  features[0] = (int16_t)v_adc;
  features[1] = (int16_t)i_adc;
  features[2] = (int16_t)angle_tenths;
  float q = pfc_predictor_predict(features, 3); //calling predictor
  if (q < 1) q = compute_qvar_test((float)pf_from_angle(angle_tenths) / 1000.0); // fallback
  return q;
}

uint8_t set_relays(float q_var) {
  uint8_t caseval = 0;
  int q_int = (int)q_var;

  // 1. Map Q_VAR to a case index (0-15).
  uint8_t case_index = (q_int / 100);

  // Clamp the index to 0-15 (0000 to 1111)
  if (case_index > 15) {
    case_index = 15;
  }
  
  caseval = case_index;

  // 2. Set the relays based on the caseval (0 to 15)
  // Relay 1 is LSB (0b0001), Relay 4 is MSB (0b1000)

  // Relay 1 (bit 0): ON if caseval & 0b0001
  digitalWrite(RELAY1_PIN, caseval & 0b0001);
  digitalWrite(LED_RELAY1, caseval & 0b0001);

  // Relay 2 (bit 1): ON if caseval & 0b0010
  digitalWrite(RELAY2_PIN, caseval & 0b0010);
  digitalWrite(LED_RELAY2, caseval & 0b0010);

  // Relay 3 (bit 2): ON if caseval & 0b0100
  digitalWrite(RELAY3_PIN, caseval & 0b0100);
  digitalWrite(LED_RELAY3, caseval & 0b0100);

  // Relay 4 (bit 3): ON if caseval & 0b1000
  digitalWrite(RELAY4_PIN, caseval & 0b1000);
  digitalWrite(LED_RELAY4, caseval & 0b1000);

  return caseval;
}

// Proper CB binary print
void to_binary_str(uint8_t val, char *out) {
  for (int i = 3; i >= 0; i--) out[3 - i] = (val & (1 << i)) ? '1' : '0';
  out[4] = '\0';
}

void update_lcd(uint16_t angle_tenths, int16_t pf_scaled, float q_var, uint8_t caseval) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ang:");
  lcd.print(angle_tenths / 10);
  lcd.print(".");
  lcd.print(angle_tenths % 10);
  lcd.print(" PF:");
  lcd.print((float)pf_scaled / 1000.0, 2);

  lcd.setCursor(0, 1);
  lcd.print("Q:");
  lcd.print((int)q_var*16.325,0);
  lcd.print(" CB:");
  char cb_str[5];
  to_binary_str(caseval, cb_str);
  lcd.print(cb_str);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(V_ADC_PIN, INPUT);
  pinMode(I_ADC_PIN, INPUT);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  pinMode(LED_RELAY1, OUTPUT);
  pinMode(LED_RELAY2, OUTPUT);
  pinMode(LED_RELAY3, OUTPUT);
  pinMode(LED_RELAY4, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);

  pinMode(BTN_INC_PIN, INPUT_PULLUP);
  pinMode(BTN_DEC_PIN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.print("PFC Ready");
  delay(1000);
  lcd.clear();

  Serial.println("ESP32 Intelligent PFC (Wokwi) Ready.");
}

// ---------------- Loop ----------------
void loop() {
  digitalWrite(DEBUG_LED, HIGH);
  delay(30);
  digitalWrite(DEBUG_LED, LOW);

  uint32_t now = millis();
  if (digitalRead(BTN_INC_PIN) == LOW && now - last_inc > DEBOUNCE_MS) {
    last_inc = now;
    sim_angle_tenths = min<uint16_t>(sim_angle_tenths + ANGLE_STEP_TENTHS, ANGLE_MAX);
  }
  if (digitalRead(BTN_DEC_PIN) == LOW && now - last_dec > DEBOUNCE_MS) {
    last_dec = now;
    sim_angle_tenths = max<uint16_t>(sim_angle_tenths - ANGLE_STEP_TENTHS, ANGLE_MIN);
  }

  uint16_t angle_tenths = sim_angle_tenths;
  int16_t pf_scaled = pf_from_angle(angle_tenths);
  float pf = (float)pf_scaled / 1000.0;

  uint16_t v_adc = analogRead(V_ADC_PIN) >> 2;
  uint16_t i_adc = analogRead(I_ADC_PIN) >> 2;

  float q_var = predict_qvar(v_adc, i_adc, angle_tenths);
  uint8_t caseval = set_relays(q_var);

  update_lcd(angle_tenths, pf_scaled, q_var, caseval);

  char cb_str[5];
  to_binary_str(caseval, cb_str); 

  Serial.printf("V:%d I:%d Angle:%.1f PF:%.3f Q:%.2f CB:%s\n",
                  v_adc, i_adc, angle_tenths / 10.0, pf, q_var*16.325, cb_str);

  delay(950);
}

