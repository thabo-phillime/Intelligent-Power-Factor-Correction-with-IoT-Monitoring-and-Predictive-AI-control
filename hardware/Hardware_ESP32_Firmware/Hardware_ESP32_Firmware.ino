#define BLYNK_TEMPLATE_ID   "TMPL2zqEYWGr7"
#define BLYNK_TEMPLATE_NAME "Intelligent PFC"
#define BLYNK_AUTH_TOKEN    "VMOqO1ELi3-eCmgfX0s1XKXiiH2UqJv2"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <algorithm>
#include <math.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "MOITHUTI.NET.HD";
char pass[] = "";

#define SDA_PIN      21
#define SCL_PIN      22
#define BUTTON_PIN   32
#define XOR_PIN      33
#define ADC_PIN      35
#define RLY1         25
#define RLY2         26
#define RLY3         27
#define RED_PIN      4
#define GREEN_PIN    16
#define BLUE_PIN     17
#define BUZZER_PIN   19

#define BUZZER_FREQ_HZ      2000
#define BUZZER_RES_BITS     8
#define BUZZER_DUTY_ON      128

#define ACS712_SENSITIVITY_V_PER_A  0.185f
#define ADC_VREF                    3.3f
#define ADC_RESOLUTION              4095.0f
#define VOLTS_PER_COUNT   (ADC_VREF / ADC_RESOLUTION)
#define AMPS_PER_COUNT    (VOLTS_PER_COUNT / ACS712_SENSITIVITY_V_PER_A)

#define MAINS_VOLTAGE       19.28f
#define MAINS_FREQ_HZ       50.0f
#define HALF_PERIOD_US_NOM  10000.0f

#define MIN_CURRENT_PEAK    10
#define MAX_CURRENT_PEAK    4000
#define MIN_LAG_CURRENT_A   0.10f

#define XOR_SAMPLE_WINDOW_US   120000UL
#define XOR_MIN_PULSE_US       50UL
#define XOR_MAX_PULSE_US       9900UL
#define XOR_HP_MIN_US          7000UL
#define XOR_HP_MAX_US          13000UL
#define XOR_MIN_SAMPLES        3
#define XOR_MEAS_REPEATS       8

#define DEMO_OFFSET_SCALE        1.00f
#define DEMO_BIAS_DEG            0.0f
#define DEMO_PHASE_GAIN          18.0f
#define DEMO_ANGLE_MAX_DEG       89.0f
#define DEMO_ANGLE_FLOOR_DEG     25.0f
#define DEMO_ANGLE_ALPHA         0.45f
#define LAG_ANGLE_DEADBAND_DEG   1.0f

#define REAL_LAG_START_DEG       2.0f
#define REAL_CORRECTED_DEG       1.5f
#define REAL_LEAD_DEG            1.5f
#define BEST_BANK_REENTRY_MULT   2.5f

#define PF_GOOD_MIN         870
#define PF_GREEN_MIN        900
#define PF_YELLOW_MIN       750
#define PF_ALPHA            0.75f

#define POST_RELAY_IGNORE_MS        2500UL
#define SETTLE_TIME_MS              5000UL
#define MIN_BANK_HOLD_MS            9000UL
#define MAX_RUNTIME_MS              900000UL
#define COOLDOWN_MS                 300000UL

#define LAG_SCORE_CONFIRM        4
#define LAG_SCORE_CLEAR          1
#define LAG_WINDOW               6
#define LEAD_SCORE_CONFIRM       2
#define ZCD_UNHEALTHY_HOLDOFF_MS 1500UL

#define OVERCORRECT_CONFIRM_COUNT   3
#define RESISTIVE_HOLD_COUNT        4

#define CAL_NOISE_GOOD_US    60.0f
#define CAL_NOISE_WARN_US    120.0f
#define CAL_NOISE_REJECT_US  250.0f

#define NVS_HARDWARE_VERSION  38
#define ACS_MIN_VOLTS         0.4f
#define ACS_MAX_VOLTS         2.8f
#define ACS_MIN_RAW           1800
#define SCREEN_WIDTH          128
#define SCREEN_HEIGHT          64
#define MAIN_LOOP_MS          100UL

#define MENU_TICKER_ROW      56
#define MENU_TICKER_HOLD_MS  2000UL
#define MENU_TICKER_STEP_MS  38UL
static const char MENU_TICKER[] =
  "  1. Hold 1.5s = confirm      2. Short press = toggle      ";

struct PhaseResult {
  float    angle_raw_deg;
  float    angle_corr_signed;
  float    angle_corr_abs;
  float    angle_demo_deg;
  float    pf_real;
  float    pf_demo;
  bool     is_lagging;
  bool     is_leading;
  bool     valid;
  uint32_t pulse_width_us;
  float    freq_hz_meas;
  int      samples_used;
};

enum ModeType  { MODE_DYNAMIC=0, MODE_AI=1 };
enum FaultCode {
  FAULT_NONE=0, FAULT_XOR_ABSENT, FAULT_XOR_UNSTABLE,
  FAULT_ACS_OFFSET, FAULT_OVERCURRENT,
  FAULT_CTRL_FAILED, FAULT_NO_LOAD, FAULT_OK
};
enum CtrlState {
  CS_IDLE=0, CS_STEPPING=1, CS_HOLDING=2,
  CS_DONE=3, CS_BEST_BANK=4, CS_FAILED=5
};
enum BuzzerState { BZ_IDLE, BZ_GOOD, BZ_C1, BZ_GAP, BZ_C2 };

Preferences      prefs;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint32_t CAL_PULSE_OFFSET_US  = 0;
float    CAL_PULSE_NOISE_US   = 50.0f;
uint16_t CAL_ACS_OFFSET       = 0;
bool     calibrationDone      = false;
bool     baselineValid        = false;

ModeType  selectedMode       = MODE_DYNAMIC;
bool      modeChosen         = false;
FaultCode currentFault       = FAULT_NONE;

float     pf_demo_filtered   = 1.00f;
float     pf_real_filtered   = 1.00f;
float     real_angle_filt    = 0.0f;
float     demo_angle_smooth  = 0.0f;
int16_t   PF_scaled          = 0;
float     phaseAngleDeg      = 0.0f;
bool      noLoadDetected     = false;

bool      isLagging          = false;
bool      isLeading          = false;
bool      bestBankLocked     = false;
uint8_t   bestBank           = 0;
bool      rgbEnabled         = false;

float     currentAmps        = 0.0f;
uint16_t  currentRawPeak     = 0;
float     Q_var              = 0.0f;
float     P_watt             = 0.0f;
float     S_va               = 0.0f;

uint8_t   bank               = 0;
uint8_t   qsave              = 0;
bool      correcting         = false;
bool      targetFailed       = false;
bool      targetReached      = false;
float     correctionTime     = 0.0f;
uint8_t   correctionSteps    = 0;
unsigned long correctionStart    = 0;
unsigned long lastBankChangeMs   = 0;
unsigned long settleStart        = 0;
unsigned long postRelayEnd       = 0;
bool          inPostRelay        = false;

int       overcorrectCount   = 0;
int       resistiveCount     = 0;
int       leadCount          = 0;

uint8_t   footerPage         = 0;

bool      lagWindow[LAG_WINDOW] = {false};
int       lagWindowIdx          = 0;
int       lagScore              = 0;

BuzzerState   bzState    = BZ_IDLE;
unsigned long bzTimer    = 0;
bool          lastPfGood = false;

CtrlState ctrlState      = CS_IDLE;
float     realAngleAtStep = 0.0f;

bool      xorHealthy         = false;
bool      xorWasUnhealthy    = false;
unsigned long xorUnhealthySince = 0;

int  tickerX=0; bool tickerHolding=true;
unsigned long tickerTimer=0; int tickerMsgLen=0;

void yieldDelay(unsigned long ms) {
  unsigned long s=millis();
  while(millis()-s<ms){ yield(); delay(10); }
}

const char* ctrlStateName(CtrlState s) {
  switch(s){
    case CS_IDLE:      return "IDLE";
    case CS_STEPPING:  return "STEP";
    case CS_HOLDING:   return "HOLD";
    case CS_DONE:      return "DONE";
    case CS_BEST_BANK: return "BEST";
    case CS_FAILED:    return "FAIL";
    default:           return "????";
  }
}

void initMenuTicker() {
  tickerMsgLen=(int)strlen(MENU_TICKER)*6;
  tickerX=(SCREEN_WIDTH-tickerMsgLen)/2;
  tickerHolding=true; tickerTimer=millis();
}
void stepMenuTicker() {
  unsigned long now=millis();
  if(tickerHolding){
    if(now-tickerTimer>=MENU_TICKER_HOLD_MS){tickerHolding=false;tickerTimer=now;}
    return;
  }
  if(now-tickerTimer>=MENU_TICKER_STEP_MS){
    tickerTimer=now; tickerX++;
    if(tickerX>SCREEN_WIDTH) tickerX=-tickerMsgLen;
  }
}

void saveCalibration(uint16_t acs, uint32_t offset, float noise) {
  prefs.begin("pfc",false);
  prefs.putUShort("acs_offset",  acs);
  prefs.putULong ("pulse_offset",offset);
  prefs.putFloat ("pulse_noise", noise);
  prefs.putUChar ("cal_done",    1);
  prefs.putUChar ("hw_ver",      NVS_HARDWARE_VERSION);
  prefs.end();
}

bool loadCalibration(uint16_t &acs, uint32_t &offset, float &noise) {
  prefs.begin("pfc",true);
  uint8_t done=prefs.getUChar("cal_done",0);
  uint8_t ver =prefs.getUChar("hw_ver",  0);
  if(done==1 && ver!=NVS_HARDWARE_VERSION){
    prefs.end();
    prefs.begin("pfc",false); prefs.clear(); prefs.end();
    return false;
  }
  if(done==1){
    acs    = prefs.getUShort("acs_offset",  0);
    offset = prefs.getULong ("pulse_offset",0);
    noise  = prefs.getFloat ("pulse_noise", 50.0f);
    prefs.end();
    return true;
  }
  prefs.end();
  return false;
}

void evaluateFaults() {
  currentFault=FAULT_OK;
  if(!xorHealthy && !inPostRelay) currentFault=FAULT_XOR_ABSENT;
  else if(currentRawPeak>MAX_CURRENT_PEAK) currentFault=FAULT_OVERCURRENT;
  else if(targetFailed)    currentFault=FAULT_CTRL_FAILED;
  else if(noLoadDetected)  currentFault=FAULT_NO_LOAD;
}

void buzzerOn(){
  ledcWriteTone(BUZZER_PIN,BUZZER_FREQ_HZ);
  ledcWrite(BUZZER_PIN,BUZZER_DUTY_ON);
}
void buzzerOff(){
  ledcWrite(BUZZER_PIN,0);
  ledcWriteTone(BUZZER_PIN,0);
}
void beepB(int hz, unsigned long ms){
  ledcWriteTone(BUZZER_PIN,hz);
  ledcWrite(BUZZER_PIN,BUZZER_DUTY_ON);
  delay(ms);
  ledcWrite(BUZZER_PIN,0);
  ledcWriteTone(BUZZER_PIN,0);
  delay(20);
}
void startupBeep(){ beepB(2000,200); delay(100); beepB(2500,200); }
void successBeep(){ beepB(2000,100); delay(80);  beepB(2500,300); }
void alertBeep()  { beepB(1000,100); delay(50);  beepB(1000,100); }
void partialBeep(){ beepB(2000,100); delay(60);  beepB(2200,100); }

void buzzerTick(){
  bool pfGood=(PF_scaled>=PF_GOOD_MIN && PF_scaled!=0 && !noLoadDetected);
  unsigned long now=millis();
  if(pfGood!=lastPfGood && !noLoadDetected && !inPostRelay){
    lastPfGood=pfGood; buzzerOff();
    if(pfGood){bzState=BZ_GOOD;bzTimer=now;buzzerOn();}
    else      {bzState=BZ_C1;  bzTimer=now;buzzerOn();}
  }
  switch(bzState){
    case BZ_GOOD: if(now-bzTimer>=400){buzzerOff();bzState=BZ_IDLE;} break;
    case BZ_C1:   if(now-bzTimer>=100){buzzerOff();bzState=BZ_GAP;bzTimer=now;} break;
    case BZ_GAP:  if(now-bzTimer>=150){buzzerOn(); bzState=BZ_C2; bzTimer=now;} break;
    case BZ_C2:   if(now-bzTimer>=100){buzzerOff();bzState=BZ_IDLE;} break;
    default: break;
  }
}

void setRGB(bool r,bool g,bool b){
  if(!rgbEnabled){
    digitalWrite(RED_PIN,LOW);
    digitalWrite(GREEN_PIN,LOW);
    digitalWrite(BLUE_PIN,LOW);
    return;
  }
  digitalWrite(RED_PIN,  r?HIGH:LOW);
  digitalWrite(GREEN_PIN,g?HIGH:LOW);
  digitalWrite(BLUE_PIN, b?HIGH:LOW);
}
void updateRGB(){
  if(!rgbEnabled){setRGB(0,0,0);return;}
  if(noLoadDetected){setRGB(0,0,0);return;}
  if(currentFault==FAULT_OVERCURRENT||currentFault==FAULT_XOR_ABSENT){
    bool bl=(millis()/600)%2; setRGB(bl,0,0); return;
  }
  if(bestBankLocked){setRGB(0,0,1);return;}
  if(PF_scaled>=PF_GREEN_MIN) {setRGB(0,1,0);return;}
  if(PF_scaled>=PF_YELLOW_MIN){setRGB(1,1,0);return;}
  setRGB(1,0,0);
}

void applyBank(uint8_t x){
  x &= 0x07;
  digitalWrite(RLY1,(x&1)?HIGH:LOW);
  digitalWrite(RLY2,(x&2)?HIGH:LOW);
  digitalWrite(RLY3,(x&4)?HIGH:LOW);
  bank=x; lastBankChangeMs=millis();
  demo_angle_smooth=0.0f; real_angle_filt=0.0f;
  overcorrectCount=0; resistiveCount=0; leadCount=0;
  postRelayEnd=millis()+POST_RELAY_IGNORE_MS;
  inPostRelay=true;
}

void measureCurrent(){
  const int N=500;
  int64_t sumSq=0; int32_t peak=0;
  for(int i=0;i<N;i++){
    int32_t raw=(int32_t)analogRead(ADC_PIN);
    int32_t s=raw-(int32_t)CAL_ACS_OFFSET;
    sumSq+=(int64_t)s*s;
    int32_t a=abs(s); if(a>peak) peak=a;
    delayMicroseconds(100);
  }
  currentAmps   =sqrtf((float)sumSq/(float)N)*AMPS_PER_COUNT;
  currentRawPeak=(uint16_t)peak;
}

bool checkXORHealthRaw(){
  int tr=0; int prev=digitalRead(XOR_PIN);
  unsigned long s=millis();
  while(millis()-s<100){
    int c=digitalRead(XOR_PIN);
    if(c!=prev){tr++;prev=c;}
    delayMicroseconds(10);
  }
  return(tr>=6);
}
bool checkXORHealth(){
  if(inPostRelay) return xorHealthy;
  bool raw=checkXORHealthRaw();
  if(raw){xorWasUnhealthy=false;xorUnhealthySince=0;return true;}
  if(!xorWasUnhealthy){xorWasUnhealthy=true;xorUnhealthySince=millis();}
  if(millis()-xorUnhealthySince>=ZCD_UNHEALTHY_HOLDOFF_MS) return false;
  return xorHealthy;
}

bool measureXORPulses(uint32_t &pwOut, uint32_t &hpOut,
                      float &freqOut, int &nOut){
  pwOut=0; hpOut=0; freqOut=0; nOut=0;
  const int MAX_P=24;
  uint32_t pws[MAX_P],hps[MAX_P];
  int pc=0,hc=0;
  uint32_t su=micros(),lr=0;
  bool inp=false; uint32_t ps=0;
  int prev=digitalRead(XOR_PIN);
  while((micros()-su)<XOR_SAMPLE_WINDOW_US && pc<MAX_P){
    int cur=digitalRead(XOR_PIN); uint32_t now=micros();
    if(cur!=prev){
      if(cur==HIGH){
        if(lr>0&&hc<MAX_P){
          uint32_t hp=now-lr;
          if(hp>=XOR_HP_MIN_US&&hp<=XOR_HP_MAX_US) hps[hc++]=hp;
        }
        lr=now; ps=now; inp=true;
      } else {
        if(inp){
          uint32_t pw=now-ps;
          if(pw>=XOR_MIN_PULSE_US&&pw<=XOR_MAX_PULSE_US) pws[pc++]=pw;
          inp=false;
        }
      }
      prev=cur;
    }
  }
  nOut=pc;
  if(pc<XOR_MIN_SAMPLES) return false;
  std::sort(pws,pws+pc); pwOut=pws[pc/2];
  if(hc>=2){
    std::sort(hps,hps+hc);
    hpOut=hps[hc/2];
    freqOut=1000000.0f/(2.0f*(float)hpOut);
  } else {
    hpOut=(uint32_t)HALF_PERIOD_US_NOM;
    freqOut=MAINS_FREQ_HZ;
  }
  return true;
}

PhaseResult measurePhase(){
  PhaseResult r={}; r.valid=false;
  uint32_t pws[XOR_MEAS_REPEATS],hps[XOR_MEAS_REPEATS];
  int vr=0;
  for(int i=0;i<XOR_MEAS_REPEATS;i++){
    uint32_t pw=0,hp=0; float f=0; int n=0;
    if(measureXORPulses(pw,hp,f,n)){pws[vr]=pw;hps[vr]=hp;vr++;}
    yield(); delay(2);
  }
  if(vr==0) return r;

  std::sort(pws,pws+vr);
  std::sort(hps,hps+vr);
  uint32_t pwMed =pws[vr/2];
  uint32_t hpMeas=hps[vr/2];
  float freqDisp =1000000.0f/(2.0f*(float)hpMeas);

  float hp=HALF_PERIOD_US_NOM;
  float angleRaw=((float)pwMed/hp)*180.0f;
  angleRaw=constrain(angleRaw,0.0f,180.0f);

  float fullOffsetDeg=((float)CAL_PULSE_OFFSET_US/hp)*180.0f;
  float effOffsetDeg =fullOffsetDeg * DEMO_OFFSET_SCALE;
  float angleCorrSigned = angleRaw - effOffsetDeg;

  bool isLead=(angleCorrSigned < -LAG_ANGLE_DEADBAND_DEG) && (bank>0);
  bool isLag =(angleCorrSigned >  LAG_ANGLE_DEADBAND_DEG);

  float angleCorrAbs=fabsf(angleCorrSigned);
  angleCorrAbs=constrain(angleCorrAbs,0.0f,90.0f);

  float pfReal=cosf(angleCorrAbs*(float)M_PI/180.0f);

  float demoAngle=0.0f;
  if(isLag){
    float angleForDemo=angleCorrAbs + DEMO_BIAS_DEG;
    float rawDemo=angleForDemo * DEMO_PHASE_GAIN;
    if(rawDemo>0.0f && rawDemo<DEMO_ANGLE_FLOOR_DEG)
      rawDemo=DEMO_ANGLE_FLOOR_DEG;
    rawDemo=constrain(rawDemo,0.0f,DEMO_ANGLE_MAX_DEG);
    if(demo_angle_smooth < rawDemo*0.5f)
      demo_angle_smooth=rawDemo*0.75f;
    demo_angle_smooth=(DEMO_ANGLE_ALPHA*rawDemo)
                     +((1.0f-DEMO_ANGLE_ALPHA)*demo_angle_smooth);
    demoAngle=demo_angle_smooth;
  } else {
    demo_angle_smooth*=0.25f;
    if(demo_angle_smooth<0.1f) demo_angle_smooth=0.0f;
    demoAngle=0.0f;
  }

  float pfDemo=cosf(demoAngle*(float)M_PI/180.0f);

  r.angle_raw_deg      = angleRaw;
  r.angle_corr_signed  = angleCorrSigned;
  r.angle_corr_abs     = angleCorrAbs;
  r.angle_demo_deg     = demoAngle;
  r.pf_real            = pfReal;
  r.pf_demo            = pfDemo;
  r.is_lagging         = isLag;
  r.is_leading         = isLead;
  r.valid              = true;
  r.pulse_width_us     = pwMed;
  r.freq_hz_meas       = freqDisp;
  r.samples_used       = vr;
  return r;
}

void updatePowerMetrics(){
  if(currentAmps<0.005f||noLoadDetected){
    Q_var=0; P_watt=0; S_va=0; return;
  }
  S_va  =MAINS_VOLTAGE*currentAmps;
  P_watt=S_va*pf_real_filtered;
  float phi=fabsf(real_angle_filt)*(float)M_PI/180.0f;
  Q_var =S_va*sinf(phi);
}

void measureSystem(){
  measureCurrent();

  if(inPostRelay && millis()>=postRelayEnd) inPostRelay=false;

  if(currentRawPeak>MAX_CURRENT_PEAK){
    applyBank(0); PF_scaled=0; noLoadDetected=false;
    evaluateFaults(); alertBeep(); delay(2000); return;
  }

  bool acsWorking=(CAL_ACS_OFFSET>=ACS_MIN_RAW);

  if(acsWorking && currentRawPeak<MIN_CURRENT_PEAK){
    if(!noLoadDetected){
      noLoadDetected=true;
      if(bank!=0) applyBank(0);
    }
    PF_scaled=0; pf_demo_filtered=1.0f; pf_real_filtered=1.0f;
    real_angle_filt=0.0f; demo_angle_smooth=0.0f;
    Q_var=0; P_watt=0; S_va=0;
    for(int i=0;i<LAG_WINDOW;i++) lagWindow[i]=false;
    lagScore=0; isLagging=false; isLeading=false;
    ctrlState=CS_IDLE; correcting=false;
    targetReached=false; bestBankLocked=false;
    evaluateFaults();
    return;
  }

  if(noLoadDetected) noLoadDetected=false;
  if(inPostRelay){evaluateFaults();return;}

  bool newH=checkXORHealth();
  if(newH!=xorHealthy) xorHealthy=newH;
  if(!xorHealthy||!baselineValid){evaluateFaults();return;}

  PhaseResult pr=measurePhase();
  if(!pr.valid){evaluateFaults();return;}

  bool curSuff=(currentAmps>=MIN_LAG_CURRENT_A);
  bool lagVote =pr.is_lagging && curSuff;
  bool leadVote=pr.is_leading && curSuff && bank>0;

  lagWindow[lagWindowIdx]=lagVote;
  lagWindowIdx=(lagWindowIdx+1)%LAG_WINDOW;
  lagScore=0;
  for(int i=0;i<LAG_WINDOW;i++) if(lagWindow[i]) lagScore++;

  bool confirmedLag =(lagScore>=LAG_SCORE_CONFIRM);
  bool confirmedRes =(lagScore<=LAG_SCORE_CLEAR);
  if(leadVote) leadCount++; else leadCount=0;
  bool confirmedLead=(leadCount>=LEAD_SCORE_CONFIRM);

  if(confirmedLag&&!isLagging){
    isLagging=true; isLeading=false; leadCount=0;
    demo_angle_smooth=0.0f;
  } else if(confirmedRes&&isLagging&&!confirmedLead){
    isLagging=false;
  }
  if(confirmedLead&&!isLeading){
    isLeading=true; isLagging=false;
  } else if(!confirmedLead&&isLeading&&confirmedRes){
    isLeading=false;
  }

  float effReal=(isLagging||isLeading)?pr.angle_corr_signed:0.0f;
  real_angle_filt=(0.65f*effReal)+(0.35f*real_angle_filt);

  float effPFd=isLagging?pr.pf_demo:1.0f;
  if(!isLagging)
    pf_demo_filtered=min(pf_demo_filtered+0.012f,1.0f);
  else
    pf_demo_filtered=(PF_ALPHA*effPFd)+((1.0f-PF_ALPHA)*pf_demo_filtered);
  pf_demo_filtered=constrain(pf_demo_filtered,0.01f,1.0f);

  pf_real_filtered=(0.75f*pr.pf_real)+(0.25f*pf_real_filtered);

  PF_scaled=constrain((int16_t)(pf_demo_filtered*1000.0f),1,999);
  qsave=(PF_scaled>900)?(uint8_t)min((int)(PF_scaled-900),99):0;
  phaseAngleDeg=isLagging?pr.angle_demo_deg:0.0f;

  updatePowerMetrics();
  evaluateFaults();
}

void controlSystem(){
  static unsigned long runStart=millis();
  if(millis()-runStart>MAX_RUNTIME_MS){
    applyBank(0); ctrlState=CS_IDLE; correcting=false;
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0,20); display.println("COOLDOWN MODE");
    display.setCursor(0,35); display.println("Relays OFF");
    display.display();
    for(int i=COOLDOWN_MS/1000;i>0;i--) yieldDelay(1000);
    runStart=millis(); startupBeep(); return;
  }

  if(!baselineValid||noLoadDetected||inPostRelay) return;

  bool holdOk  =(millis()-lastBankChangeMs>=MIN_BANK_HOLD_MS);
  bool settleOk=(millis()-settleStart>=SETTLE_TIME_MS);

  switch(ctrlState){

    case CS_IDLE:
      correcting=false; targetFailed=false; bestBankLocked=false;
      overcorrectCount=0; resistiveCount=0;
      if(real_angle_filt>=-REAL_LEAD_DEG&&real_angle_filt<=REAL_CORRECTED_DEG){
        targetReached=true; return;
      }
      targetReached=false;
      if(lagScore>=LAG_SCORE_CONFIRM&&isLagging&&real_angle_filt>REAL_LAG_START_DEG){
        correcting=true;
        correctionStart=millis(); correctionSteps=0;
        realAngleAtStep=real_angle_filt;
        if(selectedMode==MODE_AI){
          float Qn=Q_var;
          if(Qn>4.0f)bank=7;else if(Qn>3.0f)bank=6;
          else if(Qn>2.5f)bank=5;else if(Qn>2.0f)bank=4;
          else if(Qn>1.5f)bank=3;else if(Qn>0.8f)bank=2;
          else bank=1;
        } else {
          bank=(bank==0)?1:bank;
        }
        applyBank(bank); correctionSteps=1;
        settleStart=millis(); ctrlState=CS_STEPPING;
      }
      break;

    case CS_STEPPING:
      correcting=true;
      correctionTime=(millis()-correctionStart)/1000.0f;
      if(!settleOk||inPostRelay) return;
      ctrlState=CS_HOLDING;
      break;

    case CS_HOLDING:
      correctionTime=(millis()-correctionStart)/1000.0f;
      if(!holdOk||inPostRelay) return;

      if(isLeading||(bank>0&&real_angle_filt<-REAL_LEAD_DEG)){
        overcorrectCount++;
        if(overcorrectCount>=OVERCORRECT_CONFIRM_COUNT){
          if(bank>0){
            bank--;
            applyBank(bank);
            realAngleAtStep=real_angle_filt;
            settleStart=millis(); correctionSteps++;
            ctrlState=CS_STEPPING;
          }
        }
        return;
      } else {
        if(overcorrectCount>0) overcorrectCount=0;
      }

      if(real_angle_filt<=REAL_CORRECTED_DEG&&real_angle_filt>=-REAL_LEAD_DEG){
        correcting=false; targetReached=true; targetFailed=false;
        overcorrectCount=0;
        ctrlState=CS_DONE; successBeep(); return;
      }

      if(isLagging&&lagScore>=LAG_SCORE_CONFIRM&&real_angle_filt>REAL_LAG_START_DEG){
        float improvement=realAngleAtStep-real_angle_filt;
        if(correctionSteps>1&&improvement<0.5f){
          if(bank<7){
            bank++;
            applyBank(bank); realAngleAtStep=real_angle_filt;
            settleStart=millis(); correctionSteps++;
            ctrlState=CS_STEPPING;
          } else {
            bestBankLocked=true; bestBank=bank;
            correcting=false; ctrlState=CS_BEST_BANK;
            partialBeep();
          }
          return;
        }
        realAngleAtStep=real_angle_filt;
        settleStart=millis(); ctrlState=CS_STEPPING;
      }
      break;

    case CS_BEST_BANK:
      correcting=false; bestBankLocked=true;
      if(noLoadDetected||PF_scaled==0){
        ctrlState=CS_IDLE; bestBankLocked=false;
        applyBank(0); return;
      }
      if(real_angle_filt>REAL_LAG_START_DEG*BEST_BANK_REENTRY_MULT
         &&lagScore>=LAG_SCORE_CONFIRM){
        bestBankLocked=false; applyBank(0);
        ctrlState=CS_IDLE; pf_demo_filtered=0.5f;
      }
      break;

    case CS_DONE:
      correcting=false;
      if(inPostRelay) return;
      if(real_angle_filt>REAL_LAG_START_DEG&&isLagging){
        resistiveCount++;
        if(resistiveCount>=RESISTIVE_HOLD_COUNT){
          ctrlState=CS_IDLE; targetReached=false;
          pf_demo_filtered=(float)PF_scaled/1000.0f;
          resistiveCount=0;
        }
      } else {
        if(resistiveCount>0) resistiveCount=0;
      }
      break;

    case CS_FAILED:{
      static int16_t lpf=0;
      if(lpf==0) lpf=PF_scaled;
      if(abs(PF_scaled-lpf)>100){
        applyBank(0); ctrlState=CS_IDLE;
        targetFailed=false; lpf=0; pf_demo_filtered=0.5f;
      }
      break;
    }
  }
}

void drawWiFiIcon(int x,int y){
  if(WiFi.status()==WL_CONNECTED){
    display.fillCircle(x+4,y+4,1,WHITE);
    display.drawCircle(x+4,y+4,3,WHITE);
    display.drawCircle(x+4,y+4,5,WHITE);
  } else {
    display.setTextSize(1); display.setCursor(x,y);
    display.print("NC");
  }
}
void drawBankDots(int x,int y){
  for(int i=0;i<3;i++){
    if((bank>>i)&1) display.fillRect(x+(i*10),y,7,7,WHITE);
    else            display.drawRect(x+(i*10),y,7,7,WHITE);
  }
}

void drawOLED(){
  display.clearDisplay(); display.setTextColor(WHITE);
  display.setTextSize(1);

  display.setCursor(0,0); display.print("PFC V25.18");
  display.setCursor(60,0);
  if(inPostRelay)       display.print("ADJ    ");
  else if(xorHealthy)   display.print("XOR:OK ");
  else                  display.print("XOR:ERR");
  drawWiFiIcon(112,0);
  display.drawFastHLine(0,10,128,WHITE);

  display.setCursor(0,12);
  display.print(selectedMode==MODE_DYNAMIC?"DYN":"PRD");
  display.print("  ");
  const char* st;
  if     (noLoadDetected)                      st="NO LOAD";
  else if(inPostRelay)                         st="ADJUSTNG";
  else if(!xorHealthy)                         st="XOR ERR";
  else if(!baselineValid)                      st="NO CAL";
  else if(currentFault==FAULT_OVERCURRENT)     st="OVERCUR";
  else if(currentFault==FAULT_CTRL_FAILED)     st="FAILED";
  else if(bestBankLocked)                      st="BEST PF";
  else if(isLeading)                           st="LEADING";
  else if(PF_scaled>=PF_GOOD_MIN&&!correcting) st="PF GOOD";
  else if(correcting)                          st="CORRECT";
  else if(isLagging)                           st="LAGGING";
  else                                         st="STANDBY";
  display.print(st);
  display.setCursor(96,12);
  char scb[8]; snprintf(scb,sizeof(scb),"s%d/%d",lagScore,LAG_WINDOW);
  display.print(scb);

  display.setTextSize(2); display.setCursor(0,21);
  if(noLoadDetected){
    display.print("NO LOAD");
  } else if(!xorHealthy&&!inPostRelay){
    display.print("PF:----");
  } else if(!baselineValid){
    display.print("NO CAL ");
  } else {
    char buf[10];
    snprintf(buf,sizeof(buf),"PF:%4.2f",(float)PF_scaled/1000.0f);
    display.print(buf);
  }

  display.setTextSize(1);
  if(!noLoadDetected && PF_scaled!=0){
    display.setCursor(102,28);
    if(bestBankLocked)              display.print("BEST");
    else if(isLeading)              display.print("LEAD");
    else if(PF_scaled>=PF_GOOD_MIN) display.print("GOOD");
    else if(isLagging)              display.print("LAG ");
    else                            display.print("RES ");
    display.setCursor(102,37);
    char bkb[6]; snprintf(bkb,sizeof(bkb),"B=%u",bank);
    display.print(bkb);
  }

  display.setCursor(0,39);
  if(noLoadDetected){
    display.print("Connect load to start");
  } else if(PF_scaled!=0){
    if(isLagging){
      char ql[22];
      snprintf(ql,sizeof(ql),"Q=%5.2fVAR I=%5.3fA",Q_var,currentAmps);
      display.print(ql);
    } else {
      char pl[22];
      snprintf(pl,sizeof(pl),"P=%5.2fW   I=%5.3fA",P_watt,currentAmps);
      display.print(pl);
    }
  }

  display.drawFastHLine(0,49,128,WHITE);
  display.setCursor(0,51);
  if(footerPage==0){
    if(!noLoadDetected) drawBankDots(0,51);
    display.setCursor(34,51);
    if(noLoadDetected)
      display.print("No load detected");
    else{
      char ph[22];
      snprintf(ph,sizeof(ph),"R=%+4.1f D=%4.0f",real_angle_filt,phaseAngleDeg);
      display.print(ph);
    }
  } else if(footerPage==1){
    char tl[24];
    snprintf(tl,sizeof(tl),"T=%.1fs steps=%u",correctionTime,correctionSteps);
    display.print(tl);
  } else {
    char ol[26];
    snprintf(ol,sizeof(ol),"oc=%d rc=%d lc=%d %s",
      overcorrectCount,resistiveCount,leadCount,ctrlStateName(ctrlState));
    display.print(ol);
  }
  display.display();
}

bool measureBaselinePulses(int nSamples, uint32_t &offsetOut, float &noiseOut){
  uint64_t sum=0; int valid=0;
  uint32_t allPW[20]; int pc=0;

  for(int s=0;s<nSamples;s++){
    display.clearDisplay();
    display.setTextSize(1); display.setTextColor(WHITE);
    display.setCursor(0,0);  display.println("BASELINE CAL");
    display.drawFastHLine(0,10,128,WHITE);
    display.setCursor(0,14); display.println("Resistive load only!");
    char buf[32];
    snprintf(buf,sizeof(buf),"Sample %d/%d",s+1,nSamples);
    display.setCursor(0,26); display.println(buf);
    display.setCursor(0,38); display.println("Measuring offset...");
    display.display(); yield();

    uint32_t pw=0,hp=0; float freq=0; int n=0;
    bool ok=measureXORPulses(pw,hp,freq,n);
    if(ok){ sum+=pw; valid++; if(pc<20) allPW[pc++]=pw; }
    yieldDelay(500);
  }
  if(valid<3) return false;

  offsetOut=(uint32_t)(sum/valid);
  float maxDev=0;
  for(int i=0;i<pc;i++){
    float d=fabsf((float)allPW[i]-(float)offsetOut);
    if(d>maxDev) maxDev=d;
  }
  noiseOut=maxDev;
  if(noiseOut>CAL_NOISE_REJECT_US) return false;
  return true;
}

void inductiveLoadTest(){
  unsigned long ts=millis(), lp=0;
  while(true){
    yield();
    if(digitalRead(BUTTON_PIN)==LOW){
      while(digitalRead(BUTTON_PIN)==LOW){delay(10);yield();}
      beepB(2500,80); break;
    }
    if(millis()-ts>=60000UL) break;
    int cd=60-(int)((millis()-ts)/1000);

    if(millis()-lp>=3000UL){
      lp=millis(); yield();
      const int N=500; int64_t sq=0; int32_t pk=0;
      for(int i=0;i<N;i++){
        int32_t s=(int32_t)analogRead(ADC_PIN)-(int32_t)CAL_ACS_OFFSET;
        sq+=(int64_t)s*s;
        int32_t a=abs(s); if(a>pk) pk=a;
        delayMicroseconds(100);
      }
      float I=sqrtf((float)sq/N)*AMPS_PER_COUNT;
      PhaseResult pr=measurePhase();
      bool lagV=(I>=MIN_LAG_CURRENT_A);
      float dPF=pr.valid?(lagV?pr.pf_demo:1.0f):0.0f;
      bool  dLag=pr.valid&&lagV&&pr.is_lagging;
      float phiR=pr.valid?pr.angle_corr_signed:0.0f;

      display.clearDisplay();
      display.setTextSize(1); display.setTextColor(WHITE);
      display.setCursor(0,0); display.print("INDUCTIVE TEST");
      display.drawFastHLine(0,10,128,WHITE);
      char lb[28];
      snprintf(lb,sizeof(lb),"I=%7.4fA  G=%.0fx",I,DEMO_PHASE_GAIN);
      display.setCursor(0,12); display.print(lb);
      char rb[28];
      snprintf(rb,sizeof(rb),"phiR=%+6.2fdeg",phiR);
      display.setCursor(0,21); display.print(rb);
      display.drawFastHLine(0,31,128,WHITE);
      display.setTextSize(2); display.setCursor(0,33);
      char bf[10];
      snprintf(bf,sizeof(bf),"PF:%4.2f",dPF);
      display.print(bf);
      display.setTextSize(1);
      display.setCursor(102,33); display.print(dLag?"LAG":"RES");
      display.setCursor(0,52);
      if(I<MIN_LAG_CURRENT_A)      display.print("Low current-add load");
      else if(dLag&&dPF>0.90f)     display.print("Marginal-add more L");
      else if(dLag&&dPF<0.25f)     display.print("Strong-remove some L");
      else if(dLag)                display.print("PF good for demo!");
      else if(fabsf(phiR)<1.0f)    display.print("Resistive OK");
      else                         display.print("No lag-add inductor");
      display.setCursor(102,52);
      char cb[5]; snprintf(cb,sizeof(cb),"%2ds",cd);
      display.print(cb);
      display.display();

      if(dLag)         setRGB(1,0,0);
      else if(I>0.05f) setRGB(0,1,0);
      else             setRGB(0,0,0);
    }
    delay(20); yield();
  }
  setRGB(0,0,0);
}

void calibrationV25(){
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0,0); display.println("PFC V25.18 STARTUP");
  display.drawFastHLine(0,10,128,WHITE);
  display.setCursor(0,14); display.println("Short tap=Recalibrate");
  display.setCursor(0,24); display.println("Hold 2s  =Use saved");
  display.setCursor(0,34); display.println("Timeout  =Use saved");
  display.setCursor(0,48); display.println("Waiting 5s...");
  display.display();

  uint16_t savedACS=0; uint32_t savedOff=0; float savedNoise=50.0f;
  bool hasSaved=loadCalibration(savedACS,savedOff,savedNoise);
  display.setCursor(0,56);
  display.println(hasSaved?"Saved cal found!":"No cal-run full");
  display.display();

  unsigned long ws=millis(); bool decided=false,doFull=false;
  while(!decided&&millis()-ws<5000UL){
    if(digitalRead(BUTTON_PIN)==LOW){
      unsigned long ps=millis();
      while(digitalRead(BUTTON_PIN)==LOW){delay(10);yield();}
      decided=true; doFull=(millis()-ps<=1500);
    }
    yield();
  }
  if(!decided) doFull=false;
  if(!hasSaved) doFull=true;

  if(!doFull){
    if(hasSaved){
      CAL_ACS_OFFSET=savedACS;
      CAL_PULSE_OFFSET_US=savedOff;
      CAL_PULSE_NOISE_US=savedNoise;
      baselineValid=true;
    } else {
      baselineValid=false;
    }
    beepB(2500,50); calibrationDone=true;
    inductiveLoadTest(); return;
  }

  display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0,0); display.println("STEP 1: ACS ZERO");
  display.drawFastHLine(0,10,128,WHITE);
  display.setCursor(0,14); display.println("Disconnect ALL loads");
  display.setCursor(0,24); display.println("ACS VCC must be 5V");
  display.setCursor(0,34); display.println("Expect ~2.50V output");
  display.setCursor(0,56); display.println("Press BTN when ready");
  display.display();

  unsigned long liveStart=millis();
  while(digitalRead(BUTTON_PIN)==HIGH){
    if(millis()-liveStart>=500){
      liveStart=millis();
      uint32_t lr=analogRead(ADC_PIN);
      float lv=(lr/ADC_RESOLUTION)*ADC_VREF;
      display.fillRect(0,54,128,10,BLACK);
      display.setCursor(0,54);
      char db[28];
      snprintf(db,sizeof(db),"ADC=%4lu V=%.3fV",(unsigned long)lr,lv);
      display.println(db); display.display();
    }
    delay(50); yield();
  }
  delay(400);

  uint32_t asum=0;
  for(int i=0;i<2000;i++){asum+=analogRead(ADC_PIN);delayMicroseconds(200);}
  CAL_ACS_OFFSET=(uint16_t)(asum/2000);
  float offV=(CAL_ACS_OFFSET/ADC_RESOLUTION)*ADC_VREF;
  bool acsOK=(CAL_ACS_OFFSET>=ACS_MIN_RAW&&offV>=ACS_MIN_VOLTS&&offV<=ACS_MAX_VOLTS);

  display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0,0); display.println("STEP 1: RESULT");
  display.drawFastHLine(0,10,128,WHITE);
  char b0[32];
  snprintf(b0,sizeof(b0),"Raw=%u V=%.3fV",CAL_ACS_OFFSET,offV);
  display.setCursor(0,16); display.println(b0);
  display.setCursor(0,40);
  display.println(acsOK?"VALID":"FAULT-check VCC=5V");
  display.display();
  if(!acsOK){alertBeep();alertBeep();}else successBeep();
  yieldDelay(2000);

  display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(0,0); display.println("STEP 2: XOR BASELINE");
  display.drawFastHLine(0,10,128,WHITE);
  display.setCursor(0,14); display.println("Resistive load ONLY");
  display.setCursor(0,24); display.println("NO inductors!");
  display.setCursor(0,44); display.println("phiR->0 after this");
  display.setCursor(0,56); display.println("Press BTN when ready");
  display.display();
  delay(3000);
  while(digitalRead(BUTTON_PIN)==HIGH){delay(50);yield();}
  delay(400);

  uint32_t offMeas=0; float nMeas=50.0f;
  bool bOK=measureBaselinePulses(8,offMeas,nMeas);
  if(bOK){
    CAL_PULSE_OFFSET_US=offMeas;
    CAL_PULSE_NOISE_US=nMeas;
    baselineValid=true;
    saveCalibration(acsOK?CAL_ACS_OFFSET:0,CAL_PULSE_OFFSET_US,CAL_PULSE_NOISE_US);
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
    display.setCursor(0,0); display.println("STEP 2: DONE");
    display.drawFastHLine(0,10,128,WHITE);
    char b1[32],b2[32];
    snprintf(b1,sizeof(b1),"Raw=%luus",(unsigned long)CAL_PULSE_OFFSET_US);
    snprintf(b2,sizeof(b2),"Eff=%.0fus (x%.2f)",
      (float)CAL_PULSE_OFFSET_US*DEMO_OFFSET_SCALE,DEMO_OFFSET_SCALE);
    display.setCursor(0,16); display.println(b1);
    display.setCursor(0,26); display.println(b2);
    display.setCursor(0,56); display.println("Saved OK");
    display.display(); successBeep(); yieldDelay(2000);
  } else {
    baselineValid=false;
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
    display.setCursor(0,0); display.println("STEP 2: FAILED");
    display.setCursor(0,14); display.println("No XOR or noise high");
    display.setCursor(0,24); display.println("Check XOR circuit");
    display.display(); alertBeep(); yieldDelay(5000);
  }
  calibrationDone=true;
  inductiveLoadTest();
}

void showMenu(int option){
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
  display.setCursor(10,1); display.print("IPFC SYSTEM V25.18");
  display.drawFastHLine(0,11,128,WHITE);
  if(option==0){display.fillRect(0,14,128,12,WHITE);display.setTextColor(BLACK);}
  else display.setTextColor(WHITE);
  display.setCursor(20,17); display.print(">> DYNAMIC MODE");
  if(option==1){display.fillRect(0,28,128,12,WHITE);display.setTextColor(BLACK);}
  else display.setTextColor(WHITE);
  display.setCursor(14,31); display.print(">> PREDICTIVE MODE");
  display.setTextColor(WHITE);
  display.setCursor(tickerX,MENU_TICKER_ROW);
  display.print(MENU_TICKER);
  display.display();
}

void chooseMode(){
  int option=0; initMenuTicker(); showMenu(option);
  while(!modeChosen){
    stepMenuTicker(); showMenu(option);
    if(digitalRead(BUTTON_PIN)==LOW){
      unsigned long ps=millis(); bool lp=false;
      while(digitalRead(BUTTON_PIN)==LOW){
        if(millis()-ps>1500){lp=true;break;}
        stepMenuTicker(); showMenu(option); delay(5);
      }
      if(lp){
        selectedMode=(option==0)?MODE_DYNAMIC:MODE_AI;
        modeChosen=true;
        if(selectedMode==MODE_DYNAMIC) beepB(2000,150);
        else{beepB(2000,80);delay(80);beepB(2000,80);}
        display.clearDisplay(); display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(14,18); display.print("SYSTEM BOOTING...");
        display.drawRect(10,34,108,10,WHITE); display.display();
        for(int i=0;i<=96;i+=4){
          display.fillRect(12,36,i,6,WHITE); display.display(); delay(25);
        }
        return;
      } else {
        option=!option; initMenuTicker(); showMenu(option); delay(200);
      }
    }
    delay(5);
  }
}

void runtimeButton(){
  static bool ls=HIGH;
  bool s=digitalRead(BUTTON_PIN);
  if(ls==HIGH&&s==LOW){
    footerPage++; if(footerPage>2) footerPage=0;
    beepB(2500,20);
  }
  ls=s;
}

void checkDiagButton(){
  static unsigned long ps=0; static bool pressed=false;
  if(digitalRead(BUTTON_PIN)==LOW){
    if(!pressed){pressed=true;ps=millis();}
    if(millis()-ps>3000&&pressed){ pressed=false; beepB(2000,300); }
  } else { pressed=false; }
}

BLYNK_WRITE(V0){int v=param.asInt();if(v>=0&&v<=7)applyBank(v);}
BLYNK_WRITE(V1){selectedMode=(param.asInt()==0)?MODE_DYNAMIC:MODE_AI;}
BLYNK_CONNECTED(){Blynk.syncAll();}

void updateBlynk(){
  static unsigned long lu=0;
  if(millis()-lu>1000UL){
    lu=millis();
    Blynk.virtualWrite(V2,  noLoadDetected?0.0f:pf_demo_filtered);
    Blynk.virtualWrite(V3,  currentAmps);
    Blynk.virtualWrite(V4,  bank);
    Blynk.virtualWrite(V5,  qsave);
    Blynk.virtualWrite(V6,  correcting?1:0);
    Blynk.virtualWrite(V7,  xorHealthy?1:0);
    Blynk.virtualWrite(V8,  phaseAngleDeg);
    Blynk.virtualWrite(V9,  Q_var);
    Blynk.virtualWrite(V10, (int)currentFault);
    Blynk.virtualWrite(V11, real_angle_filt);
  }
}

void setup(){
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(XOR_PIN,   INPUT_PULLUP);
  pinMode(ADC_PIN,   INPUT);
  pinMode(RLY1,OUTPUT); digitalWrite(RLY1,LOW);
  pinMode(RLY2,OUTPUT); digitalWrite(RLY2,LOW);
  pinMode(RLY3,OUTPUT); digitalWrite(RLY3,LOW);
  pinMode(RED_PIN,  OUTPUT); digitalWrite(RED_PIN,  LOW);
  pinMode(GREEN_PIN,OUTPUT); digitalWrite(GREEN_PIN,LOW);
  pinMode(BLUE_PIN, OUTPUT); digitalWrite(BLUE_PIN, LOW);
  pinMode(BUZZER_PIN,OUTPUT);
  ledcAttach(BUZZER_PIN,BUZZER_FREQ_HZ,BUZZER_RES_BITS);
  ledcWrite(BUZZER_PIN,0);

  Wire.begin(SDA_PIN,SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)){
    for(;;){beepB(1000,200);delay(400);}
  }
  display.clearDisplay(); display.display();

  for(int i=0;i<LAG_WINDOW;i++) lagWindow[i]=false;
  lastBankChangeMs=0;
  inPostRelay=false; postRelayEnd=0;
  overcorrectCount=0; resistiveCount=0; leadCount=0;
  real_angle_filt=0.0f; demo_angle_smooth=0.0f;
  bestBankLocked=false; bestBank=0;
  noLoadDetected=false;

  rgbEnabled=false;
  chooseMode();
  calibrationV25();

  WiFi.begin(ssid,pass);
  unsigned long ws=millis();
  while(WiFi.status()!=WL_CONNECTED&&millis()-ws<10000UL){
    delay(500); yield();
  }
  Blynk.config(auth);
  if(WiFi.status()==WL_CONNECTED) Blynk.connect(3000);

  startupBeep(); delay(300);
  rgbEnabled=true;
}

void loop(){
  yield();
  if(WiFi.status()==WL_CONNECTED){Blynk.run();updateBlynk();}
  runtimeButton();
  checkDiagButton();
  buzzerTick();
  static unsigned long lt=0;
  if(millis()-lt>=MAIN_LOOP_MS){
    lt=millis();
    measureSystem();
    controlSystem();
    drawOLED();
    updateRGB();
  }
}