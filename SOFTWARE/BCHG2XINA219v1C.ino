#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <string.h>
#include <math.h>

/* ===================== PINS ===================== */
#define I2C_SDA 21
#define I2C_SCL 22

#define TFT_CS    5
#define TFT_DC   27
#define TFT_RST  26
#define TFT_SCK  18
#define TFT_MOSI 23
#define TFT_MISO 19

#define PIN_CHG_MOSFET 14
#define PIN_STATUS_LED 2

// ✅ AO3400 (LOAD low-side switch) gate pin
#define PIN_LOAD_MOSFET 13

// ADC za VIN (napon punjenja, meren preko delitelja)
#define PIN_VCHG_ADC 34   // GPIO34 (ADC1)

/* ===================== ADC SCALE (R1/R2) ===================== */
// VIN -> 100k -> NODE -> 22k -> GND, NODE -> GPIO34
static const float ADC_R1 = 100000.0f;
static const float ADC_R2 =  22000.0f;
static const float ADC_DIV_GAIN = (ADC_R1 + ADC_R2) / ADC_R2; // 5.54545

// Kalibracija VIN po multimetru
static const float VIN_CAL = 0.975f;

/* ===================== INA219 (2x) ===================== */
static uint8_t INA1_ADDR = 0x41;   // ICHG (shunt)
static uint8_t INA2_ADDR = 0x44;   // VBAT (bus) + ILOAD (shunt)

#define INA219_REG_CONFIG      0x00
#define INA219_REG_SHUNT_V     0x01
#define INA219_REG_BUS_V       0x02

static const float BUS_LSB_V   = 0.004f;      // 4mV/bit
static const float SHUNT_LSB_V = 0.00001f;    // 10uV/bit
static const float RSHUNT      = 0.100f;      // R100 0.1Ω

static const uint16_t INA219_CFG = 0x399F;    // 32V, PGA 320mV, cont

/* ===================== RULES ===================== */
static const float VBAT_NOT_PRESENT_TH = 2.0f;

// VIN detect (ovo je mereno preko ADC)
static const float VIN_PRESENT_ON  = 12.5f;
static const float VIN_PRESENT_OFF = 11.5f;

static const float VBAT_LOW_TH   = 12.0f;
static const float VBAT_LOW_REC  = 12.1f;
static const float VBAT_CRIT_TH  = 11.7f;
static const float VBAT_CRIT_REC = 11.8f;
static const float VBAT_LVC_TH   = 11.5f;
static const float VBAT_LVC_REC  = 11.8f;

// ✅ LOAD LVC (AO3400) – odsecanje potrošača
static const float LOAD_OFF_VBAT = 12.7f;  // isključi LOAD
static const float LOAD_ON_VBAT  = 13.0f;  // uključi LOAD (histereza)

static const float I_CHG_MAX = 0.8f;
static const uint32_t AC_RETURN_DELAY_MS = 2500;

// Soft charge ON/OFF po VBAT
static const float CHG_OFF_VBAT = 14.2f;
static const float CHG_ON_VBAT  = 13.7f;

/* ===================== UI RANGES ===================== */
static const float VIN_MIN  = 0.0f;
static const float VIN_MAX  = 22.0f;

static const float VBAT_MIN = 11.0f;
static const float VBAT_MAX = 14.6f;

static const float ICHG_MIN = 0.0f;
static const float ICHG_MAX = 0.8f;

static const float ILOAD_MIN = 0.0f;
static const float ILOAD_MAX = 0.30f;

static const float NET_NEG_MIN = -0.30f;
static const float NET_POS_MAX =  0.80f;

/* ===================== TYPES ===================== */
enum BattState { BAT_NOT_PRESENT, BAT_LVC, BAT_CRITICAL, BAT_LOW, BAT_OK };
enum SysState  { SYS_IDLE, SYS_CHARGING, SYS_DISCHARGE, SYS_FAULT };

struct Measurements {
  float Vin;   // VIN preko ADC
  float Vbat;  // INA2 BUS
  float Ichg;  // INA1 SHUNT / RSHUNT
  float Iload; // INA2 SHUNT / RSHUNT
  float Inet;  // Ichg - Iload
  bool  sh1Sat;
  bool  sh2Sat;
};

/* ===================== DISPLAY ===================== */
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

/* ===================== UI LAYOUT ===================== */
static const int TEXTSIZE = 2;
static const int CHAR_W   = 6 * TEXTSIZE;

static const int X_LEFT   = 6;
static const int X_LABEL  = 6;
static const int X_NUM    = 210;

static const int X_FOOT_BATLAB = 6;
static const int X_FOOT_BATVAL = 60;

static const int Y_STATUS = 0;

static const int Y_VIN    = 28;
static const int Y_ICHG   = 64;
static const int Y_VBAT   = 100;
static const int Y_ILOAD  = 136;
static const int Y_NET    = 172;
static const int Y_FOOT   = 208;

static const int BAR_X = 92;
static const int BAR_W = 210;
static const int BAR_H = 14;
static const int BAR_Y_OFF = 18;

static const int NUM_W_CHARS = 9;
static const int NUM_W_PX    = NUM_W_CHARS * CHAR_W;

static const int YLBL_VIN   = Y_VIN   + BAR_Y_OFF;
static const int YLBL_ICHG  = Y_ICHG  + BAR_Y_OFF;
static const int YLBL_VBAT  = Y_VBAT  + BAR_Y_OFF;
static const int YLBL_ILOAD = Y_ILOAD + BAR_Y_OFF;
static const int YLBL_NET   = Y_NET   + BAR_Y_OFF;

/* ===================== COLORS ===================== */
static inline uint16_t rgb565(uint8_t r,uint8_t g,uint8_t b){
  return ((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3);
}
static const uint16_t COL_BG   = ILI9341_BLACK;
static const uint16_t COL_DIM  = rgb565(80,80,80);
static const uint16_t COL_LAB  = rgb565(200,200,200);
static const uint16_t COL_WHT  = ILI9341_WHITE;

static const uint16_t COL_OK   = rgb565(0,255,0);
static const uint16_t COL_LOW  = rgb565(255,255,0);
static const uint16_t COL_CRIT = rgb565(255,165,0);
static const uint16_t COL_LVC  = rgb565(255,0,0);

static const uint16_t COL_MODE = rgb565(0,200,255);

static const uint16_t COL_ICHG = rgb565(0,255,0);
static const uint16_t COL_ILOAD= rgb565(255,80,80);
static const uint16_t COL_NETC = rgb565(160,160,160);
static const uint16_t COL_VIN  = rgb565(180,220,255);

/* ===================== GLOBAL STATE ===================== */
BattState battState = BAT_NOT_PRESENT;
SysState  sysState  = SYS_FAULT;

bool acPresent=false;
bool chargeEnabled=false;
uint32_t acReturnMs=0;

// ✅ AO3400 LOAD enable state (GPIO13)
static bool loadEnabled = true;

bool freezeBars=false;

// ✅ TFT recover state
static bool lastAcPresent = false;
static uint32_t lastTftRecoverMs = 0;
static const uint32_t TFT_RECOVER_COOLDOWN_MS = 1500;

/* ===================== CACHES ===================== */
static char lastMode[20]="";
static char lastDiag[24]="";

static char lastVINs[16]="";
static char lastVBATs[16]="";
static char lastICHGs[16]="";
static char lastILOADs[16]="";
static char lastNETs[16]="";

static char lastAVin=' ', lastAV=' ', lastAI=' ', lastAL=' ', lastAN=' ';

static float shownVin=NAN, shownVbat=NAN, shownIchg=NAN, shownIload=NAN, shownInet=NAN;

static int w_vin=0, w_vbat=0, w_ichg=0, w_iload=0;
static int w_net_pos=0, w_net_neg=0;

/* ===================== HELPERS ===================== */
static uint16_t lerp565(uint16_t a,uint16_t b,float t){
  if(t<0)t=0;if(t>1)t=1;
  uint8_t ar=(a>>11)&31, ag=(a>>5)&63, ab=a&31;
  uint8_t br=(b>>11)&31, bg=(b>>5)&63, bb=b&31;
  return ((uint16_t)(ar+(br-ar)*t)<<11)|
         ((uint16_t)(ag+(bg-ag)*t)<<5) |
         ((uint16_t)(ab+(bb-ab)*t));
}

static uint16_t vbatGradient(float v){
  uint16_t RED=rgb565(255,0,0), YEL=rgb565(255,255,0), GRN=rgb565(0,255,0), BLU=rgb565(0,128,255);
  if(!isfinite(v)) return COL_WHT;
  if(v<=11.5f) return RED;
  if(v<12.0f)  return lerp565(RED, YEL, (v-11.5f)/0.5f);
  if(v<12.6f)  return lerp565(YEL, GRN, (v-12.0f)/0.6f);
  if(v<14.4f)  return lerp565(GRN, BLU, (v-12.6f)/1.8f);
  return BLU;
}

static uint16_t battColor(BattState s){
  switch(s){
    case BAT_OK: return COL_OK;
    case BAT_LOW: return COL_LOW;
    case BAT_CRITICAL: return COL_CRIT;
    case BAT_LVC: return COL_LVC;
    case BAT_NOT_PRESENT: return COL_LVC;
  }
  return COL_WHT;
}

static const char* battText(BattState s){
  switch(s){
    case BAT_OK: return "OK";
    case BAT_LOW: return "LOW";
    case BAT_CRITICAL: return "CRIT";
    case BAT_LVC: return "LVC";
    case BAT_NOT_PRESENT: return "MISS";
  }
  return "";
}

static void printFixed(int x,int y,const char* txt,int widthChars,uint16_t fg,uint16_t bg=COL_BG){
  tft.setCursor(x,y);
  tft.setTextColor(fg, bg);
  int n=(int)strlen(txt);
  for(int i=0;i<widthChars;i++){
    char c = (i<n) ? txt[i] : ' ';
    tft.write(c);
  }
}

static float clamp01(float x){
  if(!isfinite(x)) return 0;
  if(x<0) return 0;
  if(x>1) return 1;
  return x;
}

static void barDeltaDraw(int y, int &w_prev, int w_new, uint16_t col){
  if(w_new<0) w_new=0;
  int maxW = (BAR_W-2);
  if(w_new>maxW) w_new=maxW;
  if(w_new == w_prev) return;

  int x0 = BAR_X+1;
  int yy = y+1;
  int hh = BAR_H-2;

  if(w_new > w_prev){
    tft.fillRect(x0 + w_prev, yy, w_new - w_prev, hh, col);
  } else {
    tft.fillRect(x0 + w_new,  yy, w_prev - w_new, hh, COL_BG);
  }
  w_prev = w_new;
}

static float deadzoneHold(float prev, float now, float db){
  if(!isfinite(now)) return NAN;
  if(!isfinite(prev)) return now;
  if(fabsf(now-prev) < db) return prev;
  return prev + 0.35f*(now-prev);
}

static char trendArrow(float prevShown, float nowShown, float db){
  if(!isfinite(prevShown) || !isfinite(nowShown)) return ' ';
  float d = nowShown - prevShown;
  if(d > db)  return '^';
  if(d < -db) return 'v';
  return ' ';
}

/* ===================== INA219 low-level ===================== */
static bool inaW(uint8_t a,uint8_t r,uint16_t v){
  Wire.beginTransmission(a);
  Wire.write(r); Wire.write(v>>8); Wire.write(v & 0xFF);
  return Wire.endTransmission()==0;
}
static bool inaR(uint8_t a,uint8_t r,uint16_t &v){
  Wire.beginTransmission(a);
  Wire.write(r);
  if(Wire.endTransmission(false)!=0) return false;
  if(Wire.requestFrom(a,(uint8_t)2)!=2) return false;
  v = (Wire.read()<<8) | Wire.read();
  return true;
}
static bool inaPing(uint8_t addr){
  Wire.beginTransmission(addr);
  return (Wire.endTransmission()==0);
}
static bool ina219Init(uint8_t addr){
  if(!inaPing(addr)) return false;
  return inaW(addr, INA219_REG_CONFIG, INA219_CFG);
}
static float busV(uint8_t addr){
  uint16_t raw;
  if(!inaR(addr, INA219_REG_BUS_V, raw)) return NAN;
  uint16_t v = (raw >> 3) & 0x1FFF;
  return (float)v * BUS_LSB_V;
}
static float shuntV(uint8_t addr){
  uint16_t raw;
  if(!inaR(addr, INA219_REG_SHUNT_V, raw)) return NAN;
  int16_t s = (int16_t)raw;
  return (float)s * SHUNT_LSB_V;
}
static float currA_fromShunt(float vsh){
  if(!isfinite(vsh)) return NAN;
  return vsh / RSHUNT;
}

/* ===================== ADC (VIN) ===================== */
static float readVinAdc(){
  const int N = 8;
  int sum = 0;
  for(int i=0;i<N;i++){
    sum += analogReadMilliVolts(PIN_VCHG_ADC);
    delayMicroseconds(80);
  }
  int mv = sum / N;
  if(mv <= 0) return NAN;

  float v_adc = (float)mv / 1000.0f;

  // ✅ Floating zaštita
  if(v_adc < 0.05f) return NAN;

  float vin = v_adc * ADC_DIV_GAIN;
  vin *= VIN_CAL;
  return vin;
}

/* ===================== LOGIC ===================== */
static BattState computeBatt(float v){
  if(!isfinite(v) || v < VBAT_NOT_PRESENT_TH) return BAT_NOT_PRESENT;

  static bool lvc=false, crit=false, low=false;

  if(!lvc && v<=VBAT_LVC_TH) lvc=true;
  if(lvc && v>=VBAT_LVC_REC) lvc=false;
  if(lvc) return BAT_LVC;

  if(!crit && v<VBAT_CRIT_TH) crit=true;
  if(crit && v>=VBAT_CRIT_REC) crit=false;
  if(crit) return BAT_CRITICAL;

  if(!low && v<VBAT_LOW_TH) low=true;
  if(low && v>=VBAT_LOW_REC) low=false;
  if(low) return BAT_LOW;

  return BAT_OK;
}

static void updateAcPresent(float vin){
  if(!acPresent){
    if(isfinite(vin) && (vin > VIN_PRESENT_ON)){
      acPresent = true;
      acReturnMs = millis();
    }
  } else {
    if(!isfinite(vin) || (vin < VIN_PRESENT_OFF)){
      acPresent = false;
    }
  }
}

static Measurements readAll(){
  Measurements m{};
  m.sh1Sat = false;
  m.sh2Sat = false;

  float vin = readVinAdc();

  float vsh1 = shuntV(INA1_ADDR);
  if(isfinite(vsh1) && fabsf(vsh1) > 0.319f) m.sh1Sat = true;
  float ichg = m.sh1Sat ? NAN : currA_fromShunt(vsh1);

  float vbat = busV(INA2_ADDR);
  float vsh2 = shuntV(INA2_ADDR);
  if(isfinite(vsh2) && fabsf(vsh2) > 0.319f) m.sh2Sat = true;
  float iload = m.sh2Sat ? NAN : currA_fromShunt(vsh2);

  // ✅ ILOAD uvek pozitivno kao potrošnja
  if(isfinite(iload) && iload < 0) iload = -iload;

  bool batMissing = (!isfinite(vbat) || vbat < VBAT_NOT_PRESENT_TH);
  if(batMissing){
    ichg = 0.0f;
    iload = 0.0f;
  }

  if(isfinite(ichg) && fabsf(ichg) < 0.02f) ichg = 0.0f;
  if(isfinite(iload) && fabsf(iload) < 0.01f) iload = 0.0f;

  float net = (isfinite(ichg) && isfinite(iload)) ? (ichg - iload) : NAN;

  m.Vin  = vin;
  m.Vbat = vbat;
  m.Ichg = ichg;
  m.Iload= iload;
  m.Inet = net;

  return m;
}

static void applyRules(const Measurements& m){
  battState = computeBatt(m.Vbat);
  updateAcPresent(m.Vin);

  // ✅ AO3400 LOAD LVC (nezavisno od charge logike)
  if(battState == BAT_NOT_PRESENT || !isfinite(m.Vbat)){
    loadEnabled = false;
  } else {
    if(loadEnabled){
      if(m.Vbat <= LOAD_OFF_VBAT) loadEnabled = false;
    } else {
      if(m.Vbat >= LOAD_ON_VBAT)  loadEnabled = true;
    }
  }

  bool hardBlock = (battState==BAT_NOT_PRESENT || battState==BAT_LVC || !acPresent);
  bool inDelay   = (acPresent && (millis()-acReturnMs < AC_RETURN_DELAY_MS));

  if(hardBlock || inDelay){
    chargeEnabled = false;
  } else {
    if(chargeEnabled){
      if(isfinite(m.Vbat) && m.Vbat >= CHG_OFF_VBAT) chargeEnabled = false;
    } else {
      if(isfinite(m.Vbat) && m.Vbat <= CHG_ON_VBAT)  chargeEnabled = true;
    }
  }

  bool fault = (battState==BAT_NOT_PRESENT || battState==BAT_LVC);
  sysState = fault ? SYS_FAULT :
             (isfinite(m.Ichg) && m.Ichg > 0.05f && chargeEnabled) ? SYS_CHARGING :
             (isfinite(m.Iload) && m.Iload > 0.02f && !chargeEnabled) ? SYS_DISCHARGE :
             SYS_IDLE;

  // nikad ne zamrzavamo prikaz
  freezeBars = false;

  // Outputs
  digitalWrite(PIN_CHG_MOSFET, chargeEnabled ? HIGH : LOW);
  digitalWrite(PIN_STATUS_LED, chargeEnabled ? HIGH : LOW);

  // ✅ AO3400 gate: HIGH=LOAD ON, LOW=LOAD OFF
  digitalWrite(PIN_LOAD_MOSFET, loadEnabled ? HIGH : LOW);
}

/* ===================== UI ===================== */
static void drawStatic(){
  tft.fillScreen(COL_BG);
  tft.setTextSize(TEXTSIZE);
  tft.setTextWrap(false);

  tft.setTextColor(COL_LAB, COL_BG);
  tft.setCursor(X_LEFT, Y_STATUS+4);
  tft.print("MODE:");

  tft.setTextColor(COL_LAB, COL_BG);
  tft.setCursor(X_LABEL, YLBL_VIN);   tft.print("VIN");
  tft.setCursor(X_LABEL, YLBL_ICHG);  tft.print("ICHG");
  tft.setCursor(X_LABEL, YLBL_VBAT);  tft.print("VBAT");
  tft.setCursor(X_LABEL, YLBL_ILOAD); tft.print("ILOAD");
  tft.setCursor(X_LABEL, YLBL_NET);   tft.print("NET");

  tft.drawRect(BAR_X, Y_VIN  +BAR_Y_OFF, BAR_W, BAR_H, COL_DIM);
  tft.drawRect(BAR_X, Y_ICHG +BAR_Y_OFF, BAR_W, BAR_H, COL_DIM);
  tft.drawRect(BAR_X, Y_VBAT +BAR_Y_OFF, BAR_W, BAR_H, COL_DIM);
  tft.drawRect(BAR_X, Y_ILOAD+BAR_Y_OFF, BAR_W, BAR_H, COL_DIM);
  tft.drawRect(BAR_X, Y_NET  +BAR_Y_OFF, BAR_W, BAR_H, COL_DIM);

  tft.setTextSize(1);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setCursor(160, Y_STATUS+2);
  char buf[32];
  snprintf(buf, sizeof(buf), "INA1=0x%02X INA2=0x%02X", INA1_ADDR, INA2_ADDR);
  tft.print(buf);
  tft.setTextSize(TEXTSIZE);
}

static void drawStatusLine(const Measurements& m){
  const char* mode = "OFF";
  if(chargeEnabled){
    mode = (isfinite(m.Ichg) && m.Ichg > 0.05f) ? "BULK" : "FLOAT";
  }
  bool showRestored = (acPresent && (millis()-acReturnMs < AC_RETURN_DELAY_MS));
  const char* modeTxt = showRestored ? "AC RESTORED" : mode;

  char diag[24];
  diag[0] = 0;

  // Prioriteti dijagnostike
  if(battState == BAT_NOT_PRESENT) {
    snprintf(diag, sizeof(diag), "BAT MISS");
  } else if(!loadEnabled) {
    snprintf(diag, sizeof(diag), "LOAD OFF");
  } else if(m.sh1Sat && m.sh2Sat) {
    snprintf(diag, sizeof(diag), "SH SAT");
  } else if(m.sh1Sat) {
    snprintf(diag, sizeof(diag), "SH1 SAT");
  } else if(m.sh2Sat) {
    snprintf(diag, sizeof(diag), "SH2 SAT");
  } else if(chargeEnabled && isfinite(m.Ichg) && m.Ichg > (I_CHG_MAX + 0.05f)) {
    snprintf(diag, sizeof(diag), "ICHG HI");
  } else if(sysState == SYS_FAULT) {
    snprintf(diag, sizeof(diag), "FAULT");
  }

  if(strcmp(lastMode, modeTxt)!=0){
    strncpy(lastMode, modeTxt, sizeof(lastMode)-1);
    lastMode[sizeof(lastMode)-1]=0;
    printFixed(72, Y_STATUS+4, lastMode, 12, COL_MODE);
  }

  if(strcmp(lastDiag, diag)!=0){
    strncpy(lastDiag, diag, sizeof(lastDiag)-1);
    lastDiag[sizeof(lastDiag)-1]=0;

    uint16_t c =
      (strcmp(diag,"BAT MISS")==0) ? COL_LVC :
      (strcmp(diag,"LOAD OFF")==0) ? COL_LVC :
      (strstr(diag,"SAT")!=nullptr) ? COL_CRIT :
      (strcmp(diag,"ICHG HI")==0)   ? COL_CRIT :
      (strcmp(diag,"FAULT")==0)     ? battColor(battState) :
      COL_DIM;

    printFixed(200, Y_STATUS+4, lastDiag, 10, c);
  }
}

static void drawValueWithArrowCached(int x, int y, const char* s, char arrow,
                                     char* cacheStr, size_t cacheStrSz,
                                     char &cacheArrow, uint16_t col, int widthChars){
  bool ch = (strncmp(cacheStr, s, cacheStrSz)!=0) || (cacheArrow != arrow);
  if(!ch) return;
  strncpy(cacheStr, s, cacheStrSz-1);
  cacheStr[cacheStrSz-1]=0;
  cacheArrow = arrow;

  printFixed(x, y, cacheStr, widthChars, col);
  char a[2]={cacheArrow,0};
  printFixed(x + NUM_W_PX + 2, y, a, 1, col);
}

static void drawNetBarDelta(float netA){
  int cx = BAR_X + (BAR_W/2);
  int yy = (Y_NET + BAR_Y_OFF) + 1;
  int hh = BAR_H-2;

  int newPos=0, newNeg=0;
  if(isfinite(netA)){
    if(netA >= 0){
      float n = clamp01(netA / NET_POS_MAX);
      newPos = (int)(n * (BAR_W/2 - 2));
      newNeg = 0;
    } else {
      float n = clamp01((-netA) / (-NET_NEG_MIN));
      newNeg = (int)(n * (BAR_W/2 - 2));
      newPos = 0;
    }
  }

  if(newPos == w_net_pos && newNeg == w_net_neg) return;

  int posX = cx+1;
  if(newPos != w_net_pos){
    if(newPos > w_net_pos) tft.fillRect(posX + w_net_pos, yy, newPos-w_net_pos, hh, COL_ICHG);
    else                   tft.fillRect(posX + newPos,   yy, w_net_pos-newPos, hh, COL_BG);
    w_net_pos = newPos;
  }

  int leftEnd = cx-1;
  if(newNeg != w_net_neg){
    if(newNeg > w_net_neg){
      tft.fillRect(leftEnd - newNeg + 1, yy, newNeg-w_net_neg, hh, COL_ILOAD);
    } else {
      tft.fillRect(leftEnd - w_net_neg + 1, yy, w_net_neg-newNeg, hh, COL_BG);
    }
    w_net_neg = newNeg;
  }

  tft.drawFastVLine(cx, (Y_NET + BAR_Y_OFF) + 1, BAR_H-2, COL_NETC);
}

static void updateFooterBATOnly(){
  static BattState last = (BattState)255;
  if(last == battState) return;
  last = battState;

  tft.setTextSize(TEXTSIZE);
  tft.setTextWrap(false);

  tft.setTextColor(COL_LAB, COL_BG);
  tft.setCursor(X_FOOT_BATLAB, Y_FOOT);
  tft.print("BAT:");

  const char* bt = battText(battState);
  uint16_t btCol = battColor(battState);
  printFixed(X_FOOT_BATVAL + CHAR_W, Y_FOOT, bt, 6, btCol);
}

static float pickShownOrLive(float &shown, float live, float db, bool freeze){
  if(!isfinite(live)) return shown;
  if(!isfinite(shown)) return live;
  if(freeze) return shown;
  return deadzoneHold(shown, live, db);
}

static void updateUI(const Measurements& m){
  float vin  = pickShownOrLive(shownVin,  m.Vin,  0.02f, freezeBars);
  float ichg = pickShownOrLive(shownIchg, m.Ichg, 0.01f, freezeBars);
  float vbat = pickShownOrLive(shownVbat, m.Vbat, 0.02f, freezeBars);
  float iload= pickShownOrLive(shownIload,m.Iload,0.01f, freezeBars);
  float inet = pickShownOrLive(shownInet, m.Inet, 0.01f, freezeBars);

  char a_vin= trendArrow(shownVin,  vin,  0.02f);
  char a_i  = trendArrow(shownIchg, ichg, 0.01f);
  char a_v  = trendArrow(shownVbat, vbat, 0.02f);
  char a_l  = trendArrow(shownIload,iload,0.01f);
  char a_n  = trendArrow(shownInet, inet, 0.01f);

  shownVin=vin; shownIchg=ichg; shownVbat=vbat; shownIload=iload; shownInet=inet;

  drawStatusLine(m);

  char buf[20];

  if(!isfinite(vin)) snprintf(buf,sizeof(buf),"--.--V");
  else snprintf(buf,sizeof(buf),"%5.2fV", vin);
  drawValueWithArrowCached(X_NUM, Y_VIN, buf, a_vin, lastVINs, sizeof(lastVINs), lastAVin,
                           COL_VIN, NUM_W_CHARS);

  if(!isfinite(ichg)) snprintf(buf,sizeof(buf),"-.---A");
  else snprintf(buf,sizeof(buf),"%5.3fA", ichg);
  drawValueWithArrowCached(X_NUM, Y_ICHG, buf, a_i, lastICHGs, sizeof(lastICHGs), lastAI,
                           COL_ICHG, NUM_W_CHARS);

  if(!isfinite(vbat)) snprintf(buf,sizeof(buf),"--.--V");
  else snprintf(buf,sizeof(buf),"%5.2fV", vbat);
  drawValueWithArrowCached(X_NUM, Y_VBAT, buf, a_v, lastVBATs, sizeof(lastVBATs), lastAV,
                           vbatGradient(vbat), NUM_W_CHARS);

  if(!isfinite(iload)) snprintf(buf,sizeof(buf),"-.---A");
  else snprintf(buf,sizeof(buf),"%5.3fA", iload);
  drawValueWithArrowCached(X_NUM, Y_ILOAD, buf, a_l, lastILOADs, sizeof(lastILOADs), lastAL,
                           COL_ILOAD, NUM_W_CHARS);

  if(!isfinite(inet)) snprintf(buf,sizeof(buf),"-.---A");
  else snprintf(buf,sizeof(buf),"%+6.3fA", inet);
  uint16_t netCol = (!isfinite(inet)) ? COL_WHT : (inet>=0 ? COL_ICHG : COL_ILOAD);
  drawValueWithArrowCached(X_NUM, Y_NET, buf, a_n, lastNETs, sizeof(lastNETs), lastAN,
                           netCol, NUM_W_CHARS);

  // Barovi se uvek crtaju (freezeBars je false)
  int maxW = (BAR_W-2);

  float nvin = isfinite(vin) ? (vin - VIN_MIN) / (VIN_MAX - VIN_MIN) : 0;
  int wNew = (int)(clamp01(nvin) * maxW);
  barDeltaDraw(Y_VIN + BAR_Y_OFF, w_vin, wNew, COL_VIN);

  float ni = isfinite(ichg) ? (ichg - ICHG_MIN) / (ICHG_MAX - ICHG_MIN) : 0;
  wNew = (int)(clamp01(ni) * maxW);
  barDeltaDraw(Y_ICHG + BAR_Y_OFF, w_ichg, wNew, COL_ICHG);

  float nv = isfinite(vbat) ? (vbat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) : 0;
  wNew = (int)(clamp01(nv) * maxW);
  barDeltaDraw(Y_VBAT + BAR_Y_OFF, w_vbat, wNew, vbatGradient(vbat));

  float nl = isfinite(iload)? (iload - ILOAD_MIN)/(ILOAD_MAX-ILOAD_MIN) : 0;
  wNew = (int)(clamp01(nl) * maxW);
  barDeltaDraw(Y_ILOAD + BAR_Y_OFF, w_iload, wNew, COL_ILOAD);

  drawNetBarDelta(inet);

  updateFooterBATOnly();
}

/* ===================== TFT RECOVER ===================== */
static void resetUiCaches(){
  w_vin=w_vbat=w_ichg=w_iload=0;
  w_net_pos=w_net_neg=0;

  shownVin=shownVbat=shownIchg=shownIload=shownInet=NAN;

  lastMode[0]=0; lastDiag[0]=0;
  lastVINs[0]=lastVBATs[0]=lastICHGs[0]=lastILOADs[0]=lastNETs[0]=0;
  lastAVin=lastAV=lastAI=lastAL=lastAN=' ';
}

static void tftRecover(){
  uint32_t now = millis();
  if(now - lastTftRecoverMs < TFT_RECOVER_COOLDOWN_MS) return;
  lastTftRecoverMs = now;

  tft.begin();
  tft.setRotation(1);
  drawStatic();
  resetUiCaches();
}

/* ===================== SETUP / LOOP ===================== */
void setup(){
  pinMode(PIN_CHG_MOSFET, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);

  // ✅ AO3400 LOAD pin
  pinMode(PIN_LOAD_MOSFET, OUTPUT);

  digitalWrite(PIN_CHG_MOSFET, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);

  // ✅ start: LOAD OFF dok ne izmeri VBAT i odluči
  digitalWrite(PIN_LOAD_MOSFET, LOW);
  loadEnabled = false;

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VCHG_ADC, ADC_11db);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(1);

  drawStatic();

  (void)ina219Init(INA1_ADDR);
  (void)ina219Init(INA2_ADDR);

  resetUiCaches();
  lastAcPresent = false;
}

void loop(){
  static uint32_t last=0;
  if(millis()-last < 250) return;
  last = millis();

  Measurements m = readAll();
  applyRules(m);

  // ✅ Detekcija VIN povratka: OFF -> ON
  if(!lastAcPresent && acPresent){
    tftRecover();
  }
  lastAcPresent = acPresent;

  updateUI(m);
}