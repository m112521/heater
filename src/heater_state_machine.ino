#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <QuadDisplay2.h>

// ---------- Pins ----------
#define CS_PIN        7
#define ONE_WIRE_BUS  10
#define HEATER_PIN    9
#define POT_PIN       A0   // setpoint potentiometer (used in edit mode)
#define BTN1_PIN      5    // BTN1: show current setpoint for 3 s
#define BTN2_PIN      4    // BTN2: enter edit mode / confirm new setpoint

// Level read when a button is pressed.
// HIGH = button connects pin to VCC (external pull-down).
// LOW  = button connects pin to GND (pull-up).
#define BTN_ACTIVE_LEVEL HIGH

// ---------- Settings ----------
const double SETPOINT_MIN     = 30.0;
const double SETPOINT_MAX     = 60.0;
const double SETPOINT_STEP    = 0.5;
const double SETPOINT_DEFAULT = 40.0;

const int POT_DEADBAND = 8;          // ADC counts the pot must move to take over the value
const float POT_FILTER = 0.2;        // smoothing, 0..1 (lower = smoother)

const unsigned long DEBOUNCE_MS          = 30;
const unsigned long SHOW_SETPOINT_MS     = 3000;
const unsigned long EDIT_TIMEOUT_MS      = 3000;  // cancel edit after this long without pot movement
const unsigned long DISPLAY_REFRESH_MS   = 100;

const int sampleTime = 800;

// ---------- Objects ----------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
QuadDisplay qd(CS_PIN);

double Setpoint, Input, Output;
double Kp = 80, Ki = 0.5, Kd = 40;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long lastTime = 0;
unsigned long lastDisplay = 0;
bool sensorOk = true;

// ---------- Buttons (debounced, returns true once per press) ----------
struct Button {
  uint8_t pin;
  bool stableState;
  bool lastRaw;
  unsigned long lastChange;
};

Button btn1 = { BTN1_PIN, false, false, 0 };
Button btn2 = { BTN2_PIN, false, false, 0 };

bool buttonPressed(Button &b) {
  unsigned long now = millis();
  bool raw = (digitalRead(b.pin) == BTN_ACTIVE_LEVEL);

  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastChange = now;
  }

  if ((now - b.lastChange) >= DEBOUNCE_MS && raw != b.stableState) {
    b.stableState = raw;
    if (raw) return true;
  }
  return false;
}

// ---------- Potentiometer ----------
float potFiltered = 0;
int potAtEditStart = 0;
bool potTookOver = false;
double editValue = 0;

double potToSetpoint(float raw) {
  double t = SETPOINT_MIN + (raw / 1023.0) * (SETPOINT_MAX - SETPOINT_MIN);
  t = round(t / SETPOINT_STEP) * SETPOINT_STEP;
  return constrain(t, SETPOINT_MIN, SETPOINT_MAX);
}

void updatePotFilter() {
  potFiltered += ((float)analogRead(POT_PIN) - potFiltered) * POT_FILTER;
}

// ---------- State machine ----------
enum State { STATE_SHOW_TEMP, STATE_SHOW_SETPOINT, STATE_EDIT_SETPOINT };
State state = STATE_SHOW_TEMP;
unsigned long stateEnteredAt = 0;
unsigned long lastEditActivity = 0;

void enterState(State s) {
  state = s;
  stateEnteredAt = millis();
  lastDisplay = 0;  // force immediate display refresh

  if (s == STATE_EDIT_SETPOINT) {
    // Start from the current setpoint; the pot takes over only after it is moved,
    // so the value does not jump to wherever the knob happens to be.
    editValue = Setpoint;
    lastEditActivity = millis();
    potAtEditStart = (int)potFiltered;
    potTookOver = false;
  }
}

void handleUI() {
  updatePotFilter();
  bool b1 = buttonPressed(btn1);
  bool b2 = buttonPressed(btn2);

  switch (state) {
    case STATE_SHOW_TEMP:
      if (b1)      enterState(STATE_SHOW_SETPOINT);
      else if (b2) enterState(STATE_EDIT_SETPOINT);
      break;

    case STATE_SHOW_SETPOINT:
      if (b2) {
        enterState(STATE_EDIT_SETPOINT);
      } else if (b1) {
        stateEnteredAt = millis();           // restart 3 s timer
      } else if (millis() - stateEnteredAt >= SHOW_SETPOINT_MS) {
        enterState(STATE_SHOW_TEMP);
      }
      break;

    case STATE_EDIT_SETPOINT:
      if (!potTookOver && abs((int)potFiltered - potAtEditStart) >= POT_DEADBAND) {
        potTookOver = true;
      }
      if (potTookOver) {
        double v = potToSetpoint(potFiltered);
        if (v != editValue) {
          editValue = v;
          lastEditActivity = millis();       // pot moved -> restart timeout
          lastDisplay = 0;
        }
      }

      if (b2) {
        Setpoint = editValue;                // submit
        enterState(STATE_SHOW_TEMP);
      } else if (millis() - lastEditActivity >= EDIT_TIMEOUT_MS) {
        enterState(STATE_SHOW_TEMP);         // auto-cancel, new value discarded
      }
      break;
  }
}

void updateDisplay() {
  if (millis() - lastDisplay < DISPLAY_REFRESH_MS) return;
  lastDisplay = millis();

  switch (state) {
    case STATE_SHOW_TEMP:
      if (sensorOk) qd.displayFloat(Input, 1);
      else          qd.displayFloat(-99.9, 1);  // sensor error indication
      break;
    case STATE_SHOW_SETPOINT:
      qd.displayFloat(Setpoint, 1);
      break;
    case STATE_EDIT_SETPOINT:
      qd.displayFloat(editValue, 1);
      break;
  }
}

// ---------- Temperature / PID (non-blocking) ----------
void handleControl() {
  if (millis() - lastTime < sampleTime) return;
  lastTime = millis();

  float t = sensors.getTempCByIndex(0);
  sensors.requestTemperatures();

  if (t == DEVICE_DISCONNECTED_C || t < -100.0) {
    // Safety: no valid reading -> heater off
    sensorOk = false;
    Output = 0;
    analogWrite(HEATER_PIN, 0);
  } else {
    sensorOk = true;
    Input = t;
    myPID.Compute();
    analogWrite(HEATER_PIN, (int)Output);
  }

  Serial.print("Current_Temp:");
  Serial.print(Input);
  Serial.print(",");
  Serial.print("Target_Temp:");
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.print("PWM_Power_%:");
  Serial.println(map((long)Output, 0, 255, 0, 100));
}

void setup() {
  Serial.begin(9600);

  sensors.begin();
  sensors.setResolution(12);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  if (BTN_ACTIVE_LEVEL == HIGH) {
    pinMode(BTN1_PIN, INPUT);
    pinMode(BTN2_PIN, INPUT);
  } else {
    pinMode(BTN1_PIN, INPUT_PULLUP);
    pinMode(BTN2_PIN, INPUT_PULLUP);
  }

  potFiltered = analogRead(POT_PIN);
  Setpoint = SETPOINT_DEFAULT;

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(sampleTime);

  qd.begin();
  enterState(STATE_SHOW_TEMP);
}

void loop() {
  handleUI();
  handleControl();
  updateDisplay();
}
