#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

#define ONE_WIRE_BUS 10  
#define HEATER_PIN 9    

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

double Setpoint, Input, Output;

// Коэффициенты ПИД
double Kp = 80, Ki = 0.5, Kd = 40;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

unsigned long lastTime = 0;
const int sampleTime = 800; 

void setup() {
  Serial.begin(9600);
  
  sensors.begin();
  sensors.setResolution(12); 
  sensors.setWaitForConversion(false); 
  sensors.requestTemperatures();

  pinMode(HEATER_PIN, OUTPUT);

  Setpoint = 40.0; 

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); 
  myPID.SetSampleTime(sampleTime);
}

void loop() {
  if (millis() - lastTime >= sampleTime) {
    Input = sensors.getTempCByIndex(0); 
    
    sensors.requestTemperatures(); 
    lastTime = millis();

    myPID.Compute();
    analogWrite(HEATER_PIN, Output);

    // Формат вывода для Serial Plotter в Arduino IDE 2.x
    Serial.print("Current_Temp:");
    Serial.print(Input);
    Serial.print(",");
    Serial.print("Target_Temp:");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print("PWM_Power_%:");
    Serial.println(map(Output, 0, 255, 0, 100)); 
  }
}

