// ****************** HOLODINO HEAD ***************** //

#define PRODUCT "HOLODINO"
#define VERSION "v. 1.0.0"
#define MODULE  "Thermostat"
#define AUTHOR  "A.Olkhovoy"
#define EMAIL   "ao@ze1.org"

// ***************** HOLODINO DEPS ******************* //

#include <Wire.h>
#include <PID_v1.h>
#include <Sodaq_DS3231.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085_U.h>

// ***************** HOLODINO DATA ****************** //

// INPUT: BARO/THERMO MODULE (BMP085/BMP180)
// =========================================
//  + Connect SCL to analog 5
//  + Connect SDA to analog 4
//  + Connect VDD to +3.3V DC
//  + Connect GROUND to common ground
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085), *bmp_init = &bmp;

// OUTPUT: MOTOR RELAY DIGITAL SIGNAL
// ==================================
//  + Connect IN  to digital 6
//  + Connect VCC to power 5V DC
//  + Connect GND to common ground
const int OutputPin = 3; // =========

bool Signal = false;
unsigned long SignalStart, WindowStart, WindowSize = 5000; // response delay window start/end time and duration
double Setpoint, Input, Output; // variables we'll be connecting to
PID pid(&Input, &Output, &Setpoint, 2, 5, 1, REVERSE); // the links and initial tuning parameters

// DISPLAY: MOTOR RELAY DIGITAL SIGNAL
// ==================================
#define OLED_MOSI   9 // D1
#define OLED_CLK   10 // D0
#define OLED_DC    11 //
#define OLED_CS    12 //
#define OLED_RESET 13 //
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// ***************** HOLODINO INIT ***************** //
void setup() {

    Serial.begin(9600); // start logging to serial 
    Serial.println(PRODUCT " " MODULE " " VERSION " " AUTHOR);

    Setpoint = -20; // target temperature
    pid.SetOutputLimits(0, WindowSize); // tell the PID to range between 0 and the full window size
    pid.SetMode(AUTOMATIC); // turn the PID controller on

    WindowStart = millis(); // start new time interval 
    pinMode(OutputPin, OUTPUT); 

    // by default, we'll generate the high voltage from the 3.3v line internally
    display.begin(SSD1306_SWITCHCAPVCC);
    display.display();
}

// ******************* HOLODINO LOOP ******************** //
void loop() {

    // INITIALIZE: BARO/TEMP SENSOR
    if (bmp_init && bmp.begin()) bmp_init = NULL; // clear var when temperature sensor is ready

    // INPUT: GET TEMPERATURE
    float temperature = -333.0; bmp.getTemperature(&temperature);  // read value from sensor
    if (temperature < 300.0) Input = temperature; // failsafe strategy: hold last good value

    // PROCESS: PID CONTROLLER
    pid.Compute(); // calculate output value

    unsigned long now = millis(), cur = now - WindowStart;
    if (cur > WindowStart + WindowSize) WindowStart += WindowSize; // time to shift the Relay Window
                                
    // OUTPUT: RELAY SIGNAL - ON/OFF MOTOR
    int out = WindowStart + Output, sig = out < now;
    if (sig != Signal) { Signal = sig; SignalStart = now; }
    digitalWrite(OutputPin, Signal ? HIGH : LOW); // turn the output pin on/off based on pid output

    // UPDATE SCREEN INFO
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.setTextColor(WHITE);
    display.println(PRODUCT " " MODULE " " VERSION);
    display.print(temperature < 0 ? "-" : "+"); display.print(temperature); display.print("C  ");
    display.print(Signal ? "sig:ON  out:" : "sig:OFF  out:"); display.print(Output);
    display.print(out < now ? " < now:" : " > now:");
    display.print(now - WindowSize);
    display.print("  dur:");
    display.print(now - SignalStart);
    display.println("");
    display.display();
}
