
#define PRODUCT "HOLODINO"
#define VERSION "v.1.0"
#define MODULE  "Thermostat"
#define AUTHOR  "A.Olkhovoy"
#define EMAIL   "ao@ze1.org"

#include <Wire.h>
#include <PID_v1.h>
#include <Sodaq_DS3231.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085_U.h>

// SETUP: PID CONTROLLER PARAMETERS
// ================================

#define DELAY_TIME  4 * 60
#define WINDOW_SIZE 20 * 60

#define TEST_MODE

#ifdef TEST_MODE
    #define TEST_TIME_SPEED          10.000
    #define TEST_TEMP_START         +20.000
    #define TEST_TEMP_SIGNAL_IDLE    +0.005
    #define TEST_TEMP_SIGNAL_ACTIVE  -0.010
#endif

enum State {sigNone, sigDelay, sigActive, sigIdle};
State state = sigNone; // signal phase in the current window
int    state_time = 0; // time when signal changed to it's current state
double temperature_history[128] = {}; // temperature when signal just changed to it's current state
double *temperature= NULL; // pointer to the most recent temperature value
int    window_start = 0; // response delay window start time

double Setpoint, Input, Output; // variables we'll be connecting to
PID pid(&Input, &Output, &Setpoint, 20, 50, 10, REVERSE); // the links and initial tuning parameters

// INPUT: BARO/THERMO MODULE (BMP085/BMP180)
// =========================================
//  + Connect SCL to analog 5
//  + Connect SDA to analog 4
//  + Connect VDD to +3.3V DC
//  + Connect GROUND to common ground
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
bool bmp_ready = false;

// OUTPUT: MOTOR RELAY DIGITAL SIGNAL
// ==================================
//  + Connect IN  to digital pin
//  + Connect VCC to power 5V DC
//  + Connect GND to common ground
const int OutputPin = 3; // =========

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
    pid.SetOutputLimits(0, WINDOW_SIZE - START_DELAY); // tell the PID to range between 0 and the full window size
    pid.SetMode(AUTOMATIC); // turn the PID controller on

    pinMode(OutputPin, OUTPUT); 

    // by default, we'll generate the high voltage from the 3.3v line internally
    display.begin(SSD1306_SWITCHCAPVCC);
    display.display();
}

// ******************* HOLODINO LOOP ******************** //
void loop() {

    // GET CURRENT TIME
    int now = millis() / 1000; // current time, seconds

    // INPUT: GET TEMPERATURE
    float temperature = -1000; // current temperature

#ifdef TEST_MODE

    // Test environment uses defined constants for simulation
    now *= TEST_TIME_SPEED;
    tmp =  + (now - SignalTime) * 
        (Signal ? TEST_HIGH_DEGREES_PER_SEC : TESP_LOW_DEGREES_PER_SEC);
#else

    // INIT: BARO/TEMP SENSOR
    if (!bmp_ready) {
        if (!bmp.begin()) {
            Serial.println("BARO: INIT ERROR");
            return delay(1000);
        }
        else {
            bmp_ready = true;
            Serial.println("BARO: INIT SUCCESS");
            WindowStart = now - WINDOW_SIZE;
        }
    }

    bmp.getTemperature(&temperature);
    if (temperature < -300) {

        Serial.print("BARO: INVALID TEMP ");
        Serial.print(temperature);
        Serial.println("C");
    }

#endif

    if (temperature > -300) {

        // PROCESS: PID CONTROLLER
        Input = temperature;
        pid.Compute();
    }

    int pos = now - WindowStart;
    if (pos > WINDOW_SIZE) {

        pos = 0;
        Signal = false;
        SignalTime = now;
        SignalTemp = temperature;
        SignalPhase = SignalPhases::sigNone;
        WindowStart += WINDOW_SIZE; // shift the Relay Window
    }

    // OUTPUT: RELAY SIGNAL - ON/OFF MOTOR
    bool sig = false;
    int out = Output < 30 ? 0 : Output;
    if (pos >= START_DELAY && SignalPhase != SignalPhases::sigLow) {
        if (pos < START_DELAY + out) {
            sig = true;
            SignalPhase = SignalPhases::sigHigh;
        }
        else
            SignalPhase = SignalPhases::sigLow;
    }
    if (Signal != sig) {
        Signal = sig;
        SignalTime = now;
        SignalTemp = temperature;
    }
    digitalWrite(OutputPin, Signal ? HIGH : LOW); // turn the output pin on/off based on pid output

    // UPDATE SCREEN INFO
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.setTextColor(WHITE);
    display.println("***" PRODUCT " " VERSION "***");
    display.print(temperature < 0 ? "" : "+"); display.print(temp); display.print("C => ");
    display.print(Setpoint < 0 ? "" : "+"); display.print(Setpoint); display.println("C");
    display.print(Signal ? "sig:ON   " : "sig:OFF  ");
    display.print("dif:"); display.print(temp - SignalTemp); display.println("C");
    display.print("pos:"); display.print(pos - START_DELAY); display.print("s ");
    display.print("out:"); display.print(out); display.println("s");
    display.display();
    display.invertDisplay(Signal);
}
