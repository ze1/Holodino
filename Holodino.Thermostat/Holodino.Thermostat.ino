#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#define PRODUCT "HOLODINO"
#define VERSION "v.1.0"
#define MODULE  "Thermostat"
#define AUTHOR  "A.Olkhovoy"
#define EMAIL   "ao@ze1.org"

// SETUP: PID CONTROLLER PARAMETERS
// ================================
#define TEMP_TARGET  -18.0
#define INITIAL_PID_P 2
#define INITIAL_PID_I 5
#define INITIAL_PID_D 10
double ctrl_input = -273, ctrl_output = 0, ctrl_target = TEMP_TARGET;
PID ctrl(&ctrl_input, &ctrl_output, &ctrl_target, INITIAL_PID_P, INITIAL_PID_I, INITIAL_PID_D, REVERSE);

#define START_DELAY   4 * 60
#define WINDOW_SIZE   20 * 60

#define TEST_MODE
#ifdef TEST_MODE
#define TEST_TIME_SPEED           10.0
#define TEST_TEMP_START          +20.0
#define TEST_TEMP_DEGREE_MSEC_ON  -0.000010
#define TEST_TEMP_DEGREE_MSEC_OFF +0.000005
#endif

// INPUT: BARO/THERMO MODULE (BMP085/BMP180)
// =========================================
//  + Connect SCL to analog 5
//  + Connect SDA to analog 4
//  + Connect VDD to +3.3V DC
//  + Connect GROUND to common ground
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// OUTPUT: MOTOR RELAY DIGITAL SIGNAL
// ==================================
//  + Connect IN  to digital pin
//  + Connect VCC to power 5V DC
//  + Connect GND to common ground
#define OUTPUT_PIN  3

// DISPLAY: MOTOR RELAY DIGITAL SIGNAL
// ==================================
#define OLED_MOSI   9 // D1
#define OLED_CLK   10 // D0
#define OLED_DC    11 //
#define OLED_CS    12 //
#define OLED_RESET 13 //
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// ************** HOLODINO TIMERS ************* //

#define MIN2(x, y) ((x) < (y) ? (x) : (y))
#define MAX2(x, y) ((x) > (y) ? (x) : (y))

typedef unsigned long msec;

class Timer {
public:
    Timer() { Reset(); }
    virtual void Reset() { start_ = millis(); }
    virtual msec Now() { return millis() - start_; }
private:
    msec start_;
};

class VarTimer : public Timer {
public:
    VarTimer(double speed) : Timer(), speed_(speed) {}
    virtual msec Now() { return Timer::Now() * speed_; }
private:
    double speed_;
};

// **************** HOLODINO STATE ***************** //

class Controller {
public:

    Controller(double speed, msec duration, msec cooldown) :
        timer_(speed), duration_(duration), cooldown_(cooldown), 
        start_(), state_(), num_() {
    }
    virtual bool Init() { return true; }
    virtual bool Input(double &data) = 0;
    virtual bool Output(bool signal) { return true; }

    enum States { stInit = 0, stStart, stOn, stOff };
    static const char* StateStr(States state) {
        switch (state) { case stInit: return "Init"; case stStart: return "Start"; case stOn: return "On"; case stOff: return "Off"; }
        return "Error";
    }
    virtual bool State(States state) {
        if (state == state_) return false;
        if (state == stStart) timer_.Reset();
        state_ = state;
        start_ = timer_.Now();
        return true;
    }
    States state() { return state_; }

    msec TimeElapsed() {
        return timer_.Now() - start_;
    }
    msec TimeLeft() {
        if (state_ == stInit) return (msec)-1;
        msec now = timer_.Now(),
            end = state_ == stStart ? (start_ + cooldown_) : (state_ == stOn ? (start_ + (ctrl_output < 30 ? 0 : ctrl_output * 1000)) : duration_);
        return now < end ? end - now : 0;
    }

    States Execute() {
        num_++;
        if (state_ == stInit && Init()) State(stStart);
        if (state_ != stInit) {
            if (!Input(ctrl_input)) State(stInit);
            else {
                ctrl.Compute();
                if (state_ == stStart && !TimeLeft()) State(stOn);
                if (state_ == stOn && !TimeLeft()) State(stOff);
                if (!TimeLeft() && (state_ == stOff || State(stOff))) State(stStart);
            }
        }
        Output(state_ == stOn);
        return state_;
    }
    unsigned long num() { return num_; }

private:

    VarTimer timer_;
    msec duration_;
    msec cooldown_;
    msec start_;
    States state_;
    unsigned long num_;
};

class ControllerTest : public Controller {
public:
    ControllerTest() :
        Controller(TEST_TIME_SPEED, WINDOW_SIZE * 1000, START_DELAY * 1000),
        input_(TEST_TEMP_START) {
    }
    virtual bool State(States state) override {
        if (!Controller::State(state)) return false;
        Input(input_);
        return true;
    }
    virtual bool Input(double &data) {
        data = input_ + TimeElapsed() * (state() == stOn ? TEST_TEMP_DEGREE_MSEC_ON : TEST_TEMP_DEGREE_MSEC_OFF);
        return true;
    }
private:
    double input_;
};

/*
class ControllerBaro : public Controller {
public:
    ControllerBaro() :
        Controller(TEMP_TARGET, 1.0, WINDOW_SIZE, START_DELAY),
        output_pin_(OUTPUT_PIN) {
    }
    virtual bool Initialize() {
        if (!Controller::Initialize()) return false;
        pinMode(output_pin_, OUTPUT);
        if (bmp.begin()) {
            Serial.println("BARO: INIT SUCCESS");
            return true;
        }
        Serial.println("BARO: INIT ERROR");
        return false;
    }
    virtual void Signal(bool _signal) {
        Controller::Signal(_signal);
        digitalWrite(output_pin_, Controller::Signal() ? HIGH : LOW); // turn the output pin on/off based on pid output
    }
    virtual bool Temperature(double &temperature) {
        float temp = -273.0;
        bmp.getTemperature(&temp);
        if (temp < -273.15) return false;
        temperature = temp;
        return true;
    }
private:
    int output_pin_;
};
*/

// ***************** HOLODINO ***************** //

ControllerTest holod;

void setup() {
   
    Serial.begin(9600);

    display.begin(SSD1306_SWITCHCAPVCC); // by default, we'll generate the high voltage from the 3.3v line internally
    display.display();

    ctrl.SetOutputLimits(0, WINDOW_SIZE - START_DELAY); // tell the PID to range between 0 and the full window size excluding cooldown time
    ctrl.SetMode(AUTOMATIC); // turn the PID controller on
}

void loop() {

    Controller::States state = holod.Execute();

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.setTextColor(WHITE);
    display.println("* " PRODUCT " * " VERSION " *");

    display.print(holod.TimeElapsed() / 1000); display.print(" << "); display.print(holod.StateStr(holod.state()));
    msec left = holod.TimeLeft(); if (left != (msec)-1) { display.print(" << -"); display.print(left / 1000); }
    display.println("");

    display.print("t="); if (ctrl_input > 0) display.print("+"); display.print(ctrl_input); display.print("C");
    display.print(ctrl_input < ctrl_target ? " <= " : " => "); if (ctrl_target > 0) display.print("+"); display.print(ctrl_target); display.print("C");
    display.print(" <~ "); display.print(ctrl_output); display.println(" sec");

    display.display();
    display.invertDisplay(state == Controller::stOn || (state == Controller::stInit && holod.num() % 3 == 0));

    delay(333);
}
