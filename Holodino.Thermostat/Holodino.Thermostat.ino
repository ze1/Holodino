#include <PID_v1.h>
#include <Sodaq_DS3231.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PRODUCT       "HOLODINO"

#define TEMP_TARGET       -18.0
#define INITIAL_PID_P        30
#define INITIAL_PID_I        60
#define INITIAL_PID_D       270

#define START_DELAY         270
#define WINDOW_SIZE        1200

//#define TEST_MODE
#ifdef TEST_MODE
#define TIME_SPEED          10.0
#define TEMP_INITIAL      +20.0
#define TEMP_RATE_ON     -0.10
#define TEMP_RATE_OFF     0.01
#endif

// OUTPUT: MOTOR RELAY SIGNAL PIN#
// ==================================
//  + Connect IN  to digital pin
//  + Connect VCC to power 5V DC
//  + Connect GND to common ground
#define OUTPUT_PIN            6

// DISPLAY: INTERNAL INFO
// ==================================
#define OLED_MOSI             9 // D1
#define OLED_CLK             10 // D0
#define OLED_DC              11 //
#define OLED_CS              12 //
#define OLED_RESET           13 //
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// ************** HOLODINO TIMER ************* //

typedef   signed long  sec;
typedef unsigned long msec;

class Timer {
public:
    Timer(double speed) : speed_(speed) { Reset(); }
    virtual void Reset() { offset_ = millis(); }
    virtual msec MilliSeconds() { return (millis() - offset_) * speed_; }
    double speed() { return speed_; }
private:
    msec offset_;
    double speed_;
};

struct History {
    unsigned char Index;
    signed char Data[128];
    signed char Min, Max;
    void Add(signed char value) {
        if (Index >= sizeof(Data)) {
            Min = Max = Data[0];
            for (Index = 0; Index < sizeof(Data) - 1; Index++) {
                signed char val = Data[Index + 1];
                if (val < Min) Min = val;
                if (val > Max) Max = val;
                Data[Index] = val;
            }
        }
        if (!Index || value < Min) Min = value;
        if (!Index || value > Max) Max = value;
        Data[Index++] = value;
    }
}
history;

// **************** HOLODINO STATE ***************** //

class Controller {
public:
    Controller(double speed, sec duration, sec cooldown) :
        timer_(speed),
        duration_(duration * 1000),
        cooldown_(cooldown * 1000), 
        start_(), hist_(speed), state_(), output_pin_(OUTPUT_PIN),
        input_(-273), output_(), target_(TEMP_TARGET), pid_(&input_, &output_, &target_, INITIAL_PID_P, INITIAL_PID_I, INITIAL_PID_D, REVERSE) {
    }
    virtual bool Init() {
        pid_.SetSampleTime(1000 / timer_.speed());
        pid_.SetOutputLimits(30, (duration_ - cooldown_) / 1000); // tell the PID to range between 0 and the full window size excluding cooldown time
        pid_.SetMode(AUTOMATIC); // turn the PID controller on
        pinMode(output_pin_, OUTPUT); // configure a pin for relay
        digitalWrite(output_pin_, HIGH);
        rtc.begin();
        return true;
    }
    virtual bool Input(double&) = 0;
    virtual bool Output(bool signal) {
        digitalWrite(output_pin_, signal ? LOW : HIGH); // turn the output relay pin on/off based on the state
        return true;
    }
    double speed() { return timer_.speed(); }
    double input() { return input_; }
    double output() { return output_; }
    double target() { return target_; }
    enum States { stInit = 0, stStart, stOn, stOff };
    static const char* StateStr(States s) { return s == stInit ? "INIT" : s == stStart ? "DELAY" : s == stOn ? "*ON*" : s == stOff ? "OFF" : "ERROR"; }
    States state() { return state_; }
    virtual bool State(States s) {
        if (s == state_) return false;
        if (s == stStart) timer_.Reset();
        state_ = s;
        start_ = timer_.MilliSeconds();
        return true;
    }
    msec TimeElapsed() { return timer_.MilliSeconds() - start_; }
    msec TimeLeft() {
        msec now = timer_.MilliSeconds();
        msec end = state_ == stInit ? 1000000000
            : (state_ == stStart ? cooldown_
                : (state_ == stOn ? start_ + (output_ < 60.0 ? 0 : output_ * 1000)
                    : duration_));
        return end > now ? end - now : 0;
    }
    States Execute() {
        if (state_ == stInit && Init()) State(stStart);
        if (state_ != stInit) {
            if (!Input(input_)) State(stInit);
            else {
                pid_.Compute();
                if (!TimeLeft())
                    if (state_ == stStart) State(stOn);
                    else
                        if (state_ == stOn) State(stOff);
                        else State(stStart);
                        if (!history.Index ||
                            hist_.MilliSeconds() >= 60000) {
                            hist_.Reset();
                            history.Add(input_);
                        }
            }
        }
        Output(state_ == stOn);
        return state_;
    }
private:
    Timer timer_;
    msec duration_;
    msec cooldown_;
    msec start_;
    Timer hist_;
    States state_;
    int output_pin_;
    double input_;
    double output_;
    double target_;
    PID pid_;
};

// ***************** HOLODINO ***************** //

#ifndef TEST_MODE

class ControllerRTC : public Controller {
public:
    ControllerRTC() :
        Controller(1.0, WINDOW_SIZE, START_DELAY) {
    }
    virtual bool Input(double &temp) {
        rtc.convertTemperature();
        temp = rtc.getTemperature();
        return true;
    }
};

ControllerRTC holod;

#else

class ControllerTest : public Controller {
public:
    ControllerTest() :
        Controller(TIME_SPEED, WINDOW_SIZE, START_DELAY),
        temp_(TEMP_INITIAL) {
    }
    virtual bool State(States s) {
        if (s != this->state()) Input(temp_);
        return Controller::State(s);
    }
    virtual bool Input(double &temp) {
        temp = temp_ + (state() == stOn ? TEMP_RATE_ON : TEMP_RATE_OFF) * TimeElapsed() / 1000;
        return true;
    }
private:
    double temp_;
};

ControllerTest holod;

#endif // TEST_MODE

void setup() {
    display.begin(SSD1306_SWITCHCAPVCC); // by default, we'll generate the high voltage from the 3.3v line internally
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.setTextColor(WHITE);
    display.println(PRODUCT);
    display.display();
}

#define VIEW_WIDTH 128
#define VIEW_HEIGHT 64
#define HIST_HEIGHT 28

void loop() {

    // EXECUTE PID CONTROLLER CALCULATIONS
    Controller::States state = holod.Execute();

    // DISPLAY INFO 
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.setTextColor(WHITE);

    // Print current and target temperature values: t=-13.94C => -18.00C
    display.print("t="); if (holod.input() > 0) display.print("+"); display.print(holod.input()); display.print("C");
    display.print(holod.input() < holod.target() ? " <= " : " => "); if (holod.target() > 0) display.print("+"); display.print(holod.target()); display.println("C");

    // Print time left, current state and time elapsed: +23 << OFF << -742
    display.print("+"); display.print(holod.TimeElapsed() / 1000); display.print(" << "); display.print(holod.StateStr(state));
    sec left = holod.TimeLeft() / 1000; if (left <= 3600) { display.print(" << -"); display.print(left); } display.println("");

    // Print calculated output value, duration in seconds of the ON state: == 920 sec ==
    display.print("  == "); display.print(holod.output()); display.println(" sec ==");

    // Display historical data chart: temperature by minutes
    int hist_range = history.Max + 1 - history.Min,
        hist_space = HIST_HEIGHT - hist_range;
    float hist_scale = hist_space < 0 ? (float)HIST_HEIGHT / hist_range : 1.0f;
    int hist_hi = VIEW_HEIGHT - HIST_HEIGHT - 4 + (hist_space < 0 ? 0 : hist_space / 2),
        hist_lo = hist_hi + hist_scale * hist_range,
        hist_inp = hist_hi + hist_scale * (history.Max - round(holod.input()));
    for (int i = 0, x = VIEW_WIDTH - history.Index; i <= history.Index; ++i, ++x) { display.drawPixel(x, hist_hi + hist_scale * (history.Max - history.Data[i]), WHITE); }
    for (int x = 30; x < VIEW_WIDTH; x += 6) { display.drawPixel(x, hist_hi, WHITE); display.drawPixel(x, hist_lo, WHITE); }
    for (int x = 40; x < VIEW_WIDTH; x += 2) { display.drawPixel(x, hist_inp, WHITE); }
    int d = hist_lo - hist_hi; if (d <= 8) { hist_hi -= 4 - d / 2; hist_lo += 4 - (d - d / 2); }
    display.setCursor(0, hist_hi - 3); display.print("hi:"); if (history.Max > 0) display.print("+"); display.print(history.Max);
    display.setCursor(0, hist_lo - 3); display.print("lo:"); if (history.Min > 0) display.print("+"); display.print(history.Min);
    display.display();

    // Delay before next iteration
    delay(1000 / holod.speed());
}

