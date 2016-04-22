#include <Arduino.h>
#include <PID_v1.h>
#include <Sodaq_DS3231.h>

#define PRODUCT       "HOLODINO"

#define TEMP_TARGET       -12.0
#define INITIAL_PID_P        30
#define INITIAL_PID_I        60
#define INITIAL_PID_D       270

#define START_DELAY         270
#define WINDOW_SIZE        1200

#define TEST_MODE
#ifdef TEST_MODE
#define TIME_SPEED          1.0
#define TEMP_INITIAL      +20.0
#define TEMP_RATE_ON      -0.10
#define TEMP_RATE_OFF      0.01
#endif

// DISPLAY: INTERNAL INFO
// ==================================

#define OLED_DISPLAY

#ifdef OLED_DISPLAY
	#include <Adafruit_GFX.h>
	#include <Adafruit_SSD1306.h>
	#define OLED_MOSI             9 // D1
	#define OLED_CLK             10 // D0
	#define OLED_DC              11 // DC
	#define OLED_CS              12 //  -
	#define OLED_RESET           13 // RST
	Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#endif

// OUTPUT: MOTOR RELAY SIGNAL PIN#
// ==================================
//  + Connect IN  to digital pin
//  + Connect VCC to power 5V DC
//  + Connect GND to common ground
#define OUTPUT_PIN            10

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

// ************** HOLODINO HISTORY ************* //

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
};

// **************** HOLODINO CONTROLLER ***************** //

class Controller {
public:
	enum States { stInit, stStart, stOn, stOff };
	Controller(double speed, sec duration, sec cooldown, int output_pin) :
        timer_(speed), history_timer_(speed), duration_(duration), cooldown_(cooldown), output_pin_(output_pin),
        input_(-273), output_(), target_(TEMP_TARGET), pid_(&input_, &output_, &target_, INITIAL_PID_P, INITIAL_PID_I, INITIAL_PID_D, REVERSE),
		offset_(), state_(), history_() {
    }
    virtual bool Init() {
        pid_.SetSampleTime(1000 / timer_.speed());
        pid_.SetOutputLimits(30, max(duration_ - cooldown_, 30));
        pid_.SetMode(AUTOMATIC);
        pinMode(output_pin_, OUTPUT);
		return Output(false);
    }
    virtual bool Input(double&) = 0;
    virtual bool Output(bool signal) {
        digitalWrite(output_pin_, signal ? LOW : HIGH);
        return true;
    }
    virtual bool State(States state) {
        if (state == state_) return false;
        if (state == stStart) timer_.Reset();
        state_ = state;
        offset_ = timer_.MilliSeconds();
        return true;
    }
    msec TimeElapsed() {
		return timer_.MilliSeconds() - offset_;
	}
    msec TimeLeft() {
		msec end;
		switch (state_) {
			case stOn: end = output_ * 1000; break;
			case stOff:	end = duration_ * 1000 - offset_; break;
			case stStart: end = cooldown_ * 1000 - offset_; break;
			default: end = 0;
		}
		return max((signed)(end - TimeElapsed()), 0);
    }
    States Execute() {
        if (state_ == stInit && Init()) State(stStart);
		if (state_ != stInit) {
            if (!Input(input_)) State(stInit);
            else {
				pid_.Compute();
                if (!TimeLeft()) State(state_ == stStart ? stOn : (state_ == stOn ? stOff : (state_ == stOff ? stStart : stInit)));
            }
        }
        Output(state_ == stOn);
		if (!history_.Index || history_timer_.MilliSeconds() >= 60000) {
			history_timer_.Reset();
			history_.Add(input_);
		}
		msec td = 1000 / timer_.speed(), te = TimeElapsed(), tt = (msec)(((float)te / (te + TimeLeft())) * td);
		analogWrite(11, state_ == stStart ? 96 : (state_ == stOn ? 255 : 32)); delay(tt);
		analogWrite(11, 0); delay(td - tt);
        return state_;
    }
	States state() { return state_; }
	static const char* StateStr(States s) { return s == stInit ? "INIT" : s == stStart ? "DELAY" : s == stOn ? "*ON*" : s == stOff ? "OFF" : "ERROR"; }
	double input() { return input_; }
	double output() { return output_; }
	double target() { return target_; }
	const History& history() { return history_; }
private:
    Timer timer_;
	Timer history_timer_;
	sec duration_;
    sec cooldown_;
	int output_pin_;
    double input_;
    double output_;
    double target_;
    PID pid_;
	msec offset_;
	States state_;
	History history_;
};

// ************** HOLODINO WIFI ************* //

#define esp   Serial        // use Serial1 to talk to esp8266

#define SSID  "HOLODINO"    // change this to match your WiFi SSID
#define PASS  "9164035980"  // change this to match your WiFi password
#define PORT  "80"          // using port 8080 by default

#define BUFFER_SIZE 512
char buffer[BUFFER_SIZE];

// By default we are looking for OK\r\n
char OK[] = "OK\r\n";

// wait for at most timeout milliseconds or if OK\r\n is found
byte esp_wait_for_response(unsigned long timeout = 3000, char* response = OK) {
	unsigned short index = 0, length = strlen(response);
	char *buffer = new char(length);
	bool result = false;
	timeout += millis();
	do {
		delay(1);
		while (!result && esp.available()) {
			buffer[index++ % length] = esp.read();
			result = index >= length;
			for (unsigned long r = length, b = index; result && r > 0; )
				result = buffer[--b % length] == response[--r];
		}
	}
	while (!result && millis() < timeout);
	delete buffer;
	return result;
}

// ***************** HOLODINO MAIN ***************** //

#ifndef TEST_MODE

class ControllerRTC : public Controller {
public:
    ControllerRTC() :
        Controller(1.0, WINDOW_SIZE, START_DELAY, OUTPUT_PIN) {
    }
	virtual bool Init() {
		if (!Controller::Init() || !rtc.begin() || !rtc.convertTemperature()) return false;
		return true; 
	}
    virtual bool Input(double &temp) {
        if (!rtc.convertTemperature()) return false;
        temp = rtc.getTemperature();
        return true;
    }
}
holod;

#else

class ControllerTest : public Controller {
public:
    ControllerTest() :
        Controller(TIME_SPEED, WINDOW_SIZE, START_DELAY, OUTPUT_PIN),
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
}
holod;

#endif // TEST_MODE

void setup() {

#ifdef OLED_DISPLAY	
	//display.begin(SSD1306_EXTERNALVCC);
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.setTextColor(WHITE);
    display.println(PRODUCT);
    display.display();
#endif
}

void loop() {

	// EXECUTE PID CONTROLLER CALCULATIONS
    holod.Execute();

#ifdef OLED_DISPLAY	
	// DISPLAY INFO 
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(WHITE);
    // Print current and target temperature values: t=-13.94C => -18.00C
	display.setCursor(0, 0); display.print("t="); if (holod.input() > 0) display.print("+"); display.print(holod.input()); display.print("C");
    display.print(holod.input() < holod.target() ? " <= " : " => "); if (holod.target() > 0) display.print("+"); display.print(holod.target()); display.print("C");
    // Print time left, current state and time elapsed: +23 << OFF << -742
	display.setCursor(0, 10); display.print("+"); display.print(holod.TimeElapsed() / 1000); display.print(" << "); display.print(holod.StateStr(holod.state()));
    sec left = holod.TimeLeft() / 1000; if (left <= 3600) { display.print(" << -"); display.print(left); }
    // Print calculated output value, duration in seconds of the ON state: == 920 sec ==
	display.setCursor(0, 20); display.print("  == "); display.print(holod.output()); display.print(" sec ==");
    // Display historical data chart: temperature by minutes
    int hist_min = holod.history().Min, hist_max = holod.history().Max;
	int	hist_range = hist_max - hist_min + 1, hist_height = SSD1306_LCDHEIGHT / 2;
	int	hist_top = 30;//SSD1306_LCDHEIGHT - hist_height + max(0, hist_height - hist_range + 1);
    float hist_scale = min((float)hist_height / hist_range, 1.0f);
	int hist_now = hist_top + hist_scale * (hist_max - round(holod.input())), hist_bottom = hist_top + hist_scale * hist_range;
	for (int x = SSD1306_LCDWIDTH - 1; x >= 40; x -= 2) { display.drawPixel(x, hist_now, WHITE); if (x % 6 == 0) { display.drawPixel(x, hist_top, WHITE); display.drawPixel(x, hist_bottom, WHITE); }}
    for (int x = SSD1306_LCDWIDTH - 1, i = holod.history().Index; x >= 0 && i >= 0; --x, --i) display.drawPixel(x, hist_top + hist_scale * (hist_max - holod.history().Data[i]), WHITE);
    int h = hist_bottom - hist_top; if (h <= 8) { hist_top -= 4 - h / 2; hist_bottom += 4 - (h - h / 2); }
    display.setCursor(0, hist_top - 3); display.print("hi:"); if (hist_max > 0) display.print("+"); display.print(hist_max);
    display.setCursor(0, hist_bottom - 3); display.print("lo:"); if (hist_min > 0) display.print("+"); display.print(hist_min);
    display.display();
#endif
}

