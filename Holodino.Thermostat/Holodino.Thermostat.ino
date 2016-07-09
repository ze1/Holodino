#include <Arduino.h>
#include <PID_v1.h>
#include "log_serial.h"
//#include "wifi_esp8266.h"
//#include <Sodaq_DS3231.h>

#define PRODUCT       "HOLODINO"

#define TEMP_TARGET        -16

#define INITIAL_PID_P       30
#define INITIAL_PID_I       60 
#define INITIAL_PID_D       15

#define START_DELAY         270
#define WINDOW_SIZE		   1200

typedef   signed long      sec;
typedef unsigned long     msec;

#define TEST_MODE
#ifdef TEST_MODE
	// TEST PARAMETERS
	#define TIME_SPEED     10.00
	#define TEMP_INITIAL +20.00
	#define TEMP_RATE_ON  -0.020
	#define TEMP_RATE_OFF  0.010
#else
	// TEMPERATURE SENSOR 1-WIRE PIN
	#define INPUT_PIN		  2
	#include <OneWire.h>
	#include <DallasTemperature.h>
	OneWire temperature_wire(INPUT_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
	DallasTemperature temperature(&temperature_wire); // Pass our oneWire reference to Dallas Temperature. 
#endif

// OUTPUT: MOTOR RELAY SIGNAL PIN#
// ===================================
//  + Connect IN  to digital pin
//  + Connect VCC to power 5V DC
//  + Connect GND to common ground
#define OUTPUT_PIN            4

// DEBUG: LED PWM PIN
// ===================================
#define YEL_LED_PIN           3
#define GRN_LED_PIN           5
#define RED_LED_PIN           6

// DISPLAY: INTERNAL INFO
// ==================================

//#define OLED_DISPLAY
#ifdef OLED_DISPLAY
	//#define DISPLAY_WIFI
	#define DISPLAY_HOLOD
	#include <Adafruit_GFX.h>
	#include <Adafruit_SSD1306.h>
	#define OLED_MOSI      9 // D1
	#define OLED_CLK      10 // D0
	#define OLED_DC       11 // DC
	#define OLED_CS       12 //  -
	#define OLED_RESET    13 // RST
	Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#endif

// ************** HOLODINO TIMER ************* //

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
	History() : Index(), Data(), Min(), Max() {
	}
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
        input_(), output_(), target_(TEMP_TARGET), pid_(&input_, &output_, &target_, INITIAL_PID_P, INITIAL_PID_I, INITIAL_PID_D, REVERSE),
		offset_(), state_(), history_(), temp_(), outp_(), exec_(), serialize_() {
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
		serialize_ = true;
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
		return max((signed long)(end - TimeElapsed()), 0);
    }
    States Execute() {

        if (state_ == stInit && Init()) State(stStart);
		if (state_ != stInit) {

            if (!Input(input_)) State(stInit);
            else {

				if (!temp_ || abs(temp_ - input_) >= 1) {

					temp_ = input_;
					serialize_ = true;
				}
				pid_.Compute();

				if (!outp_ || abs(outp_ - output_) >= 60) {

					outp_ = output_;
					serialize_ = true;
				}
				if (!TimeLeft())
					State(state_ == stStart ? stOn : (state_ == stOn ? stOff : (state_ == stOff ? stStart : stInit)));
			}
        }
		Output(state_ == stOn);
		
		if (!history_.Index || 
			history_timer_.MilliSeconds() >= 60000) {

			history_timer_.Reset();
			history_.Add(input_);
			serialize_ = true;
		}
		Idle();

		return state_;
	}

	void Idle() {

		if (serialize_) {

			serialize_ = false;
			Serialize();
		}

		uint8_t led_pin = state_ == stStart ? YEL_LED_PIN : (state_ == stOn ? GRN_LED_PIN : (state_ == stOff ? RED_LED_PIN : 0));
		msec exec = millis(), elapsed = !led_pin ? 0 : (msec)((double)TimeElapsed() / (TimeElapsed() + TimeLeft()) * 1000);
		exec -= (!exec_ || exec_ > exec) ? exec : exec_;

		if (!led_pin || led_pin == YEL_LED_PIN) analogWrite(YEL_LED_PIN, 0xff);
		if (!led_pin || led_pin == GRN_LED_PIN) analogWrite(GRN_LED_PIN, 0xff);
		if (!led_pin || led_pin == RED_LED_PIN) analogWrite(RED_LED_PIN, 0xff);
		delay(1000 - elapsed);

		analogWrite(YEL_LED_PIN, 0);
		analogWrite(GRN_LED_PIN, 0);
		analogWrite(RED_LED_PIN, 0);
		delay(!led_pin ? 1000 : (elapsed > exec ? elapsed - exec : 0));

		exec_ = millis();
    }

	static const char* StateStr(States s) {

		return s == stInit ? "INIT" : s == stStart ? "DELAY" : s == stOn ? "*ON*" : s == stOff ? "OFF" : "ERROR";
	}
	States state() { return state_; }

	double input() { return input_; }
	double output() { return output_; }
	double target() { return target_; }

	const History& history() { return history_; }

	void Serialize() {

		N();
		S("holod:{");
		S("\"state\":"); S(state_);
		S(",\"offset\":"); S(offset_);	
		S(",\"elapsed\":"); S(TimeElapsed() / 1000);
		S(",\"left\":"); S(TimeLeft() / 1000);
		S(",\"input\":"); S(input_);
		S(",\"output\":"); S(output_);
		S(",\"target\":"); S(target_);
		S(",\"pid_kp\":"); S(pid_.GetKp());
		S(",\"pid_ki\":"); S(pid_.GetKi());
		S(",\"pid_kd\":"); S(pid_.GetKd());
		S(",\"duration\":"); S(duration_);
		S(",\"cooldown\":"); S(cooldown_);
		S(",\"history\":[");
		for (uint16_t i = 0; i < history_.Index; ++i) {

			if (i) S(",");
			S(history_.Data[i]);
		}
		S("]}");
		N();
	}

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
	double temp_;
	double outp_;
	msec exec_;
	bool serialize_;
};

// ***************** HOLODINO MAIN ***************** //

#ifndef TEST_MODE

class ControllerRTC : public Controller {
public:
    ControllerRTC() :
        Controller(1.0, WINDOW_SIZE, START_DELAY, OUTPUT_PIN) {
    }
	virtual bool Init() {
		if (!Controller::Init()) return false;
		temperature.begin();
		return true; 
	}
    virtual bool Input(double &t) {
		temperature.requestTemperatures();
        t = temperature.getTempCByIndex(0);
        return t > -100.0;
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

	Serial.begin(115200);

#ifdef OLED_DISPLAY	
	display.begin(SSD1306_SWITCHCAPVCC);
	//display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
	//display.begin(SSD1306_EXTERNALVCC);
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

#ifdef DISPLAY_HOLOD
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
   /* int hist_min = holod.history().Min, hist_max = holod.history().Max;
	int	hist_range = hist_max - hist_min + 1, hist_height = SSD1306_LCDHEIGHT / 2;
	int	hist_top = 30;//SSD1306_LCDHEIGHT - hist_height + max(0, hist_height - hist_range + 1);
    float hist_scale = min((float)hist_height / hist_range, 1.0f);
	int hist_now = hist_top + hist_scale * (hist_max - round(holod.input())), hist_bottom = hist_top + hist_scale * hist_range;
	for (int x = SSD1306_LCDWIDTH - 1; x >= 40; x -= 2) { display.drawPixel(x, hist_now, WHITE); if (x % 6 == 0) { display.drawPixel(x, hist_top, WHITE); display.drawPixel(x, hist_bottom, WHITE); }}
    for (int x = SSD1306_LCDWIDTH - 1, i = holod.history().Index; x >= 0 && i >= 0; --x, --i) display.drawPixel(x, hist_top + hist_scale * (hist_max - holod.history().Data[i]), WHITE);
    int h = hist_bottom - hist_top; if (h <= 8) { hist_top -= 4 - h / 2; hist_bottom += 4 - (h - h / 2); }
    display.setCursor(0, hist_top - 3); display.print("hi:"); if (hist_max > 0) display.print("+"); display.print(hist_max);
    display.setCursor(0, hist_bottom - 3); display.print("lo:"); if (hist_min > 0) display.print("+"); display.print(hist_min);*/
    display.display();
#endif

}

