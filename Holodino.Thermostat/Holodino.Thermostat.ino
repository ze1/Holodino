#include <Arduino.h>
#include <PID_v1.h>
//#include <Sodaq_DS3231.h>

#define PRODUCT       "HOLODINO"

#define TEMP_TARGET       -12.0
#define INITIAL_PID_P        30
#define INITIAL_PID_I        60
#define INITIAL_PID_D       270

#define START_DELAY         270
#define WINDOW_SIZE		   1200

//#define TEST_MODE
#ifdef TEST_MODE

	// TEST PARAMETERS
	#define TIME_SPEED          1.0
	#define TEMP_INITIAL      +20.0
	#define TEMP_RATE_ON      -0.10
	#define TEMP_RATE_OFF      0.01
#else

	// TEMPERATURE SENSOR 1-WIRE PIN
	#define INPUT_PIN		      2

	#include <OneWire.h>
	#include <DallasTemperature.h>
	
	OneWire temperature_wire(INPUT_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
	DallasTemperature temperature(&temperature_wire); // Pass our oneWire reference to Dallas Temperature. 
#endif

// DISPLAY: INTERNAL INFO
// ==================================

//#define OLED_DISPLAY
#ifdef OLED_DISPLAY
	//#define DISPLAY_ESP
	#define DISPLAY_HOLOD
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
#define OUTPUT_PIN            4

#define DEBUG_PIN             3 // PWM

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
        input_(), output_(), target_(TEMP_TARGET), pid_(&input_, &output_, &target_, INITIAL_PID_P, INITIAL_PID_I, INITIAL_PID_D, REVERSE),
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
		return max((signed long)(end - TimeElapsed()), 0);
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
		analogWrite(DEBUG_PIN, state_ == stStart ? 96 : (state_ == stOn ? 255 : 32)); delay(tt);
		analogWrite(DEBUG_PIN, 0); delay(td - tt);
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
/*
#define esp   Serial        // use Serial1 to talk to esp8266

#define SSID  "HOLODINO"    // WiFi SSID
#define PASS  "9164035980"  // WiFi password
#define PORT  "80"          // HTTP port

class Wifi {
public:
	Wifi() {

	}
	template<typename T>void print(T c) {
		esp.print(c);
		display.print(c);
	}
	template<typename T>void print(const T* c) {
		esp.print(c);
		display.print(c);
	}
	template<typename T>void println(T c) {
		esp.println(c);
		display.println(c);
		display.display();
	}
	template<typename T>void println(const T* c) {
		esp.println(c);
		display.println(c);
		display.display();
	}
	// wait for at most timeout milliseconds or if OK\r\n is found
	byte wait_for(const char* response = "OK\r\n", unsigned long timeout = 3000) {
		unsigned short index = 0, length = strlen(response);
		char *buffer = new char(length);
		timeout += millis();
		bool result = false;
		do {
			delay(1);
			while (!result && esp.available()) {
				char c = esp.read();
				buffer[index++ % length] = c;
				result = index >= length;
				for (unsigned long r = length, b = index; result && r > 0; )
					result = buffer[--b % length] == response[--r];
				display.print(c);
			}
			display.display();
		}
		while (!result && millis() < timeout);
		delete buffer;
		display.display();
    display.clearDisplay();
    display.setCursor(0, 0);
		return result;
	}
private:
}
wifi;
*/
// ***************** HOLODINO MAIN ***************** //

#ifndef TEST_MODE

class ControllerRTC : public Controller {
public:
    ControllerRTC() :
        Controller(1.0, WINDOW_SIZE, START_DELAY, OUTPUT_PIN) {
    }
	virtual bool Init() {
		if (!Controller::Init()) return false;
		return true; 
	}
    virtual bool Input(double &t) {
		temperature.requestTemperatures();
        t = temperature.getTempCByIndex(0);
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

	temperature.begin();

#ifdef OLED_DISPLAY	
	display.begin(SSD1306_SWITCHCAPVCC);
 // display.begin(SSD1306_EXTERNALVCC);
	display.clearDisplay();
	display.setTextSize(1);
	display.setCursor(0, 0);
	display.setTextColor(WHITE);
	display.println(PRODUCT);
	display.display();
#endif
	/*
	// assume esp8266 operates at 115200 baud rate
	esp.begin(115200);
	// try empty AT command
	wifi.println("AT");
	wifi.wait_for();
	// set mode 2 (AP)
	wifi.println("AT+CWMODE=2");
	wifi.wait_for();
	// reset WiFi module
	wifi.print("AT+RST\r\n");
	wifi.wait_for();
	delay(3000);

	// setup AP
	wifi.print("AT+CWSAP=\""); wifi.print(SSID); wifi.print("\",\""); wifi.print(PASS); wifi.println("\",10,4");
	// this may take a while, so wait for 5 seconds
	wifi.wait_for();
  delay(10000);

	// print device IP address
	wifi.println("AT+CIFSR");
	wifi.wait_for();

	// TCP server timeout
	wifi.println("AT+CIPSTO=300");
	wifi.wait_for();

	// start server
	wifi.println("AT+CIPMUX=1");
	wifi.wait_for();

	// turn on TCP service
	wifi.print("AT+CIPSERVER=1,"); wifi.println(PORT);
	wifi.wait_for();*/
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

