#include <SerialPort.h>
#include <Arduino.h>
#include <PID_v1.h>

//#include <Sodaq_DS3231.h>

#define PRODUCT       "HOLODINO"

//#define HOLOD
#ifdef HOLOD
#define TEMP_TARGET         -12

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

// OUTPUT: MOTOR RELAY SIGNAL PIN#
// ===================================
//  + Connect IN  to digital pin
//  + Connect VCC to power 5V DC
//  + Connect GND to common ground
#define OUTPUT_PIN            4

// DEBUG: LED PWM PIN
// ===================================
#define DEBUG_PIN             3

#endif

// DISPLAY: INTERNAL INFO
// ==================================

#define OLED_DISPLAY
#ifdef OLED_DISPLAY
	#define DISPLAY_WIFI
	//#define DISPLAY_HOLOD
	#include <Adafruit_GFX.h>
	#include <Adafruit_SSD1306.h>
	#define OLED_MOSI             9 // D1
	#define OLED_CLK             10 // D0
	#define OLED_DC              11 // DC
	#define OLED_CS              12 //  -
	#define OLED_RESET           13 // RST
	Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
	//Adafruit_SSD1306 display(13);
#endif

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

// ************** HOLODINO WIFI ************* //
#define WIFI_CLIENT
//#define WIFI_SERVER
#if defined(WIFI_CLIENT) || defined(WIFI_SERVER)

//#define WIFI_ACCESSPOINT
#define WIFI_STATION

SerialPort<0, 63, 63> NewSerial;
#define esp   NewSerial        // use Serial1 to talk to esp8266

#define SSID  "O"    // WiFi SSID
#define PASS  "9164035980"  // WiFi password
#define HOST  "10.1.1.10"   // HTTP host
#define PORT  80          // HTTP port

#define WIFI_RXRES_OK      1
#define WIFI_RXRES_ERROR   2
#define WIFI_RXRES_ENDSTR  4
#define WIFI_RXRES_LENGTH  8
#define WIFI_RXRES_BUFFER  16
#define WIFI_RXRES_TIMEOUT 32

class Wifi {
public:

	Wifi() :
		rxbuf(), rxendstr(), rxendlen(), rxlen(), rxidx(), rxres(0xff), rxtim(), rxfun(), state_(), rxupd() {

	}

	template<typename T>void print(T c) {
		esp.print(c);
#ifdef DISPLAY_WIFI
		display.print(c);
		display.display();
#endif
	}

	template<typename T>void print(const T* c) {
		esp.print(c);
#ifdef DISPLAY_WIFI
		display.print(c);
		display.display();
#endif
	}

	template<typename T>void println(T c) {
		esp.println(c);
#ifdef DISPLAY_WIFI
		display.print(c);
		display.print(">");
		display.display();
#endif
	}

	template<typename T>void println(const T* c) {
		esp.println(c);
#ifdef DISPLAY_WIFI
		display.print(c);
		display.print(">");
		display.display();
#endif
	}

	typedef void (rxfunc(Wifi&));

	void wait_for(const char *endstr, uint16_t len, uint32_t timeout, rxfunc fun) {

		rxres = 0;
		rxidx = 0;
		rxfun = fun;
		rxlen = len;
		rxtim = timeout ? millis() + timeout : 0;
		rxendlen = endstr ? strlen(endstr) : 0;
		if (rxendlen) strncpy(rxendstr, endstr, rxendlen);
	}

	void loop(rxfunc fun) {

		if (state_ & 1) {

			if (!rxres) {

				do {

					while (esp.available()) {

						char chr = esp.read();
						rxbuf[rxidx++] = esp.read();
	#ifdef DISPLAY_WIFI
//						display.print(chr >= 0x20 && chr < 0x80 ? chr : '.');
//						display.display();
	#endif
					}

					rxres = 
						(rxtim && millis() >= rxtim ? WIFI_RXRES_TIMEOUT : 0) |
						(rxlen && rxidx >= rxlen ? WIFI_RXRES_LENGTH : 0) |
						(rxidx >= sizeof(rxbuf) ? WIFI_RXRES_BUFFER : 0);

					if (rxendlen) {

						rxresidx = 0;
						for (char *str = rxendstr, *end = str + rxendlen, *sep; str < end; str = ++sep, ++rxresidx) {

							sep = strchr(str, '|');
							if (!sep) sep = end;
							if (!memcmp(rxbuf + rxidx - (sep - str), str, sep - str)) {

								rxres |= WIFI_RXRES_ENDSTR;
								break;
							}
						}
					}
#ifdef DISPLAY_WIFI
					if (rxupd < millis()) {

						rxupd = millis() + 1000;
						uint8_t	x = display.getCursorY(), y = display.getCursorY();
						display.setTextColor(0, 1);
						display.setCursor(0, display.height() - 16);
						display.print("m"); display.print(millis());
						display.print("s"); display.print(state_);
						display.print("r"); display.print(rxres);
						display.print("i");	display.print(rxidx);
						display.print("l"); display.print(rxlen);
						display.print("t"); display.print(rxtim);
						display.print("e"); display.print(rxendstr);
						display.setTextColor(1, 0);
						display.setCursor(0, 20);
						rxbuf[rxidx] = 0x00;8888888888888888888888
						display.print(rxbuf);
						display.display();
					}
#endif
				}
				while (!rxres);// && esp.available());

				if (rxres) {

	#ifdef DISPLAY_WIFI
					display.print(" =");
					display.print(rxres & WIFI_RXRES_TIMEOUT ? "T" : ""); 
					display.print(rxres & WIFI_RXRES_BUFFER ? "B" : "");
					display.print(rxres & WIFI_RXRES_LENGTH ? "L" : "");
					if (rxres & WIFI_RXRES_ENDSTR) { display.print("E"); display.print(rxresidx); }
					display.println("=");
					display.display();
	#endif
					rxfun(*this);
	#ifdef DISPLAY_WIFI
					if (display.getCursorY() >= display.height() - 16) {

						display.clearDisplay();
						display.setCursor(0, 0);
					}
	#endif
				}
			}
		}
		if (state_ == 0) init();
		if (state_ == 8) connect(HOST, PORT);
		if (state_ == 64) send(fun);
	}

	bool rxresult(uint8_t resmask, uint8_t resindex = 0) const {

		return (rxres & resmask) && (!(resmask & WIFI_RXRES_ENDSTR) || rxresidx == resindex);
	}

	void setup() {

		esp.begin(9600);
	}

	void init() {

		state(1);
		wait_for("ready", 0, 5000, [](Wifi &wifi) {

			wifi.println("AT");
			wifi.wait_for("OK\r\n|ERROR\r\n", 0, 5000, [](Wifi &wifi) {

				if (wifi.rxresult(WIFI_RXRES_ENDSTR, 0)) {

					wifi.println("AT+CWMODE=1");
					wifi.wait_for("OK\r\n|ERROR\r\n", 0, 5000, [](Wifi& wifi) {

						if (wifi.rxresult(WIFI_RXRES_ENDSTR, 0)) {

							wifi.println("AT+RST");
							wifi.wait_for("ready", 0, 7000, [](Wifi& wifi) {

								if (wifi.rxresult(WIFI_RXRES_TIMEOUT)) {

									wifi.print("AT+CWJAP=\""); wifi.print(SSID); wifi.print("\",\""); wifi.print(PASS); wifi.println("\""); // connect to an AP with SSID and password
									wifi.wait_for("OK\r\n|ERROR\r\n", 0, 10000, [](Wifi& wifi) {

										if (wifi.rxresult(WIFI_RXRES_ENDSTR, 0))
											wifi.state(8);
										else
											wifi.state(0);
									});
								}
								else
									wifi.state(0);
							});
						}
						else
							wifi.state(0);
					});
				}
				else
					wifi.state(0);
			});
		});
		/*
		#ifdef WIFI_STATION
		wifi.println("AT+CWMODE=1"); // 1 = Station, 2 = AP, 3 = Both
		#endif

		#ifdef WIFI_ACCESSPOINT
		wifi.println("AT+CWMODE=2"); // 1 = Station, 2 = AP, 3 = Both
		#endif
		wifi.wait_for_ok();
		wifi.print("AT+RST\r\n"); // reset WiFi module
		wifi.wait_for_ok(10000);

		#ifdef WIFI_STATION
		wifi.print("AT+CWJAP=\""); wifi.print(SSID); wifi.print("\",\""); wifi.print(PASS); wifi.println("\""); // connect to an AP with SSID and password
		#endif

		#ifdef WIFI_ACCESSPOINT
		wifi.print("AT+CWSAP=\""); wifi.print(SSID); wifi.print("\",\""); wifi.print(PASS); wifi.println("\",10,4"); // set SSID, password and channel
		#endif
		wifi.wait_for_ok(10000);

		#ifdef WIFI_SERVER
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
		wifi.wait_for();
		#endif
		*/
	}

	void connect(const char* host, uint16_t port = 80, bool tcp = true) {

		state(9);
		print("AT+CIPSTART=\""); print(tcp ? "TCP" : "UDP"); print("\",\""); print(host); print("\","); println(port);
		wait_for("OK\r\n|ALREADY CONNECT\r\n|ERROR\r\n", 0, 10000, [](Wifi &wifi) {
			
			if (wifi.rxresult(WIFI_RXRES_ENDSTR, 0) || wifi.rxresult(WIFI_RXRES_ENDSTR, 1))
				wifi.state(16);
			else
				wifi.state(8);
		});
	}

	void send(rxfunc fun) { // const char* txdata = "", uint16_t txsize = 0) {

		if (state_ != 16) return;
		state(17);

		fun(*this);

		uint16_t txlen = strlen(rxbuf);
		if (txlen) {

			print("AT+CIPSEND="); println(txlen);
			wait_for(">", 0, 10000, [](Wifi &wifi) {
				if (wifi.rxresult(WIFI_RXRES_ENDSTR)) {
					
					wifi.print(wifi.buffer());
					wifi.wait_for("OK\r\n|SEND_OK\r\n|ERROR\r\n", 0, 10000, [](Wifi &wifi) {
						
						if (wifi.rxresult(WIFI_RXRES_ENDSTR, 0) || wifi.rxresult(WIFI_RXRES_ENDSTR, 1))
							wifi.state(32);
						else
							wifi.state(16);
					});
				}
				else
					wifi.state(16);
			});
		}
		else
			state(16);
	}

	void state(uint8_t st) {

#ifdef DISPLAY_WIFI
		display.print("[");
		display.print(state_);
		display.print(">");
		display.print(st);
		display.print("]");
		display.display();
#endif
		state_ = st;
	}

	uint8_t state() {

		return state_;
	}

	void send(const char *data, uint16_t size = 0) {

		size = min(!size ? strnlen(data, sizeof(rxbuf)) : size, sizeof(rxbuf) - 1);
		if (size) strncpy(rxbuf, data, size);
		rxbuf[size] = 0x00;
	}

	const char *buffer() {

		return rxbuf;
	}

private:

	char rxbuf[64];
	char rxendstr[32];
	uint16_t rxendlen;
	uint16_t rxlen;
	uint16_t rxidx;
	uint8_t rxres;
	uint8_t rxresidx;
	uint32_t rxtim;
	rxfunc *rxfun;
	uint8_t state_;
	uint32_t rxupd;
}
wifi;

#endif

// ************** HOLODINO HISTORY ************* //
/*
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
*/

#ifdef HOLOD

// **************** HOLODINO CONTROLLER ***************** //

class Controller {
public:
	enum States { stInit, stStart, stOn, stOff };
	Controller(double speed, sec duration, sec cooldown, int output_pin) :
        timer_(speed), duration_(duration), cooldown_(cooldown), output_pin_(output_pin),
        input_(), output_(), target_(TEMP_TARGET), pid_(&input_, &output_, &target_, INITIAL_PID_P, INITIAL_PID_I, INITIAL_PID_D, REVERSE),
		offset_(), state_()
		//, history_(), history_timer_(speed)
	{
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
		/*if (!history_.Index || history_timer_.MilliSeconds() >= 60000) {
			history_timer_.Reset();
			history_.Add(input_);
		}*/
		//msec td = 1000 / timer_.speed(), te = TimeElapsed(), tt = (msec)(((float)te / (te + TimeLeft())) * td);
		//analogWrite(DEBUG_PIN, state_ == stStart ? 96 : (state_ == stOn ? 255 : 32)); wifi.loop(tt);
		//analogWrite(DEBUG_PIN, 0); wifi.loop(td - tt);
		Idle((msec)((double)TimeElapsed() / (TimeElapsed() + TimeLeft()) * 999));
		return state_;
    }
	void Idle(msec elapsed) {

		while (millis() < exec_ + 999 - elapsed) {
			delay(10);
#if defined(WIFI_CLIENT) || defined(WIFI_SERVER)
			wifi_loop(&wifi);
#endif
		}
		analogWrite(DEBUG_PIN, state_ == stStart ? 64 : (state_ == stOn ? 255 : 128));
		while (millis() < exec_ + 999) {
			delay(10);
#if defined(WIFI_CLIENT) || defined(WIFI_SERVER)
			wifi_loop(&wifi);
#endif
		}
		analogWrite(DEBUG_PIN, 0);
		exec_ = millis();
	}
	void wifi_loop(Wifi *wifi) {

		wifi->loop([](Wifi &wifi, uint8_t result, uint8_t index, char *data, uint16_t size) {

			strcpy(data, "GET / HTTP/1.0\r\nHost: ze1.org\r\n\r\n");
		});
	}
	States state() { return state_; }
	static const char* StateStr(States s) { return s == stInit ? "INIT" : s == stStart ? "DELAY" : s == stOn ? "*ON*" : s == stOff ? "OFF" : "ERROR"; }
	double input() { return input_; }
	double output() { return output_; }
	double target() { return target_; }
	//const History& history() { return history_; }
private:
    Timer timer_;
	sec duration_;
    sec cooldown_;
	int output_pin_;
    double input_;
    double output_;
    double target_;
    PID pid_;
	msec exec_;
	msec offset_;
	States state_;
	//History history_;
	//Timer history_timer_;
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

#endif // HOLOD

void setup() {

#ifdef HOLOD
#ifndef TEST_MODE
	temperature.begin();
#endif
#endif

#ifdef OLED_DISPLAY	
	//display.begin(SSD1306_SWITCHCAPVCC);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
	// display.begin(SSD1306_EXTERNALVCC);
	display.clearDisplay();
	display.setTextSize(1);
	display.setCursor(0, 0);
	display.setTextColor(WHITE);
	display.println(PRODUCT);
	display.display();
#endif

#if defined(WIFI_CLIENT) || defined(WIFI_SERVER)
	wifi.setup();/*
	esp.begin(115200);
	wifi.println("AT");
	wifi.wait_for_ok();
	#ifdef WIFI_STATION
	wifi.println("AT+CWMODE=1"); // 1 = Station, 2 = AP, 3 = Both
	#endif
	#ifdef WIFI_ACCESSPOINT
	wifi.println("AT+CWMODE=2"); // 1 = Station, 2 = AP, 3 = Both
	#endif
	wifi.wait_for_ok();
	wifi.print("AT+RST\r\n"); // reset WiFi module
	wifi.wait_for_ok();
	delay(3000);
	#ifdef WIFI_STATION
	wifi.print("AT+CWJAP=\""); wifi.print(SSID); wifi.print("\",\""); wifi.print(PASS); wifi.println("\""); // connect to an AP with SSID and password
	#endif
	#ifdef WIFI_ACCESSPOINT
	wifi.print("AT+CWSAP=\""); wifi.print(SSID); wifi.print("\",\""); wifi.print(PASS); wifi.println("\",10,4"); // set SSID, password and channel
	#endif
	wifi.wait_for_ok(10000);
	*/
#endif

#ifdef WIFI_SERVER
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
	wifi.wait_for();
#endif

}

void loop() {

#ifdef HOLOD
	// EXECUTE PID CONTROLLER CALCULATIONS
    //holod.Execute();
#endif

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

#ifdef WIFI_CLIENT

	wifi.loop([](Wifi &wifi) {

		wifi.send("GET / HTTP/1.0\r\nHost: ze1.org\r\n\r\n");
	});

	//wifi.println("AT+CIPSTATUS");
	//wifi.wait_for_ok(100);
#endif

}

