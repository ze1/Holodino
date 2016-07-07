// wifi_esp8266.h

#ifndef WIFI_ESP8266_H__
#define WIFI_ESP8266_H__

// ************** HOLODINO WIFI ************* //
//#define WIFI_CLIENT
//#define WIFI_SERVER
#if defined(WIFI_CLIENT) || defined(WIFI_SERVER)

//#define WIFI_ACCESSPOINT
#define WIFI_STATION

#define esp   Serial        // use Serial1 to talk to esp8266

#define SSID  "O"    // WiFi SSID
#define PASS  "9164035980"  // WiFi password
#define HOST  "10.1.1.10"   // HTTP host
#define PORT  80          // HTTP port

#define WIFI_RXRES_ENDSTR  1
#define WIFI_RXRES_LENGTH  2
#define WIFI_RXRES_BUFFER  4
#define WIFI_RXRES_TIMEOUT 8

class Wifi {
public:

	Wifi() :
		rxbuf(), rxendstr(), rxendlen(), rxlen(), rxidx(), rxres(0xff), rxtim(), rxfun(), state_() {

	}

	template<typename T>void print(T c) {
		esp.print(c);
#ifdef DISPLAY_WIFI
		display.print(c);
#endif
	}

	template<typename T>void print(const T* c) {
		esp.print(c);
#ifdef DISPLAY_WIFI
		display.print(c);
#endif
	}

	template<typename T>void println(T c) {
		esp.println(c);
#ifdef DISPLAY_WIFI
		display.print(c);
		display.display();
#endif
	}

	template<typename T>void println(const T* c) {
		esp.println(c);
#ifdef DISPLAY_WIFI
		display.print(c);
		display.display();
#endif
	}

	typedef void (rxfunc(Wifi&, uint8_t, char*, uint16_t));

	void wait_for_crlf(uint32_t timeout, rxfunc fun) {

		wait_for("\r\n", 0, timeout, fun);
	}

	void wait_for(const char *end, uint16_t len, uint32_t timeout, rxfunc fun) {

		rxres = 0;
		rxidx = 0;
		rxfun = fun;
		rxtim = millis() + timeout;
		rxlen = min(sizeof(rxbuf), len ? len : 0xffff);
		rxendlen = end ? strlen(end) : 0;
		if (rxendlen) strncpy(rxendstr, end, rxendlen);
	}

	void loop() {

		if (!rxres && esp.available()) {

			do {

				char chr = esp.read();
				rxbuf[rxidx++] = chr;

#ifdef DISPLAY_WIFI
				if (chr >= 0x20 && chr < 0x80) display.print(chr);
#endif

				rxres = 
					(millis() < rxtim ? 0 : WIFI_RXRES_TIMEOUT) |
					(rxidx < sizeof(rxbuf) ? 0 : WIFI_RXRES_BUFFER) |
					(rxidx < rxlen ? 0 : WIFI_RXRES_LENGTH) |
					(rxidx < rxendlen || memcmp(rxbuf + rxidx - rxendlen, rxendstr, rxendlen) ? 0 : WIFI_RXRES_ENDSTR);

				if (rxres) {

#ifdef DISPLAY_WIFI
					display.print(" ="); 
					display.print(rxres & WIFI_RXRES_ENDSTR ? "E" : ""); display.print(rxres & WIFI_RXRES_LENGTH ? "L" : "");
					display.print(rxres & WIFI_RXRES_BUFFER ? "B" : ""); display.print(rxres & WIFI_RXRES_TIMEOUT ? "T" : ""); 
					display.println("=");
					display.display();
					//display.clearDisplay();
					//display.setCursor(0, 0);
#endif

					rxfun(*this, rxres, rxbuf, rxidx);
					return;
				}
			}
			while (esp.available());
			display.display();
		}
	}

	void setup() {

		esp.begin(115200);
	}

	void init() {

		state(1);
		println("ATE1");
		wait_for_crlf(5000, [](Wifi &wifi, uint8_t result, char *data, uint16_t size) {
			if ((result & WIFI_RXRES_ENDSTR) && !strncmp(data, "OK", size - 4)) {

				wifi.println("AT+CWMODE=1");
				wifi.wait_for_crlf(5000, [](Wifi& wifi, uint8_t result, char *data, uint16_t size) {
					if ((result & WIFI_RXRES_ENDSTR) && !strncmp(data, "OK", size - 4)) {

						wifi.println("AT+RST");
						wifi.wait_for_crlf(5000, [](Wifi& wifi, uint8_t result, char *data, uint16_t size) {
							if ((result & WIFI_RXRES_ENDSTR) && !strncmp(data, "OK", size - 4)) {

								wifi.print("AT+CWJAP=\""); wifi.print(SSID); wifi.print("\",\""); wifi.print(PASS); wifi.println("\""); // connect to an AP with SSID and password
								wifi.wait_for_crlf(15000, [](Wifi& wifi, uint8_t result, char *data, uint16_t size) {

									if ((result & WIFI_RXRES_ENDSTR) && !strncmp(data, "OK", size - 4))
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
		wait_for_crlf(10000, [](Wifi &wifi, uint8_t result, char *data, uint16_t size) {
			if ((result & WIFI_RXRES_ENDSTR) && (!strncmp(data, "OK", size - 4) || !strncmp(data, "ALREADY CONNECT", 17)))
				wifi.state(16);
			else
				wifi.state(8);
		});
	}

	void send() { // const char* txdata = "", uint16_t txsize = 0) {

				  //uint16_t txsize = 0;
				  //if (!txdata || !*txdata) return;
				  //if (!txsize) txsize = strlen(txdata);

		state(17);
		print("AT+CIPSEND=33"); println("");//txsize);
		wait_for(">", 1, 10000, [](Wifi &wifi, uint8_t result, char *data, uint16_t size) {
			if (result & WIFI_RXRES_ENDSTR) {

				wifi.print("GET / HTTP/1.0\r\nHost: ze1.org\r\n\r\n");
				wifi.wait_for_crlf(10000, [](Wifi &wifi, uint8_t result, char *data, uint16_t size) {

					if ((result & WIFI_RXRES_ENDSTR) && (!strncmp(data, "OK", size - 4) || !strncmp(data, "SEND OK", 9)))
						wifi.state(32);
					else
						wifi.state(16);
				});
			}
			else
				wifi.state(17);
		});
	}

	void state(uint8_t st) {

		state_ = st;
	}

	uint8_t state() {

		return state_;
	}

private:

	char rxbuf[32];
	char rxendstr[16];
	uint16_t rxendlen;
	uint16_t rxlen;
	uint16_t rxidx;
	uint8_t rxres;
	uint32_t rxtim;
	rxfunc *rxfun;
	uint8_t state_;
}
wifi;

#endif

#endif

