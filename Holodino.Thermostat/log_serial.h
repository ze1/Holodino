#ifndef LOG_SERIAL_H__
#define LOG_SERIAL_H__

bool N() { Serial.println(""); return true; }
template<typename T> bool S(T t, bool n = false) { Serial.print(t); if (n) Serial.println(""); return true; }
template<typename T> bool SH(T t, bool n = false) { if (sizeof(t) == 8 && t < 0x10) Serial.print("0"); Serial.print(t, HEX); if (n) Serial.println(""); return true; }

#endif // LOG_SERIAL_H__
