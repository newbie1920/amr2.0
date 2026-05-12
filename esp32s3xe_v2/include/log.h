/**
 * AMR 2.0 v2 — Logging Macros
 * Consistent log format with module tags + Telnet mirror
 */

#ifndef LOG_H
#define LOG_H

#include <Arduino.h>
#include <TelnetStream.h>

// Log levels
#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

// Dual-output helper: Serial + TelnetStream
#define _LOG_DUAL(prefix, tag, fmt, ...) do { \
    Serial.printf("[%s] " prefix fmt "\n", tag, ##__VA_ARGS__); \
    TelnetStream.printf("[%s] " prefix fmt "\n", tag, ##__VA_ARGS__); \
} while(0)

// Formatted log macros with module tag
#define LOG_E(tag, fmt, ...) do { if (LOG_LEVEL >= LOG_LEVEL_ERROR) _LOG_DUAL("ERROR: ", tag, fmt, ##__VA_ARGS__); } while(0)
#define LOG_W(tag, fmt, ...) do { if (LOG_LEVEL >= LOG_LEVEL_WARN)  _LOG_DUAL("WARN: ",  tag, fmt, ##__VA_ARGS__); } while(0)
#define LOG_I(tag, fmt, ...) do { if (LOG_LEVEL >= LOG_LEVEL_INFO)  _LOG_DUAL("",        tag, fmt, ##__VA_ARGS__); } while(0)
#define LOG_D(tag, fmt, ...) do { if (LOG_LEVEL >= LOG_LEVEL_DEBUG) _LOG_DUAL("DBG: ",   tag, fmt, ##__VA_ARGS__); } while(0)

#endif // LOG_H
