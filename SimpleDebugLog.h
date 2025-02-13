#ifndef SIMPLE_DEBUG_LOG
  #define SIMPLE_DEBUG_LOG

#include <Arduino.h>

//#define DEBUGLOG_DISABLE_LOG // uncomment this line to disable all debug logs

#if not defined(DEBUGLOG_DISABLE_LOG)

  // You can configure output Serial here
  #define PRINT(x) Serial.print(x)
  #define PRINT_LN(x) Serial.println(x)

  // You can configure output style here
  #define LOG_ERROR_HANDLER1(x) {PRINT("[ERROR] "); PRINT_LN(x);}
  #define LOG_ERROR_HANDLER2(x, y) {PRINT("[ERROR] "); PRINT(x); PRINT_LN(y);}
  #define LOG_WARN_HANDLER1(x) {PRINT("[WARNING] "); PRINT_LN(x);}
  #define LOG_WARN_HANDLER2(x, y) {PRINT("[WARNING] "); PRINT(x); PRINT_LN(y);}
  #define LOG_INFO_HANDLER1(x) {PRINT("[INFO] "); PRINT_LN(x);}
  #define LOG_INFO_HANDLER2(x, y) {PRINT("[INFO] "); PRINT(x); PRINT_LN(y);}
  #define LOG_DEBUG_HANDLER1(x) {PRINT("[DEBUG] "); PRINT_LN(x);}
  #define LOG_DEBUG_HANDLER2(x, y) {PRINT("[DEBUG] "); PRINT(x); PRINT_LN(y);}
  #define LOG_TRACE_HANDLER1(x) {PRINT("[TRACE] "); PRINT_LN(x);}
  #define LOG_TRACE_HANDLER2(x, y) {PRINT("[TRACE] "); PRINT(x); PRINT_LN(y);}

  // Helper macro to count the number of arguments
  #define GET_MACRO(_1, _2, NAME, ...) NAME

  // Define the macros according to the default log level
  #if defined(DEBUGLOG_DEFAULT_LOG_LEVEL_ERROR)
    #define LOG_ERROR(...) GET_MACRO(__VA_ARGS__, LOG_ERROR_HANDLER2, LOG_ERROR_HANDLER1)(__VA_ARGS__)
    #define LOG_WARN(...)
    #define LOG_INFO(...)
    #define LOG_DEBUG(...)
    #define LOG_TRACE(...)
  #elif defined(DEBUGLOG_DEFAULT_LOG_LEVEL_WARN)
    #define LOG_ERROR(...) GET_MACRO(__VA_ARGS__, LOG_ERROR_HANDLER2, LOG_ERROR_HANDLER1)(__VA_ARGS__)
    #define LOG_WARN(...) GET_MACRO(__VA_ARGS__, LOG_WARN_HANDLER2, LOG_WARN_HANDLER1)(__VA_ARGS__)
    #define LOG_INFO(...)
    #define LOG_DEBUG(...)
    #define LOG_TRACE(...)
  #elif defined(DEBUGLOG_DEFAULT_LOG_LEVEL_INFO)
    #define LOG_ERROR(...) GET_MACRO(__VA_ARGS__, LOG_ERROR_HANDLER2, LOG_ERROR_HANDLER1)(__VA_ARGS__)
    #define LOG_WARN(...) GET_MACRO(__VA_ARGS__, LOG_WARN_HANDLER2, LOG_WARN_HANDLER1)(__VA_ARGS__)
    #define LOG_INFO(...) GET_MACRO(__VA_ARGS__, LOG_INFO_HANDLER2, LOG_INFO_HANDLER1)(__VA_ARGS__)
    #define LOG_DEBUG(...)
    #define LOG_TRACE(...)
  #elif defined(DEBUGLOG_DEFAULT_LOG_LEVEL_DEBUG)
    #define LOG_ERROR(...) GET_MACRO(__VA_ARGS__, LOG_ERROR_HANDLER2, LOG_ERROR_HANDLER1)(__VA_ARGS__)
    #define LOG_WARN(...) GET_MACRO(__VA_ARGS__, LOG_WARN_HANDLER2, LOG_WARN_HANDLER1)(__VA_ARGS__)
    #define LOG_INFO(...) GET_MACRO(__VA_ARGS__, LOG_INFO_HANDLER2, LOG_INFO_HANDLER1)(__VA_ARGS__)
    #define LOG_DEBUG(...) GET_MACRO(__VA_ARGS__, LOG_DEBUG_HANDLER2, LOG_DEBUG_HANDLER1)(__VA_ARGS__)
    #define LOG_TRACE(...)
  #elif defined(DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE)
    #define LOG_ERROR(...) GET_MACRO(__VA_ARGS__, LOG_ERROR_HANDLER2, LOG_ERROR_HANDLER1)(__VA_ARGS__)
    #define LOG_WARN(...) GET_MACRO(__VA_ARGS__, LOG_WARN_HANDLER2, LOG_WARN_HANDLER1)(__VA_ARGS__)
    #define LOG_INFO(...) GET_MACRO(__VA_ARGS__, LOG_INFO_HANDLER2, LOG_INFO_HANDLER1)(__VA_ARGS__)
    #define LOG_DEBUG(...) GET_MACRO(__VA_ARGS__, LOG_DEBUG_HANDLER2, LOG_DEBUG_HANDLER1)(__VA_ARGS__)
    #define LOG_TRACE(...) GET_MACRO(__VA_ARGS__, LOG_TRACE_HANDLER2, LOG_TRACE_HANDLER1)(__VA_ARGS__)
  #else
    #warning "Defaulting to a log level of: DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE"
    #define LOG_ERROR(...) GET_MACRO(__VA_ARGS__, LOG_ERROR_HANDLER2, LOG_ERROR_HANDLER1)(__VA_ARGS__)
    #define LOG_WARN(...) GET_MACRO(__VA_ARGS__, LOG_WARN_HANDLER2, LOG_WARN_HANDLER1)(__VA_ARGS__)
    #define LOG_INFO(...) GET_MACRO(__VA_ARGS__, LOG_INFO_HANDLER2, LOG_INFO_HANDLER1)(__VA_ARGS__)
    #define LOG_DEBUG(...) GET_MACRO(__VA_ARGS__, LOG_DEBUG_HANDLER2, LOG_DEBUG_HANDLER1)(__VA_ARGS__)
    #define LOG_TRACE(...) GET_MACRO(__VA_ARGS__, LOG_TRACE_HANDLER2, LOG_TRACE_HANDLER1)(__VA_ARGS__)
  #endif

#elif defined(DEBUGLOG_DISABLE_LOG)

  #define LOG_ERROR(...)
  #define LOG_WARN(...)
  #define LOG_INFO(...)
  #define LOG_DEBUG(...)
  #define LOG_TRACE(...)

#endif

#endif