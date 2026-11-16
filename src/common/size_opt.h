// AllQuad - build this translation unit for size (-Os) when AQ_SIZE_OPT is set (ESP32 builds).
// Included first by the files that are not on the flight-critical path (web server, CLI, MAVLink,
// MSP, OSD, drivers that run at <= 500 Hz ...). The fast loop, filters, estimators and controllers
// stay at -O2 (FAST_CODE in IRAM on the ESP32).
#pragma once
#if defined(AQ_SIZE_OPT) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("Os")
#endif
