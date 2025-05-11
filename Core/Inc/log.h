#pragma once

#include "stdio.h"

#define log_debug(...)
#define log_error(...) printf(__VA_ARGS__)
#define log_fatal_and_die(...) { printf(__VA_ARGS__); while (1) {} }