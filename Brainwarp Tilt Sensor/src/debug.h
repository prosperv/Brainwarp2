#pragma once

#ifdef DEBUG
#define PRINT Serial.print
#define PRINTLN Serial.println
#else
#define PRINT(x)
#define PRINTLN(x)
#endif