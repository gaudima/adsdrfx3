//
// Created by gaudima on 12/4/18.
//

#ifndef ADSDRFX3_DEBUG_H
#define ADSDRFX3_DEBUG_H

#include <cyu3system.h>

inline int lastslashpos(const char *msg) {
    int i = 0;
    for(i = strlen(msg) - 1; i >= 0 ; i--) {
        if(msg[i] == '/') {
            return i;
        }
    }
    return -1;
}

#define CyU3PDebugPrintL(priority, fmt, ...) \
    CyU3PDebugPrint(priority, (char*)("%s:%d: " fmt), __FILE__ + lastslashpos(__FILE__) + 1, __LINE__, ##__VA_ARGS__)

#endif //ADSDRFX3_DEBUG_H
