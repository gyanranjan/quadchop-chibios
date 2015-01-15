#ifndef _debug_h
#define _debug_h
#include "ch.h"
#include <chprintf.h>

#define DEBUG_LEVEL  DEBUG_INFO
#define DEBUG_INFO   0
#define DEBUG_WARN   1
#define DEBUG_ERR    2
#define DEBUG_MAX    3

extern char *level_name[];
extern BaseSequentialStream *stream;
#define quad_debug(level, ...) { if ((level >= DEBUG_LEVEL) &&  (level < DEBUG_MAX) ) {\
                                 chprintf(stream,"%s:%d:",level_name[level] , chTimeNow());\
                                 chprintf(stream,__VA_ARGS__);}}
void quad_debug_init(void) ;
uint32_t quad_serial_fetch_cmd(void);
#endif
