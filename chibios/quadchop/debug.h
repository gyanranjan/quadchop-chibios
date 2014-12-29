#ifndef _debug_h
#define _debug_h


#define DEBUG_LEVEL  DEBUG_INFO
#define DEBUG_INFO   0
#define DEBUG_WARN   1
#define DEBUG_ERR    2


uint32_t quad_debug(int level, const char *format, ...);
void quad_debug_init(void) ;
#endif