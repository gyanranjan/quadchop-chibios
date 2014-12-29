#include "ch.h"
#include "debug.h"
#include "command.h"

#define MAX_Q  (5)
typedef struct circult_arr_s {
    int     start;
    int     end;
    int     valid_count;
    int     cmd[MAX_Q];
} circult_arr_t;

static circult_arr_t cq;
circult_arr_t *q = &cq;

#define DECLARE_CMD(a, b, c) b
char cmd_char[] = {
#include "command.inc"
};
#undef DECLARE_CMD

#define DECLARE_CMD(a, b, c) c
char *cmd_help[] = {
#include "command.inc"
};
#undef DECLARE_CMD

static void 
quad_init_q(void)
{
    uint32_t  i;
    q->valid_count  =  0;
    q->start        =  0;
    q->end          =  0;
    for(i=0; i<MAX_Q; i++) {
        q->cmd[i] = CMD_NONE;
    }        
    quad_debug(DEBUG_WARN , "init success\n\r");
    return;
}

static uint32_t 
quad_is_empty(void)
{
    return q->valid_count  ? 0:1;
}

static int32_t 
quad_add_cmd( quad_command_t cmd)
{
    if(q->valid_count >= MAX_Q) {
        return(-1);
    } else {
        q->valid_count++;
        q->cmd[q->end] = cmd;
        q->end = (q->end+1) % MAX_Q;
    }
    return 0;
}

int32_t 
quad_cmd_get(quad_command_t *cmd)
{
    if(quad_is_empty()) {
        *cmd = CMD_NONE;
        return(-1);
    } else {
        *cmd=q->cmd[q->start];
        q->start=(q->start + 1) % MAX_Q;
        q->valid_count--;
        return(0);
    }
}

void
quad_cmd_init(void)
{
    quad_init_q();
}

void 
quad_cmd_collector(void)
{
    char ch = quad_serial_fetch_cmd();
    quad_command_t cmd, test_cmd = CMD_NONE;
#if 1   
    /* is valid command */
    for (cmd=0; cmd < CMD_MAX; cmd++) {
        if (cmd_char[cmd] == ch) {
            if (cmd == CMD_TEST) {
                quad_cmd_get(&test_cmd);
                quad_debug(DEBUG_WARN, "Pop cmd %s\n\r", cmd_help[test_cmd]);
            } else if (cmd == CMD_NONE){
                quad_debug(DEBUG_WARN, "Ignoring cmd %s\n\r", cmd_help[cmd]);
            } else {
                if (!quad_add_cmd(cmd)) {
                    quad_debug(DEBUG_INFO, "valid adding to Q cmd %s\n\r", cmd_help[cmd]);
                } else {
                    quad_debug(DEBUG_ERR, "cmd Q full\n\r");
                }
            }
            break;
        }
    }
    if (cmd == CMD_MAX) {
        quad_debug(DEBUG_WARN, "Invalid cmd %c\n\r", ch);
    }
#endif
}
