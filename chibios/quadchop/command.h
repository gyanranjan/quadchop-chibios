#ifndef COMMAND_H
#define COMMAND_H
#define DECLARE_CMD(a, b, c) a

typedef enum
{
#include "command.inc"
}quad_command_t;
#undef  DECLARE_CMD

int32_t quad_cmd_get(quad_command_t *cmd);
void quad_cmd_init(void);
void quad_cmd_collector(void);

#endif