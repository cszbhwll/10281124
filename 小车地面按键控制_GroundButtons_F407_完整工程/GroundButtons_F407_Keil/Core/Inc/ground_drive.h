#ifndef GROUND_DRIVE_H
#define GROUND_DRIVE_H
#include <stdbool.h>
void ground_drive_init(void);
void ground_drive_poll(void);
/* Writable ASCII line, without CR/LF. Called by poll after framing. */
void ground_drive_command(char *line);
bool ground_drive_running(void);
bool ground_drive_faulted(void);
bool ground_drive_enabled(void);
#endif
