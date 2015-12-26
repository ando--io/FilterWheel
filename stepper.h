
#ifndef STEPPER_H
#define STEPPER_H

#define CW  1
#define CCW -1

void stepper_init(void);
void stepper_task(void);
void stepper_enable_output(void);
void stepper_disable_output(void);
void stepper_move(int direction) ;
int stepper_current_position(void);

#endif /* STEPPER_H */
