/*
 * Pushbutton driver
 * - Initialises pushbutton gpio pins 
 * - Creates push button callback * 
 * 
*/

#ifndef S4589514_PB_H
#define S4589514_PB_H

void init_pb(void);

bool get_pb_state(void);

int get_node_family(void);


#endif