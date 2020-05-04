#ifndef BT__H
#define BT__H

#include <stdbool.h>

void bt_init(void);
void bt_ready(void);
void bt_adv_start(void);
void bt_adv_stop(void);
bool bt_mode(void);

#endif /* BT__H */
