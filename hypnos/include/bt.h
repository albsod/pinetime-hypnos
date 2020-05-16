#ifndef BT__H
#define BT__H

#include <stdbool.h>

void bt_init(void);
bool bt_is_initialized(void);
bool bt_mode(void);
void bt_ready(void);
void bt_adv_start(void);
void bt_adv_stop(void);
void bt_on(void);
void bt_off(void);
void bt_await_on(void);
void bt_await_off(void);

#endif /* BT__H */
