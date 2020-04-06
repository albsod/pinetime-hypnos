#ifndef CLOCK__H
#define CLOCK__H

#include <zephyr.h>
#include <time.h>

/* Stringify build time */                                     
#define _xstr(s) _str(s)
#define _str(s) #s  
#define TIME_OF_BUILD _xstr(CURRENT_TIME_OF_BUILD)

/* time_t str_to_time(const char *); */

/* void clock_init(); */

/* char *time_get_local(); */

void str_to_time(const char *str, struct tm *ti);

void clock_init();

char *time_get_local();

struct tm *time_get();


void print_time();
void increment_time(struct k_timer *tick);
//void increment_time(void);


#endif /* CLOCK__H */
