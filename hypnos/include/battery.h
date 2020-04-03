
void battery_status_init();
uint8_t battery_get_percentage();
bool battery_get_charging_status();
uint32_t battery_raw_to_mv(s16_t raw);
uint32_t battery_mv_to_ppt(uint32_t mv);
