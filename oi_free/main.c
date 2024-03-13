

/**
 * main.c
 * This program simply frees the memory allocated by the open_interface.
 */

#include "lib/open_interface.h"
#include "lib/lcd.h"

int main(void)
{
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    lcd_init();
    lcd_clear();
    oi_free(sensor_data);
}
