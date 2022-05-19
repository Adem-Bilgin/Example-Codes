#include "stm32f4xx_hal.h"

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd= satır başı yapar, imleci istediğimiz yere
							   // götürür, ekranıda silebilir.

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd = Yazıyı bu komutla yazabiliyoruz.
