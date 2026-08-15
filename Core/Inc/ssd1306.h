#ifndef SSD1306_H
#define SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define SSD1306_WIDTH        128U
#define SSD1306_HEIGHT       64U
#define SSD1306_I2C_ADDRESS  (0x3CU << 1)

typedef struct
{
  I2C_HandleTypeDef *i2c;
  uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8U];
  uint8_t available;
} SSD1306_Handle;

HAL_StatusTypeDef SSD1306_Init(SSD1306_Handle *display,
                               I2C_HandleTypeDef *i2c);
void SSD1306_Clear(SSD1306_Handle *display);
void SSD1306_WriteLine(SSD1306_Handle *display, uint8_t line,
                       const char *text);
HAL_StatusTypeDef SSD1306_Update(SSD1306_Handle *display);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */
