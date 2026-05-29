// С логами ошибок
#include "main.h"   
#include "u8g2.h"   
#include <stdio.h>  

#define MAP_RPM_SIZE  8   
#define MAP_LOAD_SIZE 8   
#define SENSOR_MIN    5   
#define SENSOR_MAX    95  
#define REV_LIMIT     5500 
#define TEMP_TABLE_SIZE 6  

// --- МАСКИ ОШИБОК ДЛЯ НАШЕГО ЖУРНАЛА ОБД (БИТОВЫЕ МАСКИ) ---
#define ERR_NONE         0x00  // Ошибок нет — мотор поет!
#define ERR_DAD_BROKEN   0x01  // Бит 0: Отказал ДАД Волги (0000 0001)
#define ERR_TPS_BROKEN   0x02  // Бит 1: Отказал ДПДЗ ВАЗ   (0000 0010)
#define ERR_INJECTOR_1   0x04  // Бит 2: Сгорела Форсунка 1  (0000 0100)

// Адрес сектора Flash-памяти STM32 для сохранения лога (примерный адрес)
#define FLASH_LOG_ADDRESS 0x080E0000 

// Все наши прошлые карты зажигания, VE и переменные остаются на месте...
extern ADC_HandleTypeDef hadc1;  extern TIM_HandleTypeDef htim1;  extern I2C_HandleTypeDef hiwdg;  extern I2C_HandleTypeDef hi2c1;  
const int RPM_AXIS[MAP_RPM_SIZE]   = {800, 1200, 1600, 2200, 3000, 4000, 5000, 6000}; 
const int LOAD_AXIS[MAP_LOAD_SIZE] = {20,  30,  45,  60,  70,  80,  90,  100};  
uint16_t adc_raw_buffer[6]; u8g2_t u8g2;                  
int rpm = 800; int current_cylinder_pair = 14; int p_manifold = 40; int tps = 0; int temperature = 80; float battery_voltage = 14.0f;
float ve_coefficient = 1.0f; float uoz = 10.0f; float total_fuel = 0.0f; float o2_trim = 1.00f;

// --- ГЛОБАЛЬНЫЙ РЕГИСТР ТЕКУЩИХ ОШИБОК ---
uint8_t error_registry = ERR_NONE; 

// Функция записи кода ошибки намертво во Flash-память STM32
void write_error_to_flash(uint8_t error_code) {
    HAL_FLASH_Unlock(); // Снимаем аппаратный замок со встроенной флешки
    // Стираем сектор перед записью (специфика Flash-памяти)
    FLASH_EraseInitTypeDef erase_struct;
    erase_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_struct.Sector = FLASH_SECTOR_11; // Берем самый последний, безопасный сектор
    erase_struct.NbSectors = 1;
    uint32_t sector_error;
    HAL_FLASHEx_Erase(&erase_struct, &sector_error);

    // Намертво записываем байт ошибки по нашему адресу
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_LOG_ADDRESS, error_code);
    HAL_FLASH_Lock(); // Возвращаем замок безопасности обратно
}

// Заглушки функций тарировки для компиляции
int convert_adc_to_kpa(int adc_value) { return 40; }
int convert_adc_to_tps(int adc_value) { return 20; }
int convert_adc_to_temp(int adc_value) { return 80; }
float get_interpolated_value(const float map[8][8], int r, int l) { return 0.85f; }

int main(void)
{
  HAL_Init(); SystemClock_Config(); MX_GPIO_Init(); MX_DMA_Init(); MX_ADC1_Init(); MX_TIM1_Init(); MX_I2C1_Init();  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buffer, 6);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_cb_hw_i2c, &hi2c1);
  u8g2_InitDisplay(&u8g2); u8g2_SetPowerSave(&u8g2, 0); 

  char str_rpm[20]; char str_fuel[20]; char str_uoz[20];
  int main_injector_fault = 0; // Имитируем сгорание форсунки (0-ок, 1-авария)

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    uint16_t adc_dad  = adc_raw_buffer[0]; 
    uint16_t adc_tps  = adc_raw_buffer[1]; 
    
    int tps_ok = (adc_tps > 200 && adc_tps < 4000);
    int dad_ok = (adc_dad > 300 && adc_dad < 3900);

    // --- СИСТЕМА САМОДИАГНОСТИКИ ОБД ---
    uint8_t old_errors = error_registry; // Запоминаем прошлый статус
    error_registry = ERR_NONE;           // Сбрасываем перед проверкой

    // Проверяем ДАД
    if (!dad_ok) {
        error_registry |= ERR_DAD_BROKEN; // Включаем нужный бит ошибки через побитовое ИЛИ
        p_manifold = 60; // Аварийное давление
    } else { p_manifold = convert_adc_to_kpa(adc_dad); }

    // Проверяем ДПДЗ
    if (!tps_ok) {
        error_registry |= ERR_TPS_BROKEN; // Включаем бит поломки заслонки
        tps = 30; // Аварийная заслонка
    } else { tps = convert_adc_to_tps(adc_tps); }

    // Проверяем форсунку
    if (main_injector_fault == 1) {
        error_registry |= ERR_INJECTOR_1;
    }

    // ЛОГ ПАМЯТИ: Если появилась НОВАЯ ошибка, которой не было на прошлом круге — выжигаем её во Flash!
    if (error_registry != old_errors && error_registry != ERR_NONE) {
        write_error_to_flash(error_registry); 
    }

    // --- МАТЕМАТИКА И ВЫДАЧА ИМПУЛЬСОВ (Таймеры и зажигание) ---
    // (Тут крутится наш прошлый отлаженный Си-код расчета total_fuel и uoz...)
    total_fuel = 4.20f; uoz = 22.0f; // Временные константы для примера

    // --- УМНЫЙ ВЫВОД НА OLED-ЭКРАН С УЧЕТОМ ОБД ---
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);

    if (error_registry == ERR_NONE) {
        // Сценарий А: Ошибок нет. Выводим красивую рабочую приборку катера
        sprintf(str_rpm,  "RPM: %d", rpm);           
        sprintf(str_fuel, "INJ: %.2f ms", total_fuel); 
        sprintf(str_uoz,  "ADV: %.1f deg", uoz);     

        u8g2_DrawStr(&u8g2, 5, 15, "- SYSTEM OK -"); 
        u8g2_DrawStr(&u8g2, 5, 32, str_rpm);          
        u8g2_DrawStr(&u8g2, 5, 48, str_fuel);         
        u8g2_DrawStr(&u8g2, 5, 64, str_uoz);          
    } 
    else {
        // Сценарий Б: Авария! Экран переключается в тревожный режим диагностики
        u8g2_DrawStr(&u8g2, 5, 15, "⚠️ CHECK ENGINE"); 
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf); // Переключаемся на мелкий шрифт для логов
        
        int y_pos = 30;
        if (error_registry & ERR_DAD_BROKEN) { u8g2_DrawStr(&u8g2, 5, y_pos, "CODE 01: MAP SENSOR FAIL"); y_pos += 12; }
        if (error_registry & ERR_TPS_BROKEN) { u8g2_DrawStr(&u8g2, 5, y_pos, "CODE 02: TPS SENSOR FAIL"); y_pos += 12; }
        if (error_registry & ERR_INJECTOR_1) { u8g2_DrawStr(&u8g2, 5, y_pos, "CODE 04: INJECTOR 1 FAULT"); y_pos += 12; }
        
        u8g2_DrawStr(&u8g2, 5, 64, "MODE: LIMP HOME ACTIVE");
    }
    
    u8g2_SendBuffer(&u8g2); // Отправляем картинку на экран катера

    HAL_IWDG_Refresh(&hiwdg); 
    HAL_Delay(100); 
    /* USER CODE END WHILE */
  }
}
