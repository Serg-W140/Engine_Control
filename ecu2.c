// С датчиком детонации
#include "main.h"   
#include "u8g2.h"   
#include <stdio.h>  

#define MAP_RPM_SIZE  8   
#define MAP_LOAD_SIZE 8   
#define SENSOR_MIN    5   
#define SENSOR_MAX    95  
#define REV_LIMIT     5500 
#define TEMP_TABLE_SIZE 6  

// --- НАСТРОЙКИ ДАТЧИКА ДЕТОНАЦИИ ---
#define KNOCK_THRESHOLD 2500 // Порог АЦП, выше которого шум считается звоном мотора
#define KNOCK_RETARD    4.0f // На сколько градусов мгновенно уменьшить УОЗ при звоне
#define KNOCK_RECOVERY  0.1f // Как быстро (градусов за шаг) возвращать угол обратно

extern ADC_HandleTypeDef hadc1;  
extern TIM_HandleTypeDef htim1;  
extern I2C_HandleTypeDef hiwdg;  
extern I2C_HandleTypeDef hi2c1;  

const int RPM_AXIS[MAP_RPM_SIZE]   = {800, 1200, 1600, 2200, 3000, 4000, 5000, 6000}; 
const int LOAD_AXIS[MAP_LOAD_SIZE] = {20,  30,  45,  60,  70,  80,  90,  100};  

const int ADC_TEMP_AXIS[TEMP_TABLE_SIZE]   = {3800, 3100, 2048, 1000, 450,  200}; 
const int CAL_TEMP_VALUES[TEMP_TABLE_SIZE] = {-10,  0,    20,   50,   80,   100}; 

const float VE_MAP[MAP_RPM_SIZE][MAP_LOAD_SIZE] = {
    { 0.45f, 0.52f, 0.60f, 0.65f, 0.68f, 0.70f, 0.72f, 0.75f }, 
    { 0.48f, 0.55f, 0.63f, 0.68f, 0.71f, 0.73f, 0.75f, 0.78f }, 
    { 0.50f, 0.58f, 0.67f, 0.72f, 0.75f, 0.78f, 0.80f, 0.83f }, 
    { 0.53f, 0.62f, 0.71f, 0.78f, 0.82f, 0.84f, 0.86f, 0.88f }, 
    { 0.56f, 0.65f, 0.75f, 0.83f, 0.87f, 0.90f, 0.93f, 0.95f }, 
    { 0.55f, 0.63f, 0.74f, 0.81f, 0.85f, 0.88f, 0.91f, 0.92f }, 
    { 0.51f, 0.59f, 0.69f, 0.76f, 0.80f, 0.83f, 0.85f, 0.86f }, 
    { 0.46f, 0.54f, 0.63f, 0.70f, 0.74f, 0.76f, 0.78f, 0.79f }  
};

const float IGN_MAP[MAP_RPM_SIZE][MAP_LOAD_SIZE] = {
    { 22.0f, 18.0f, 15.0f, 12.0f, 10.0f, 9.0f,  8.0f,  7.0f  }, 
    { 25.0f, 22.0f, 19.0f, 16.0f, 14.0f, 12.0f, 11.0f, 10.0f }, 
    { 28.0f, 26.0f, 23.0f, 20.0f, 18.0f, 16.0f, 14.0f, 13.0f }, 
    { 33.0f, 31.0f, 28.0f, 25.0f, 23.0f, 21.0f, 19.0f, 17.0f }, 
    { 38.0f, 36.0f, 34.0f, 31.0f, 29.0f, 27.0f, 25.0f, 23.0f }, 
    { 40.0f, 39.0f, 36.0f, 34.0f, 32.0f, 30.0f, 28.0f, 26.0f }, 
    { 42.0f, 41.0f, 38.0f, 36.0f, 34.0f, 32.0f, 31.0f, 29.0f }, 
    { 44.0f, 42.0f, 40.0f, 38.0f, 36.0f, 35.0f, 33.0f, 31.0f }  
};

// --- ОПЕРАТИВНЫЕ ПЕРЕМЕННЫЕ ---
// Расширяем буфер DMA до 6 элементов, так как добавился ДД
uint16_t adc_raw_buffer[6]; // 0-ДАД, 1-ДПДЗ, 2-ДТОЖ, 3-Датчик Кислорода, 4-ДД (Детонация), 5-АКБ
u8g2_t u8g2;                  

int rpm = 800;                
int current_cylinder_pair = 14; 
int p_manifold = 40;          
int tps = 0;                  
int temperature = 80;         
float battery_voltage = 14.0f;

// Переменные для контроля детонации
float knock_correction = 0.0f; // Текущий динамический отскок УОЗ (всегда <= 0)

float ve_coefficient = 1.0f;  
float uoz = 10.0f;            
float total_fuel = 0.0f;      
float o2_trim = 1.00f;        

// Функции тарировки и интерполяции остаются без изменений...
int convert_adc_to_kpa(int adc_value) { int adc_min = 400; int adc_max = 3600; if (adc_value <= adc_min) return 20; if (adc_value >= adc_max) return 100; return 20 + (100 - 20) * (adc_value - adc_min) / (adc_max - adc_min); }
int convert_adc_to_tps(int adc_value) { int adc_min = 200; int adc_max = 3900; if (adc_value <= adc_min) return 0; if (adc_value >= adc_max) return 100; return (adc_value - adc_min) * 100 / (adc_max - adc_min); }
int convert_adc_to_temp(int adc_value) { if (adc_value >= ADC_TEMP_AXIS) return CAL_TEMP_VALUES; if (adc_value <= ADC_TEMP_AXIS[TEMP_TABLE_SIZE-1]) return CAL_TEMP_VALUES[TEMP_TABLE_SIZE-1]; for (int i = 0; i < TEMP_TABLE_SIZE - 1; i++) { if (adc_value <= ADC_TEMP_AXIS[i] && adc_value >= ADC_TEMP_AXIS[i+1]) { return CAL_TEMP_VALUES[i] + (CAL_TEMP_VALUES[i+1] - CAL_TEMP_VALUES[i]) * (adc_value - ADC_TEMP_AXIS[i]) / (ADC_TEMP_AXIS[i+1] - ADC_TEMP_AXIS[i]); } } return 20; }
float get_interpolated_dead_time(float voltage) { if (voltage <= 10.0f) return 1.60f; if (voltage >= 14.0f) return 0.80f; float factor = (voltage - 10.0f) / (14.0f - 10.0f); return 1.60f + factor * (0.80f - 1.60f); }
float get_warmup_coefficient(int temp) { if (temp <= -10) return 1.50f; if (temp >= 80) return 1.00f; float factor = (float)(temp - (-10)) / (80 - (-10)); return 1.50f - factor * (1.50f - 1.00f); }
float get_interpolated_value(const float map[MAP_RPM_SIZE][MAP_LOAD_SIZE], int current_rpm, int current_load) { int r1 = 0, r2 = 0, l1 = 0, l2 = 0; if (current_rpm < RPM_AXIS) current_rpm = RPM_AXIS; if (current_rpm > RPM_AXIS[MAP_RPM_SIZE-1]) current_rpm = RPM_AXIS[MAP_RPM_SIZE-1]; if (current_load < LOAD_AXIS) current_load = LOAD_AXIS; if (current_load > LOAD_AXIS[MAP_LOAD_SIZE-1]) current_load = LOAD_AXIS[MAP_LOAD_SIZE-1]; for (int i = 0; i < MAP_RPM_SIZE - 1; i++) { if (current_rpm >= RPM_AXIS[i] && current_rpm <= RPM_AXIS[i+1]) { r1 = i; r2 = i + 1; break; } } for (int j = 0; j < MAP_LOAD_SIZE - 1; j++) { if (current_load >= LOAD_AXIS[j] && current_load <= LOAD_AXIS[j+1]) { l1 = j; l2 = j + 1; break; } } float rpm_factor = (float)(current_rpm - RPM_AXIS[r1]) / (RPM_AXIS[r2] - RPM_AXIS[r1]); float load_factor = (float)(current_load - LOAD_AXIS[l1]) / (LOAD_AXIS[l2] - LOAD_AXIS[l1]); float r1_interp = map[r1][l1] + rpm_factor * (map[r2][l1] - map[r1][l1]); float r2_interp = map[r1][l2] + rpm_factor * (map[r2][l2] - map[r1][l2]); return r1_interp + load_factor * (r2_interp - r1_interp); }

int main(void)
{
  HAL_Init();           
  SystemClock_Config(); 
  MX_GPIO_Init();      
  MX_DMA_Init();       
  MX_ADC1_Init();      
  MX_TIM1_Init();      
  MX_I2C1_Init();      

  // Теперь просим DMA собирать 6 каналов датчиков
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buffer, 6);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_cb_hw_i2c, &hi2c1);
  u8g2_InitDisplay(&u8g2);    
  u8g2_SetPowerSave(&u8g2, 0); 

  char str_rpm[16]; char str_fuel[16]; char str_uoz[16];

  while (1)
  {
    /* USER CODE BEGIN WHILE */

    // Читаем датчики из памяти
    uint16_t adc_dad  = adc_raw_buffer[0]; 
    uint16_t adc_tps  = adc_raw_buffer[1]; 
    uint16_t adc_dt   = adc_raw_buffer[2]; 
    uint16_t adc_knock = adc_raw_buffer[4]; // Наш новый датчик детонации (пьезо)
    uint16_t adc_bat   = adc_raw_buffer[5]; 

    battery_voltage = (float)adc_bat * (3.3f / 4095.0f) * 4.0f; 

    // Защита датчиков (Limp Home)
    int tps_ok = (adc_tps > 200 && adc_tps < 4000);
    int dad_ok = (adc_dad > 300 && adc_dad < 3900);

    if (dad_ok && tps_ok) { p_manifold = convert_adc_to_kpa(adc_dad); tps = convert_adc_to_tps(adc_tps); } 
    else if (!dad_ok && tps_ok) { tps = convert_adc_to_tps(adc_tps); p_manifold = 20 + (tps * 0.8f); } 
    else { p_manifold = 60; tps = 30; }

    temperature = convert_adc_to_temp(adc_dt); 

    // --- ШАГ 3: ЛОГИКА ДИНАМИЧЕСКОГО ОТСКОКА ПО ДЕТОНАЦИИ ---
    if (adc_knock > KNOCK_THRESHOLD) {
        // Зафиксирован металлический звон! Детонация!
        // Мгновенно убавляем УОЗ назад (прибавляем отрицательное число)
        knock_correction -= KNOCK_RETARD;
        // Ограничиваем максимальный отскок, например, не более 12 градусов, чтобы мотор не потерял всю тягу
        if (knock_correction < -12.0f) knock_correction = -12.0f; 
    } else {
        // Если звона нет, медленно и плавно возвращаем угол к базовой таблице
        knock_correction += KNOCK_RECOVERY;
        if (knock_correction > 0.0f) knock_correction = 0.0f; // Выше базы прыгать нельзя
    }

    // --- ШАГ 4: МАТЕМАТИКА ИНТЕРПОЛЯЦИИ КАРТ 8х8 ---
    ve_coefficient = get_interpolated_value(VE_MAP, rpm, p_manifold); 
    
    // Базовый идеальный угол зажигания из таблицы
    float base_uoz = get_interpolated_value(IGN_MAP, rpm, p_manifold); 
    
    // Финальный угол зажигания с учетом динамического отскока по детонации!
    uoz = base_uoz + knock_correction; 

    // --- ШАГ 5: РАСЧЕТ ТОПЛИВА ---
    base_fuel = p_manifold * 0.12f;                                
    float warmup_coeff = get_warmup_coefficient(temperature);     
    float dead_time = get_interpolated_dead_time(battery_voltage); 

    total_fuel = (base_fuel * ve_coefficient * warmup_coeff * o2_trim) + dead_time;

    // Отсечка и ПХХ по топливу
    if (rpm >= REV_LIMIT || (tps == 0 && rpm > 1500)) { total_fuel = 0.0f; }

    // Выдача ШИМ на форсунку (ножка PA8)
    uint32_t timer_ticks = (uint32_t)(total_fuel * 100.0f);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, timer_ticks);

    // --- ШАГ 6: УПРАВЛЕНИЕ ИСКРОЙ ЗАЖИГАНИЯ (ДВЕ КАТУШКИ) ---
    if (current_cylinder_pair == 14) { 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // ИСКРА А! Корректированный угол убережет поршни 1-4 от прогара
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); 
    } 
    else if (current_cylinder_pair == 23) { 
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // ИСКРА Б!
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); 
    }

    // --- ШАГ 7: ЭКРАН 1.3" ---
    sprintf(str_rpm,  "RPM: %d", rpm);           
    sprintf(str_fuel, "INJ: %.2f ms", total_fuel); 
    sprintf(str_uoz,  "ADV: %.1f deg", uoz);     // На экране будет виден реальный угол с учетом отскока!

    u8g2_ClearBuffer(&u8g2);                      
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);   
    u8g2_DrawStr(&u8g2, 5, 15, "- ECU WITH KNOCK -"); 
    u8g2_DrawStr(&u8g2, 5, 32, str_rpm);          
    u8g2_DrawStr(&u8g2, 5, 48, str_fuel);         
    u8g2_DrawStr(&u8g2, 5, 64, str_uoz);          
    u8g2_SendBuffer(&u8g2);                       

    HAL_IWDG_Refresh(&hiwdg); 
    HAL_Delay(100); 
    
    /* USER CODE END WHILE */
  }
}
