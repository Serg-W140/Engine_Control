#include "main.h"   
#include "u8g2.h"   
#include <stdio.h>  

// --- НАСТРОЙКИ СЕТКИ КАРТ И ДАТЧИКОВ ---
#define MAP_RPM_SIZE  8   
#define MAP_LOAD_SIZE 8   
#define SENSOR_MIN    5   
#define SENSOR_MAX    95  
#define REV_LIMIT     5500 
#define TEMP_TABLE_SIZE 6  
#define DWELL_POINTS  4   

// --- НАСТРОЙКИ ВНЕШНЕЙ ПАМЯТИ EEPROM ---
#define EEPROM_I2C_ADDRESS  (0x50 << 1) 
#define EEPROM_LOG_CELL     0x0010      

// --- НАСТРОЙКИ ДАТЧИКА ДЕТОНАЦИИ ---
#define KNOCK_THRESHOLD 2500 
#define KNOCK_RETARD    4.0f 
#define KNOCK_RECOVERY  0.001f // Замедлили скорость возврата для адекватной работы в while(1)

// --- МАСКИ ОШИБОК ОБД ---
#define ERR_NONE         0x00  
#define ERR_DAD_MANIFOLD 0x01  
#define ERR_DAD_AMBIENT  0x02  
#define ERR_TPS_BROKEN   0x04  
#define ERR_INJECTOR_1   0x08  

const float INJECTOR_FLOW = 150.0f; 

// Исправленные хэндлы периферии
extern ADC_HandleTypeDef hadc1;  
extern TIM_HandleTypeDef htim1;  // ШИМ форсунки
extern TIM_HandleTypeDef htim2;  // Таймер для точных фаз зажигания (без HAL_Delay)
extern IWDG_HandleTypeDef hiwdg; // ИСПРАВЛЕНО: Правильный тип для Watchdog
extern I2C_HandleTypeDef hi2c1;  

const int RPM_AXIS[MAP_RPM_SIZE]   = {800, 1200, 1600, 2200, 3000, 4000, 5000, 6000}; 
const int LOAD_AXIS[MAP_LOAD_SIZE] = {20,  30,  45,  60,  70,  80,  90,  100};  

const int ADC_TEMP_AXIS[TEMP_TABLE_SIZE]   = {3800, 3100, 2048, 1000, 450,  200}; 
const int CAL_TEMP_VALUES[TEMP_TABLE_SIZE] = {-10,  0,    20,   50,   80,   100}; 

const float DWELL_VOLT_AXIS[DWELL_POINTS] = {10.0f, 12.0f, 14.0f, 16.0f}; 
const float DWELL_TIME_AXIS[DWELL_POINTS] = {5.00f, 3.80f, 3.00f, 2.40f}; 

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

// ИСПРАВЛЕНО: Квалификатор volatile для DMA-буфера
volatile uint16_t adc_raw_buffer[7]; 
u8g2_t u8g2;                  

volatile int rpm = 0;                     // Переменная оборотов (теперь она обновляется сама)
volatile uint32_t last_rpm_tick = 0;      // Время предыдущего импульса ВМТ
volatile uint32_t rpm_timeout_counter = 0;// Счетчик для определения того, что мотор заглох

int current_cylinder_pair = 14; 
int p_manifold = 40; int p_ambient = 100; int p_delta = 60; 
int tps = 0; int temperature = 80; float battery_voltage = 14.0f;

float knock_correction = 0.0f; float ve_coefficient = 1.0f; float uoz = 10.0f; float total_fuel = 0.0f; float o2_trim = 1.00f;
uint8_t error_registry = ERR_NONE; 

// --- ФУНКЦИИ ИНТЕРПОЛЯЦИИ ---

int convert_adc_to_kpa(int adc_value) {
    int adc_min = 400; int adc_max = 3600; 
    if (adc_value <= adc_min) return 20;   
    if (adc_value >= adc_max) return 100;  
    return 20 + (100 - 20) * (adc_value - adc_min) / (adc_max - adc_min); 
}

int convert_adc_to_tps(int adc_value) {
    int adc_min = 200; int adc_max = 3900; 
    if (adc_value <= adc_min) return 0;    
    if (adc_value >= adc_max) return 100;  
    // ИСПРАВЛЕНО: Повысили точность расчета ДПДЗ, убрали грубое округление
    return ((adc_value - adc_min) * 100) / (adc_max - adc_min); 
}

int convert_adc_to_temp(int adc_value) {
    if (adc_value >= ADC_TEMP_AXIS[0]) return CAL_TEMP_VALUES[0]; 
    if (adc_value <= ADC_TEMP_AXIS[TEMP_TABLE_SIZE-1]) return CAL_TEMP_VALUES[TEMP_TABLE_SIZE-1]; 
    for (int i = 0; i < TEMP_TABLE_SIZE - 1; i++) { 
        if (adc_value <= ADC_TEMP_AXIS[i] && adc_value >= ADC_TEMP_AXIS[i+1]) {
            return CAL_TEMP_VALUES[i] + (CAL_TEMP_VALUES[i+1] - CAL_TEMP_VALUES[i]) * (adc_value - ADC_TEMP_AXIS[i]) / (ADC_TEMP_AXIS[i+1] - ADC_TEMP_AXIS[i]);
        }
    }
    return 20; 
}

float get_interpolated_dead_time(float voltage) {
    if (voltage <= 10.0f) return 1.60f; 
    if (voltage >= 14.0f) return 0.80f; 
    float factor = (voltage - 10.0f) / (14.0f - 10.0f);
    return 1.60f + factor * (0.80f - 1.60f);
}

float get_warmup_coefficient(int temp) {
    if (temp <= -10) return 1.50f; 
    if (temp >= 80)  return 1.00f;  
    float factor = (float)(temp - (-10)) / (80 - (-10)); 
    return 1.50f - factor * (1.50f - 1.00f);
}

float get_interpolated_dwell_time(float voltage) {
    if (voltage <= DWELL_VOLT_AXIS[0]) return DWELL_TIME_AXIS[0]; 
    if (voltage >= DWELL_VOLT_AXIS[DWELL_POINTS-1]) return DWELL_TIME_AXIS[DWELL_POINTS-1]; 

    for (int i = 0; i < DWELL_POINTS - 1; i++) {
        if (voltage >= DWELL_VOLT_AXIS[i] && voltage <= DWELL_VOLT_AXIS[i+1]) {
            float factor = (voltage - DWELL_VOLT_AXIS[i]) / (DWELL_VOLT_AXIS[i+1] - DWELL_VOLT_AXIS[i]);
            return DWELL_TIME_AXIS[i] + factor * (DWELL_TIME_AXIS[i+1] - DWELL_TIME_AXIS[i]);
        }
    }
    return 3.50f; 
}

// ИСПРАВЛЕНО: Устранено деление на 0 и зацикливание на границах 3D карт
float get_interpolated_value(const float map[MAP_RPM_SIZE][MAP_LOAD_SIZE], int current_rpm, int current_load) {
    int r1 = 0, r2 = 0, l1 = 0, l2 = 0;
    
    if (current_rpm < RPM_AXIS[0]) current_rpm = RPM_AXIS[0];
    if (current_rpm >= RPM_AXIS[MAP_RPM_SIZE-1]) {
        r1 = MAP_RPM_SIZE - 2; r2 = MAP_RPM_SIZE - 1;
        current_rpm = RPM_AXIS[MAP_RPM_SIZE-1];
    } else {
        for (int i = 0; i < MAP_RPM_SIZE - 1; i++) { 
            if (current_rpm >= RPM_AXIS[i] && current_rpm < RPM_AXIS[i+1]) { r1 = i; r2 = i + 1; break; } 
        }
    }

    if (current_load < LOAD_AXIS[0]) current_load = LOAD_AXIS[0];
    if (current_load >= LOAD_AXIS[MAP_LOAD_SIZE-1]) {
        l1 = MAP_LOAD_SIZE - 2; l2 = MAP_LOAD_SIZE - 1;
        current_load = LOAD_AXIS[MAP_LOAD_SIZE-1];
    } else {
        for (int j = 0; j < MAP_LOAD_SIZE - 1; j++) { 
            if (current_load >= LOAD_AXIS[j] && current_load < LOAD_AXIS[j+1]) { l1 = j; l2 = j + 1; break; } 
        }
    }

    float rpm_factor = (float)(current_rpm - RPM_AXIS[r1]) / (RPM_AXIS[r2] - RPM_AXIS[r1]);
    float load_factor = (float)(current_load - LOAD_AXIS[l1]) / (LOAD_AXIS[l2] - LOAD_AXIS[l1]);

    float q11 = map[r1][l1]; float q21 = map[r2][l1]; float q12 = map[r1][l2]; float q22 = map[r2][l2];
    float r1_interp = q11 + rpm_factor * (q21 - q11);
    float r2_interp = q12 + rpm_factor * (q22 - q12);
    return r1_interp + load_factor * (r2_interp - r1_interp); 
}

// ИСПРАВЛЕНО: Защита EEPROM от циклического выжигания (таймаут записи)
void write_error_to_eeprom(uint8_t error_code) {
    static uint32_t last_write_time = 0;
    if (HAL_GetTick() - last_write_time < 5000) return; // Писать не чаще чем раз в 5 секунд!
    
    // ИСПРАВЛЕНО: Безопасный мелкий таймаут 2мс, чтобы шина I2C не вешала мотор на 100мс
    HAL_I2C_Mem_Write(&hi2c1, EEPROM_I2C_ADDRESS, EEPROM_LOG_CELL, I2C_MEMADD_SIZE_16BIT, &error_code, 1, 2);
    last_write_time = HAL_GetTick();
}

int main(void)
{
  HAL_Init();           
  // Предположим, инициализация периферии CubeMX выполнена корректно...

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buffer, 7);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_cb_hw_i2c, &hi2c1);
  u8g2_InitDisplay(&u8g2);    
  u8g2_SetPowerSave(&u8g2, 0); 

  char str_rpm[16]; char str_fuel[16]; char str_uoz[16];
  int main_injector_fault = 0; 
  
  uint32_t last_display_tick = 0; // Переменная для таймера экрана

  while (1)
  {

     // БЕЗОПАСНОСТЬ: Если импульсов нет больше 200 мс — значит мотор заглох или выключен
    if (HAL_GetTick() - last_rpm_tick > 200) {
        rpm = 0;
    }

    // --- ШАГ 1: СБОР ДАННЫХ ИЗ ОПЕРАТИВКИ ---
    uint16_t adc_dad_manifold = adc_raw_buffer[0]; 
    uint16_t adc_dad_ambient  = adc_raw_buffer[1]; 
    uint16_t adc_tps          = adc_raw_buffer[2]; 
    uint16_t adc_dt           = adc_raw_buffer[3]; 
    uint16_t adc_dk           = adc_raw_buffer[4]; 
    uint16_t adc_knock        = adc_raw_buffer[5]; 
    uint16_t adc_bat          = adc_raw_buffer[6]; 

    // ИСПРАВЛЕНО: Безопасный делитель (например, 10 кОм к АКБ и 2.2 кОм к земле дает коэфф 5.54)
    // Подставьте коэффициент под ваше РЕАЛЬНОЕ физическое железо!
    battery_voltage = (float)adc_bat * (3.3f / 4095.0f) * 5.54f; 

    int dad_man_ok = (adc_dad_manifold > SENSOR_MIN*41 && adc_dad_manifold < SENSOR_MAX*41);
    int dad_amb_ok = (adc_dad_ambient > SENSOR_MIN*41 && adc_dad_ambient < SENSOR_MAX*41);
    int tps_ok     = (adc_tps > 200 && adc_tps < 4000);

    // --- ШАГ 2: ОБД ДИАГНОСТИКА ---
    uint8_t old_errors = error_registry; 
    error_registry = ERR_NONE;           

    if (!dad_man_ok) { error_registry |= ERR_DAD_MANIFOLD; }
    if (!dad_amb_ok) { error_registry |= ERR_DAD_AMBIENT; }
    if (!tps_ok)     { error_registry |= ERR_TPS_BROKEN; }
    if (main_injector_fault == 1) { error_registry |= ERR_INJECTOR_1; }

    if (error_registry != old_errors && error_registry != ERR_NONE) { 
        write_error_to_eeprom(error_registry); 
    }

    if (dad_man_ok && dad_amb_ok) {
        p_manifold = convert_adc_to_kpa(adc_dad_manifold);
        p_ambient  = convert_adc_to_kpa(adc_dad_ambient);
        p_delta = p_ambient - p_manifold; 
        if (p_delta < 0) p_delta = 0;
    }
    else if (!dad_man_ok && tps_ok) {
        tps = convert_adc_to_tps(adc_tps);
        p_ambient = dad_amb_ok ? convert_adc_to_kpa(adc_dad_ambient) : 100;
        p_manifold = 20 + (tps * 0.8f);
        p_delta = p_ambient - p_manifold;
        if (p_delta < 0) p_delta = 0;
    }
    else {
        p_delta = 60; p_manifold = 40; tps = 20;
    }

temperature = convert_adc_to_temp(adc_dt);// --- ШАГ 3: ДЕТОНАЦИЯ ---
if (adc_knock > KNOCK_THRESHOLD) {
    knock_correction -= KNOCK_RETARD;
    if (knock_correction < -12.0f) knock_correction = -12.0f;
} 
else {
    knock_correction += KNOCK_RECOVERY;
    if (knock_correction > 0.0f) knock_correction = 0.0f;
}

// --- ШАГ 4: МАТЕМАТИКА КАРТ ---
ve_coefficient = get_interpolated_value(VE_MAP, rpm, p_delta);
float base_uoz = get_interpolated_value(IGN_MAP, rpm, p_delta);
uoz = base_uoz + knock_correction;

// --- ШАГ 5: ЛЯМБДА-ЗОНД ---
if (temperature >= 70 && tps < 80) {
    // ИСПРАВЛЕНО: Опорное напряжение 3.3В вместо неверных 5.0В
    float dk_voltage = (float)adc_dk * (3.3f / 4095.0f);
    if (dk_voltage < 0.4f) {o2_trim += 0.001f; 
        if (o2_trim > 1.20f) o2_trim = 1.20f;
    } 
    else if (dk_voltage > 0.5f) {
        o2_trim -= 0.001f; 
        if (o2_trim < 0.80f) o2_trim = 0.80f;
    }
}
else {
    o2_trim = 1.00f;
}

// --- ШАГ 6: РАСЧЕТ ТОПЛИВА ---// ИСПРАВЛЕНО: Объявлен тип переменной float для base_fuel
float base_fuel = (p_manifold * 180.0f) / (INJECTOR_FLOW / 60.0f);
float warmup_coeff = get_warmup_coefficient(temperature);
float dead_time = get_interpolated_dead_time(battery_voltage);
total_fuel = (base_fuel * ve_coefficient * warmup_coeff * o2_trim) + dead_time;
if (rpm >= REV_LIMIT || (tps == 0 && rpm > 1500)) { 
    total_fuel = 0.0f; 
}
// --- УПРАВЛЕНИЕ ЖЕЛЕЗОМ ФОРСУНКИ ---
uint32_t timer_ticks = (uint32_t)(total_fuel * 100.0f);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, timer_ticks);
int active_injector = (error_registry & ERR_INJECTOR_1) ? 2 : 1;

// --- ШАГ 7: УПРАВЛЕНИЕ ИСКРОЙ ЗАЖИГАНИЯ С АЛГОРИТМОМ УПРЕЖДЕНИЯ НАКОПЛЕНИЯ ---
    // 1. Считаем время накопления от вольтажа (в миллисекундах)
    float current_dwell_ms = get_interpolated_dwell_time(battery_voltage); 
    
    // 2. Переводим миллисекунды накопления в градусы ПКВ
    float dwell_degrees = (current_dwell_ms * rpm * 6.0f) / 1000.0f; 

    // 3. Находим угол начала заряда катушки (УОЗ + угол накопления)
    float charge_angle = uoz + dwell_degrees; 

    // 4. ПЕРЕВОД МАТЕМАТИКИ В ТИКИ ТАЙМЕРА (Настройка аппаратного импульса)
    // Допустим, таймер htim2 настроен так, что 1 тик = 1 микросекунда (Pulse = время заряда катушки)
    uint32_t dwell_ticks = (uint32_t)(current_dwell_ms * 1000.0f);
    
    // Передаем значение длительности заряда в регистр сравнения таймера зажигания
    // Как только сработает триггер от датчика коленвала, таймер сам поднимет ножку катушки
    // ровно на dwell_ticks микросекунд, выдаст искру при выключении и уснет.
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dwell_ticks);

    // 5. АППАРАТНОЕ ПЕРЕКЛЮЧЕНИЕ КАНАЛОВ (Выбор активной катушки)
    if (current_cylinder_pair == 14) {
        // Направляем следующий импульс таймера на Катушку А (Цилиндры 1-4)
        // Для этого аппаратно переназначается пин или активируется нужный канал таймера
        // HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_1); // Запуск готовности по прерыванию ДПКВ
    } 
    else if (current_cylinder_pair == 23) {
        // Направляем следующий импульс таймера на Катушку Б (Цилиндры 2-3)
        // HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2);
    }
// --- ШАГ 8: АСИНХРОННЫЙ ВЫВОД НА OLED ЭКРАН (ИСПРАВЛЕНО) ---
// Обновляем экран ровно раз в 200 миллисекунд, чтобы разгрузить ядро и спасти Watchdog!
if (HAL_GetTick() - last_display_tick >= 200) {
    last_display_tick = HAL_GetTick();
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
    
    if (error_registry == ERR_NONE) {
        sprintf(str_rpm,  "RPM: %d", rpm);
        // Примечание: Убедитесь, что в опциях проекта STM32CubeIDE включен флаг "-u _printf_float"
        sprintf(str_fuel, "INJ: %.2f ms", total_fuel);
        sprintf(str_uoz,  "ADV: %.1f deg", uoz);
        
        u8g2_DrawStr(&u8g2, 5, 15, "- MOTOR RUNNING -");
        u8g2_DrawStr(&u8g2, 5, 32, str_rpm);
        u8g2_DrawStr(&u8g2, 5, 48, str_fuel);
        u8g2_DrawStr(&u8g2, 5, 64, str_uoz);
        }
        else {
            u8g2_DrawStr(&u8g2, 5, 15, "CHECK ENGINE");
            u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
            int y = 28;
        if (error_registry & ERR_DAD_MANIFOLD) { 
            u8g2_DrawStr(&u8g2, 5, y, "FAIL 01: MANIFOLD MAP");
            y += 10;
        }
        if (error_registry & ERR_DAD_AMBIENT)  { 
            u8g2_DrawStr(&u8g2, 5, y, "FAIL 02: AMBIENT MAP");
            y += 10;
        }
        if (error_registry & ERR_TPS_BROKEN) {
            u8g2_DrawStr(&u8g2, 5, y, "FAIL 04: TPS SENSOR");
            y += 10;
        }
        if (error_registry & ERR_INJECTOR_1)   {
            u8g2_DrawStr(&u8g2, 5, y, "FAIL 08: INJECTOR 1 CLOG");
            y += 10;
        }

        sprintf(str_fuel, "LIMP MODE. INJ %d", active_injector);
        u8g2_DrawStr(&u8g2, 5, 64, str_fuel);
        }
        u8g2_SendBuffer(&u8g2);
    }

// --- ШАГ 9: СБРОС СТОРОЖЕВОГО ПСА ---
// Теперь цикл крутится быстро, и 50мс таймаута IWDG будет хватать с запасом
HAL_IWDG_Refresh(&hiwdg);
// ИСПРАВЛЕНО: Убрана жесткая задержка HAL_Delay(100), цикл работает на максимальной скорости
    }
}
    // Прерывание вызывается внешней схемой в момент, когда коленвал доходит до расчетного угла НАЧАЛА заряда (charge_angle)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 1. ПРЕРЫВАНИЕ ДЛЯ ИСКРЫ (Ваш Шаг 7, реагирует на PIN 0)
    if (GPIO_Pin == GPIO_PIN_0) 
    {
        uint32_t dwell_ticks = (uint32_t)(get_interpolated_dwell_time(battery_voltage) * 1000.0f);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dwell_ticks);
        __HAL_TIM_SET_AUTORELOAD(&htim2, dwell_ticks + 10); 

        if (current_cylinder_pair == 14) {
            HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_1); 
        } else if (current_cylinder_pair == 23) {
            HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2); 
        }
    }
    
    // 2. ВОТ ЭТОТ КУСОК ДОБАВЛЯЕМ: ПРЕРЫВАНИЕ ДЛЯ РАСЧЕТА ОБОРОТОВ (Реагирует на PIN 2)
    else if (GPIO_Pin == GPIO_PIN_2) 
    {
        uint32_t current_tick = HAL_GetTick(); // Получаем текущее время в миллисекундах
        uint32_t period = current_tick - last_rpm_tick; // Считаем, сколько мс прошло с прошлого оборота
        
        if (period > 0) // Защита от деления на ноль
        {
            // Формула: в 1 минуте = 60 000 миллисекунд. Делим на период одного оборота.
            rpm = 60000 / period; 
        }
        
        last_rpm_tick = current_tick; // Запоминаем время этого импульса для следующего круга
    }
} // Конец файла