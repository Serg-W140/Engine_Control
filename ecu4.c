// 2 ДАД, производительность форсунки
#include "main.h"   // Подключаем базовую библиотеку STM32 HAL от производителя
#include "u8g2.h"   // Подключаем графическую библиотеку для OLED-экранов 1.3"
#include <stdio.h>  // Нужна для функции sprintf (перевод чисел в строки текста)

// --- НАСТРОЙКИ СЕТКИ КАРТ И ДАТЧИКОВ ---
#define MAP_RPM_SIZE  8   // Размер оси оборотов (8 точек)
#define MAP_LOAD_SIZE 8   // Размер оси нагрузки (8 точек)
#define SENSOR_MIN    5   // Нижний порог исправности датчиков АЦП (5% от 5В)
#define SENSOR_MAX    95  // Верхний порог исправности датчиков АЦП (95% от 5В)
#define REV_LIMIT     5500 // Лимит оборотов (жесткая топливная отсечка)
#define TEMP_TABLE_SIZE 6  // Размер таблицы тарировки датчика температуры (ДТОЖ)

// --- МАСКИ ОШИБОК ДЛЯ ЖУРНАЛА ДИАГНОСТИКИ (OBD) ---
#define ERR_NONE         0x00  // Ошибок нет
#define ERR_DAD_MANIFOLD 0x01  // Бит 0: Отказал ДАД во впускном коллекторе
#define ERR_DAD_AMBIENT  0x02  // Бит 1: Отказал ДАД атмосферного давления
#define ERR_TPS_BROKEN   0x04  // Бит 2: Отказал ДПДЗ положения заслонки ВАЗ
#define ERR_INJECTOR_1   0x08  // Бит 3: Зафиксирован сбой/обрыв Форсунки №1

#define FLASH_LOG_ADDRESS 0x080E0000 // Адрес сектора Flash-памяти для сохранения лога отказов

// --- ПАРАМЕТРЫ ФИЗИЧЕСКОГО ЖЕЛЕЗА МОТОРА ---
// Производительность форсунки в куб.см/мин (статическая константа).
// Для форсунки ВАЗ-2107 обычно 150.0, для ГАЗ Волга — 192.0. 
// Чем больше форсунка, тем МЕНЬШЕ миллисекунд ей нужно быть открытой для одного и того же объема воздуха!
const float INJECTOR_FLOW = 150.0f; 

// Аппаратные указатели на периферию STM32 (генерируются CubeMX)
extern ADC_HandleTypeDef hadc1;  extern TIM_HandleTypeDef htim1;  extern I2C_HandleTypeDef hiwdg;  extern I2C_HandleTypeDef hi2c1;  

// Оси координат для карт 8х8
const int RPM_AXIS[MAP_RPM_SIZE]   = {800, 1200, 1600, 2200, 3000, 4000, 5000, 6000}; 
const int LOAD_AXIS[MAP_LOAD_SIZE] = {20,  30,  45,  60,  70,  80,  90,  100};  

// Таблица АЦП-Тарировки ДТОЖ ВАЗ
const int ADC_TEMP_AXIS[TEMP_TABLE_SIZE]   = {3800, 3100, 2048, 1000, 450,  200}; 
const int CAL_TEMP_VALUES[TEMP_TABLE_SIZE] = {-10,  0,    20,   50,   80,   100}; 

// 3D Карта наполнения цилиндров (VE MAP) 8х8
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

// 3D Карта Углов Зажигания (IGN MAP) 8х8
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

// --- ОПЕРАТИВНЫЕ ПЕРЕМЕННЫЕ (SRAM) ---
// Массив DMA расширяем до 7 каналов для второго ДАД!
uint16_t adc_raw_buffer[7]; // 0-ДАД коллектора, 1-ДАД атмосфера, 2-ДПДЗ, 3-ДТОЖ, 4-ДК (Лямбда), 5-Датчик Детонации, 6-АКБ
u8g2_t u8g2;                  

int rpm = 800; int current_cylinder_pair = 14; 
int p_manifold = 40; int p_ambient = 100; int p_delta = 60; // Переменные давления (добавили p_ambient и p_delta)
int tps = 0; int temperature = 80; float battery_voltage = 14.0f;

float knock_correction = 0.0f; float ve_coefficient = 1.0f; float uoz = 10.0f; float total_fuel = 0.0f; float o2_trim = 1.00f;
uint8_t error_registry = ERR_NONE; // Регистр кодов неисправностей ОБД

// --- ФУНКЦИИ ТАРИРОВК И ИНТЕРПОЛЯЦИИ ---
int convert_adc_to_kpa(int adc_value) { int adc_min = 400; int adc_max = 3600; if (adc_value <= adc_min) return 20; if (adc_value >= adc_max) return 100; return 20 + (100 - 20) * (adc_value - adc_min) / (adc_max - adc_min); }
int convert_adc_to_tps(int adc_value) { int adc_min = 200; int adc_max = 3900; if (adc_value <= adc_min) return 0; if (adc_value >= adc_max) return 100; return (adc_value - adc_min) * 100 / (adc_max - adc_min); }
int convert_adc_to_temp(int adc_value) { if (adc_value >= ADC_TEMP_AXIS) return CAL_TEMP_VALUES[0]; if (adc_value <= ADC_TEMP_AXIS[TEMP_TABLE_SIZE-1]) return CAL_TEMP_VALUES[TEMP_TABLE_SIZE-1]; for (int i = 0; i < TEMP_TABLE_SIZE - 1; i++) { if (adc_value <= ADC_TEMP_AXIS[i] && adc_value >= ADC_TEMP_AXIS[i+1]) { return CAL_TEMP_VALUES[i] + (CAL_TEMP_VALUES[i+1] - CAL_TEMP_VALUES[i]) * (adc_value - ADC_TEMP_AXIS[i]) / (ADC_TEMP_AXIS[i+1] - ADC_TEMP_AXIS[i]); } } return 20; }
float get_interpolated_dead_time(float voltage) { if (voltage <= 10.0f) return 1.60f; if (voltage >= 14.0f) return 0.80f; float factor = (voltage - 10.0f) / (14.0f - 10.0f); return 1.60f + factor * (0.80f - 1.60f); }
float get_warmup_coefficient(int temp) { if (temp <= -10) return 1.50f; if (temp >= 80) return 1.00f; float factor = (float)(temp - (-10)) / (80 - (-10)); return 1.50f - factor * (1.50f - 1.00f); }

float get_interpolated_value(const float map[MAP_RPM_SIZE][MAP_LOAD_SIZE], int current_rpm, int current_load) {
    int r1 = 0, r2 = 0, l1 = 0, l2 = 0;
    if (current_rpm < RPM_AXIS[0]) current_rpm = RPM_AXIS[0]; if (current_rpm > RPM_AXIS[MAP_RPM_SIZE-1]) current_rpm = RPM_AXIS[MAP_RPM_SIZE-1];
    if (current_load < LOAD_AXIS[0]) current_load = LOAD_AXIS[0]; if (current_load > LOAD_AXIS[MAP_LOAD_SIZE-1]) current_load = LOAD_AXIS[MAP_LOAD_SIZE-1];
    for (int i = 0; i < MAP_RPM_SIZE - 1; i++) { if (current_rpm >= RPM_AXIS[i] && current_rpm <= RPM_AXIS[i+1]) { r1 = i; r2 = i + 1; break; } }
    for (int j = 0; j < MAP_LOAD_SIZE - 1; j++) { if (current_load >= LOAD_AXIS[j] && current_load <= LOAD_AXIS[j+1]) { l1 = j; l2 = j + 1; break; } }
    float rpm_factor = (float)(current_rpm - RPM_AXIS[r1]) / (RPM_AXIS[r2] - RPM_AXIS[r1]); float load_factor = (float)(current_load - LOAD_AXIS[l1]) / (LOAD_AXIS[l2] - LOAD_AXIS[l1]);
    float q11 = map[r1][l1]; float q21 = map[r2][l1]; float q12 = map[r1][l2]; float q22 = map[r2][l2];
    float r1_interp = q11 + rpm_factor * (q21 - q11); float r2_interp = q12 + rpm_factor * (q22 - q12);
    return r1_interp + load_factor * (r2_interp - r1_interp);
}

// Функция аппаратной записи лога во Flash-память
void write_error_to_flash(uint8_t error_code) {
    HAL_FLASH_Unlock(); FLASH_EraseInitTypeDef erase_struct; erase_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_struct.Sector = FLASH_SECTOR_11; erase_struct.NbSectors = 1; uint32_t sector_error;
    HAL_FLASHEx_Erase(&erase_struct, &sector_error); HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_LOG_ADDRESS, error_code); HAL_FLASH_Lock();
}
int main(void)
{
  // Стартовая инициализация слоев железа чипа
  HAL_Init(); SystemClock_Config(); MX_GPIO_Init(); MX_DMA_Init(); MX_ADC1_Init(); MX_TIM1_Init(); MX_I2C1_Init();  

  // Запуск фонового сбора 7 датчиков через робота DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buffer, 7);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Старт ШИМ ножки форсунки PA8

  // Старт нашего OLED приборного щитка
  u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_cb_hw_i2c, &hi2c1);
  u8g2_InitDisplay(&u8g2); u8g2_SetPowerSave(&u8g2, 0); 

  char str_rpm[16]; char str_fuel[16]; char str_uoz[16];
  int main_injector_fault = 0; // Переменная статуса КЗ/обрыва форсунки (для теста защит)

  // ========================================================
  // БЕСКОНЕЧНЫЙ ЦИКЛ ОБРАБОТКИ ФИЗИКИ ДВС И БЕЗОПАСНОСТИ КАТЕРА
  // ========================================================
  while (1)
  {
    /* USER CODE BEGIN WHILE */

    // --- ШАГ 1: ВЫТАЛЬКИВАНИЕ СВЕЖИХ ЗАМЕРОВ АЦП ИЗ ОПЕРАТИВКИ ---
    uint16_t adc_dad_manifold = adc_raw_buffer[0]; // ДАД 1 во впуске
    uint16_t adc_dad_ambient  = adc_raw_buffer[1]; // ДАД 2 в атмосфере (ВАША БАРОКОРРЕКЦИЯ!)
    uint16_t adc_tps          = adc_raw_buffer[2]; // ДПДЗ заслонки
    uint16_t adc_dt           = adc_raw_buffer[3]; // ДТОЖ температуры мотора
    uint16_t adc_dk           = adc_raw_buffer[4]; // ДК Лямбда-зонда
    uint16_t adc_knock        = adc_raw_buffer[5]; // ДД микрофон детонации
    uint16_t adc_bat          = adc_raw_buffer[6]; // Вольтметр батареи АКБ

    battery_voltage = (float)adc_bat * (3.3f / 4095.0f) * 4.0f; // Считаем напряжение бортовой сети

    // Проверяем электрическую целостность трех важнейших датчиков
    int dad_man_ok = (adc_dad_manifold > 300 && adc_dad_manifold < 3900);
    int dad_amb_ok = (adc_dad_ambient > 300 && adc_dad_ambient < 3900);
    int tps_ok     = (adc_tps > 200 && adc_tps < 4000);

    // --- ШАГ 2: СИСТЕМА ДИАГНОСТИКИ ОБД И СТРАТЕГИЯ ВЫЖИВАНИЯ (Limp Home) ---
    uint8_t old_errors = error_registry; // Храним прошлый сбой
    error_registry = ERR_NONE;           // Обнуляем флаги перед новой проверкой

    // Броня ДАД Коллектора
    if (!dad_man_ok) { error_registry |= ERR_DAD_MANIFOLD; }
    // Броня ДАД Атмосферы
    if (!dad_amb_ok) { error_registry |= ERR_DAD_AMBIENT; }
    // Броня ДПДЗ заслонки
    if (!tps_ok)     { error_registry |= ERR_TPS_BROKEN; }
    // Броня цепи Форсунки №1
    if (main_injector_fault == 1) { error_registry |= ERR_INJECTOR_1; }

    // АВТО-ЛОГГЕР: Выжигаем во Flash-память только если сбой СВЕЖИЙ
    if (error_registry != old_errors && error_registry != ERR_NONE) { write_error_to_flash(error_registry); }

    // Принятие жестких аварийных решений по расчету давления
    if (dad_man_ok && dad_amb_ok) {
        // РЕЖИМ МАКСИМАЛЬНОЙ ТОЧНОСТИ: Оба ДАД живы, вычисляем ВАШ честный перепад барокоррекции!
        p_manifold = convert_adc_to_kpa(adc_dad_manifold);
        p_ambient  = convert_adc_to_kpa(adc_dad_ambient);
        p_delta = p_ambient - p_manifold; // Чистая нагрузка на ДВС, независимая от высоты в горах
        if (p_delta < 0) p_delta = 0;
    }
    else if (!dad_man_ok && tps_ok) {
        // Умер ДАД коллектора: аварийный расчет перепада по ДПДЗ (Alpha-N)
        tps = convert_adc_to_tps(adc_tps);
        p_ambient = dad_amb_ok ? convert_adc_to_kpa(adc_dad_ambient) : 100;
        p_manifold = 20 + (tps * 0.8f);
        p_delta = p_ambient - p_manifold;
        if (p_delta < 0) p_delta = 0;
    }
    else {
        // Худший апокалипсис: ослепли оба ДАД. Фиксируем средний безопасный перепад
        p_delta = 60; p_manifold = 40; tps = 20;
    }

    temperature = convert_adc_to_temp(adc_dt); // Тарируем нелинейный датчик температуры ВАЗ

    // --- ШАГ 3: ЛОГИКА ДИНАМИЧЕСКОГО ОТСКОКА ПО ДЕТОНАЦИИ (Пьезо-ДД) ---
    if (adc_knock > 2500) { knock_correction -= 4.0f; if (knock_correction < -12.0f) knock_correction = -12.0f; } // Звенит — поздним угол
    else { knock_correction += 0.1f; if (knock_correction > 0.0f) knock_correction = 0.0f; } // Тихо — плавно возвращаем к базе

    // --- ШАГ 4: СГЛАЖЕННАЯ ИНТЕРПОЛЯЦИЯ 3D-КАРТ 8х8 ---
    // Нагрузочную ось карт LOAD_AXIS мы теперь кормим не "сырым" ДАД, а вашим точным барокорректированным p_delta!
    ve_coefficient = get_interpolated_value(VE_MAP, rpm, p_delta); 
    uoz = get_interpolated_value(IGN_MAP, rpm, p_delta) + knock_correction; // Базовый УОЗ + защита от звона

    // --- ШАГ 5: ВЗРОСЛЫЙ ФИЗИЧЕСКИЙ РАСЧЕТ ТОПЛИВА С КОНСТАНТОЙ ФОРСУНКИ ---
    // Формула Идеального Газа: Базовое время открытия жестко привязано к производительности форсунки!
    // Формула: (Давление * Идеальный коэффициент пересчета константы воздуха) / Производительность форсунки в сек
    base_fuel = (p_manifold * 180.0f) / (INJECTOR_FLOW / 60.0f); 
    
    float warmup_coeff = get_warmup_coefficient(temperature);     // Коррекция холодного "подсоса"
    float dead_time = get_interpolated_dead_time(battery_voltage); // Плавный интерполированный лаг клапана от напряжения

    // Итоговое время впрыска
    total_fuel = (base_fuel * ve_coefficient * warmup_coeff * o2_trim) + dead_time;

    // --- ШАГ 6: КАТЕГОРЫ РЕЖИМЫ ОТСЕЧЕК ПО ТОПЛИВУ ---
    if (rpm >= REV_LIMIT || (tps == 0 && rpm > 1500)) { total_fuel = 0.0f; } // Жесткая отсечка или ПХХ

    // --- ШАГ 7: УПРАВЛЕНИЕ ЖЕЛЕЗОМ (Выдача команд на Таймер и Катушки зажигания) ---
    uint32_t timer_ticks = (uint32_t)(total_fuel * 100.0f);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, timer_ticks); // Пишем ШИМ форсунки на ножку PA8

    // Искра лупит всегда (даже в отсечке), выжигая остатки пленки и страхуя от взрыва выхлопа катера
    if (current_cylinder_pair == 14) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // ИСКРА А (PB0)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    } 
    else if (current_cylinder_pair == 23) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // ИСКРА Б (PB1)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    // --- ШАГ 8: УМНЫЙ OLED ДИСПЛЕЙ КАТЕРА (ПРИБОРКА И ЭКРАН СБОЕВ OBD) ---
    u8g2_ClearBuffer(&u8g2); u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);

    if (error_registry == ERR_NONE) {
        // Всё исправно — выводим гоночные параметры лодки
        sprintf(str_rpm,  "RPM: %d", rpm); sprintf(str_fuel, "INJ: %.2f ms", total_fuel); sprintf(str_uoz,  "ADV: %.1f deg", uoz);
        u8g2_DrawStr(&u8g2, 5, 15, "- MOTOR RUNNING -"); u8g2_DrawStr(&u8g2, 5, 32, str_rpm); u8g2_DrawStr(&u8g2, 5, 48, str_fuel); u8g2_DrawStr(&u8g2, 5, 64, str_uoz);
    } 
    else {
        // Авария! Экран сходит с ума и выдает коды ошибок водителю катера
        u8g2_DrawStr(&u8g2, 5, 15, "⚠️ CHECK ENGINE"); u8g2_SetFont(&u8g2, u8g2_font_6x10_tf); int y = 28;
        if (error_registry & ERR_DAD_MANIFOLD) { u8g2_DrawStr(&u8g2, 5, y, "FAIL 01: MANIFOLD MAP"); y += 10; }
        if (error_registry & ERR_DAD_AMBIENT)  { u8g2_DrawStr(&u8g2, 5, y, "FAIL 02: AMBIENT MAP"); y += 10; }
        if (error_registry & ERR_TPS_BROKEN)   { u8g2_DrawStr(&u8g2, 5, y, "FAIL 04: TPS SENSOR"); y += 10; }
        if (error_registry & ERR_INJECTOR_1)   { u8g2_DrawStr(&u8g2, 5, y, "FAIL 08: INJECTOR 1 CLOG"); y += 10; }
        u8g2_DrawStr(&u8g2, 5, 64, "LIMP HOME: SAFE MODE ACTIVE");
    }
    u8g2_SendBuffer(&u8g2); // Отправка графики на экран SH1106 (ножки PB8/PB9)

    // --- ШАГ 9: СБРОС СТОРОЖЕВОГО ПСА (WATCHDOG) ---
    HAL_IWDG_Refresh(&hiwdg); // Если ядро зависнет, пес сделает RESET через 50 мс
    HAL_Delay(100);           // Такт обновления ЭБУ
    /* USER CODE END WHILE */
  }
}
