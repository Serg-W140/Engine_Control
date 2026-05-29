// Логи во внешнюю память
#include "main.h"   // Базовая библиотека STM32 HAL от производителя
#include "u8g2.h"   // Графическая библиотека для OLED-экранов 1.3" (SH1106)
#include <stdio.h>  // Для функции sprintf (форматирование текста)

// --- НАСТРОЙКИ СЕТКИ КАРТ И ДАТЧИКОВ ---
#define MAP_RPM_SIZE  8   // Количество точек на оси оборотов
#define MAP_LOAD_SIZE 8   // Количество точек на оси нагрузки
#define SENSOR_MIN    5   // Нижний порог исправности датчиков АЦП (5% от 5В)
#define SENSOR_MAX    95  // Верхний порог исправности датчиков АЦП (95% от 5В)
#define REV_LIMIT     5500 // Жесткая топливная отсечка (об/мин)
#define TEMP_TABLE_SIZE 6  // Размер таблицы тарировки температуры ДТОЖ

// --- НАСТРОЙКИ ВНЕШНЕЙ ПАМЯТИ EEPROM ---
#define EEPROM_I2C_ADDRESS  (0x50 << 1) // I2C адрес микросхемы AT24C256 со сдвигом для HAL
#define EEPROM_LOG_CELL     0x0010      // Адрес ячейки памяти внутри EEPROM для лога ошибок

// --- НАСТРОЙКИ ДАТЧИКА ДЕТОНАЦИИ ---
#define KNOCK_THRESHOLD 2500 // Порог АЦП, выше которого шум считается звоном мотора
#define KNOCK_RETARD    4.0f // Мгновенный отскок УОЗ назад при детонации (градусы)
#define KNOCK_RECOVERY  0.1f // Скорость возврата угла к базовой карте (градусов/шаг)

// --- МАСКИ ОШИБОК ДЛЯ ЖУРНАЛА ДИАГНОСТИКИ (OBD) ---
#define ERR_NONE         0x00  // Ошибок нет
#define ERR_DAD_MANIFOLD 0x01  // Бит 0: Отказал ДАД во впускном коллекторе
#define ERR_DAD_AMBIENT  0x02  // Бит 1: Отказал ДАД атмосферного давления
#define ERR_TPS_BROKEN   0x04  // Бит 2: Отказал ДПДЗ положения заслонки ВАЗ
#define ERR_INJECTOR_1   0x08  // Бит 3: Зафиксирован сбой/обрыв Форсунки №1

// --- ТАРИРОВКА ВРЕМЕНИ НАКОПЛЕНИЯ КАТУШКИ ОТ НАПРЯЖЕНИЯ Бортсети ---
#define DWELL_POINTS 4
const float DWELL_VOLT_AXIS[DWELL_POINTS] = {10.0f, 12.0f, 14.0f, 16.0f}; // Напряжение сети в Вольтах
const float DWELL_TIME_AXIS[DWELL_POINTS] = {5.00f, 3.80f, 3.00f, 2.40f; // Время накопления в миллисекундах

// --- ПАРАМЕТРЫ ФИЗИЧЕСКОГО ЖЕЛЕЗА МОТОРА ---
const float INJECTOR_FLOW = 150.0f; // Производительность форсунки ВАЗ (куб.см/мин)

// Аппаратные указатели на периферию STM32 (создаются автоматически через CubeMX)
extern ADC_HandleTypeDef hadc1;  
extern TIM_HandleTypeDef htim1;  
extern I2C_HandleTypeDef hiwdg;  
extern I2C_HandleTypeDef hi2c1;  

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
    { 0.56f, 0.65f, 0.75f, 0.83f, 0.87f, 0.90f, 0.93f, 0.95f }, // Пик момента ВАЗ
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
// Массив DMA из 7 элементов: 0-ДАД коллектор, 1-ДАД атмосфера, 2-ДПДЗ, 3-ДТОЖ, 4-ДК, 5-ДД, 6-АКБ
uint16_t adc_raw_buffer[7]; 
u8g2_t u8g2;                  

int rpm = 800; int current_cylinder_pair = 14; 
int p_manifold = 40; int p_ambient = 100; int p_delta = 60; 
int tps = 0; int temperature = 80; float battery_voltage = 14.0f;

float knock_correction = 0.0f; float ve_coefficient = 1.0f; float uoz = 10.0f; float total_fuel = 0.0f; float o2_trim = 1.00f;
uint8_t error_registry = ERR_NONE; 

// --- ФУНКЦИИ ТАРИРОВКИ И ИНТЕРПОЛЯЦИИ ---

// Линейная тарировка ДАД (Попугаи АЦП 0-4095 -> кПа)
int convert_adc_to_kpa(int adc_value) {
    int adc_min = 400; int adc_max = 3600; 
    if (adc_value <= adc_min) return 20;   
    if (adc_value >= adc_max) return 100;  
    return 20 + (100 - 20) * (adc_value - adc_min) / (adc_max - adc_min); 
}

// Линейная тарировка ДПДЗ (Попугаи АЦП -> % открытия заслонки)
int convert_adc_to_tps(int adc_value) {
    int adc_min = 200; int adc_max = 3900; 
    if (adc_value <= adc_min) return 0;    
    if (adc_value >= adc_max) return 100;  
    return (adc_value - adc_min) * 100 / (adc_max - adc_min); 
}

// Тарировка ДТОЖ (Нелинейный перевод по таблице для терморезистора ВАЗ)
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

// Линейная интерполяция мёртвого времени форсунки по напряжению АКБ
float get_interpolated_dead_time(float voltage) {
    if (voltage <= 10.0f) return 1.60f; 
    if (voltage >= 14.0f) return 0.80f; 
    float factor = (voltage - 10.0f) / (14.0f - 10.0f);
    return 1.60f + factor * (0.80f - 1.60f);
}

// Расчет коэффициента прогрева по температуре ДТОЖ
float get_warmup_coefficient(int temp) {
    if (temp <= -10) return 1.50f; 
    if (temp >= 80)  return 1.00f;  
    float factor = (float)(temp - (-10)) / (80 - (-10)); 
    return 1.50f - factor * (1.50f - 1.00f);
}

// Универсальная билинейная интерполяция для карт 8х8
float get_interpolated_value(const float map[MAP_RPM_SIZE][MAP_LOAD_SIZE], int current_rpm, int current_load) {
    int r1 = 0, r2 = 0, l1 = 0, l2 = 0;
    if (current_rpm < RPM_AXIS[0]) current_rpm = RPM_AXIS[0];
    if (current_rpm > RPM_AXIS[MAP_RPM_SIZE-1]) current_rpm = RPM_AXIS[MAP_RPM_SIZE-1];
    if (current_load < LOAD_AXIS[0]) current_load = LOAD_AXIS[0];
    if (current_load > LOAD_AXIS[MAP_LOAD_SIZE-1]) current_load = LOAD_AXIS[MAP_LOAD_SIZE-1];

    for (int i = 0; i < MAP_RPM_SIZE - 1; i++) { if (current_rpm >= RPM_AXIS[i] && current_rpm <= RPM_AXIS[i+1]) { r1 = i; r2 = i + 1; break; } }
    for (int j = 0; j < MAP_LOAD_SIZE - 1; j++) { if (current_load >= LOAD_AXIS[j] && current_load <= LOAD_AXIS[j+1]) { l1 = j; l2 = j + 1; break; } }

    float rpm_factor = (float)(current_rpm - RPM_AXIS[r1]) / (RPM_AXIS[r2] - RPM_AXIS[r1]);
    float load_factor = (float)(current_load - LOAD_AXIS[l1]) / (LOAD_AXIS[l2] - LOAD_AXIS[l1]);

    float q11 = map[r1][l1]; float q21 = map[r2][l1]; float q12 = map[r1][l2]; float q22 = map[r2][l2];

    float r1_interp = q11 + rpm_factor * (q21 - q11);
    float r2_interp = q12 + rpm_factor * (q22 - q12);
    return r1_interp + load_factor * (r2_interp - r1_interp); 
}

// Аппаратная запись кода ошибки во внешнюю EEPROM по шине I2C
void write_error_to_eeprom(uint8_t error_code) {
    HAL_I2C_Mem_Write(&hi2c1, EEPROM_I2C_ADDRESS, EEPROM_LOG_CELL, I2C_MEMADD_SIZE_16BIT, &error_code, 1, 100);
    HAL_Delay(5); // Пауза для завершения физического цикла записи памяти
}

// Новая функция: плавно вычисляет время накопления под текущие Вольты сети
float get_interpolated_dwell_time(float voltage) {
    if (voltage <= DWELL_VOLT_AXIS[0]) return DWELL_TIME_AXIS[0]; // Меньше 10В -> даем максимум 5 мс
    if (voltage >= DWELL_VOLT_AXIS[DWELL_POINTS-1]) return DWELL_TIME_AXIS[DWELL_POINTS-1]; // Больше 16В -> зажимаем до 2.4 мс

    for (int i = 0; i < DWELL_POINTS - 1; i++) {
        if (voltage >= DWELL_VOLT_AXIS[i] && voltage <= DWELL_VOLT_AXIS[i+1]) {
            // Пропорция линейной интерполяции
            float factor = (voltage - DWELL_VOLT_AXIS[i]) / (DWELL_VOLT_AXIS[i+1] - DWELL_VOLT_AXIS[i]);
            return DWELL_TIME_AXIS[i] + factor * (DWELL_TIME_AXIS[i+1] - DWELL_TIME_AXIS[i]);
        }
    }
    return 3.50f; // Резервное значение
}

int main(void)
{
  // Автоматические стартовые настройки железа от CubeMX
  HAL_Init(); SystemClock_Config(); MX_GPIO_Init(); MX_DMA_Init(); MX_ADC1_Init(); MX_TIM1_Init(); MX_I2C1_Init();  

  // Запуск фонового сбора 7 датчиков через робота DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buffer, 7);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Старт ШИМ форсунки на ножке PA8

  // Инициализация графического экрана 1.3"
  u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_cb_hw_i2c, &hi2c1);
  u8g2_InitDisplay(&u8g2); u8g2_SetPowerSave(&u8g2, 0); 

  char str_rpm[16]; char str_fuel[16]; char str_uoz[16];
  int main_injector_fault = 0; // Флаг исправности форсунки №1 (0-ок, 1-сбой)

  // ========================================================
  // БЕСКОНЕЧНЫЙ ЦИКЛ ОБРАБОТКИ ФИЗИКИ ДВС И БЕЗОПАСНОСТИ КАТЕРА
  // ========================================================
  while (1)
  {
    /* USER CODE BEGIN WHILE */

    // --- ШАГ 1: ВЫТАЛЬКИВАНИЕ СВЕЖИХ ЗАМЕРОВ АЦП ИЗ ОПЕРАТИВКИ ---
    uint16_t adc_dad_manifold = adc_raw_buffer[0]; // ДАД во впускном коллекторе
    uint16_t adc_dad_ambient  = adc_raw_buffer[1]; // ДАД в атмосфере (Барокоррекция)
    uint16_t adc_tps          = adc_raw_buffer[2]; // ДПДЗ положения заслонки
    uint16_t adc_dt           = adc_raw_buffer[3]; // ДТОЖ температуры двигателя
    uint16_t adc_dk           = adc_raw_buffer[4]; // ДК Лямбда-зонда (0.1В - бедно, 0.9В - богато)
    uint16_t adc_knock        = adc_raw_buffer[5]; // ДД микрофон детонации
    uint16_t adc_bat          = adc_raw_buffer[6]; // Вольтметр бортовой сети

    // Переводим попугаи АЦП в реальное напряжение АКБ с учетом резисторного делителя
    battery_voltage = (float)adc_bat * (3.3f / 4095.0f) * 4.0f; 

    // Проверяем электрическую исправность датчиков
    int dad_man_ok = (adc_dad_manifold > 300 && adc_dad_manifold < 3900);
    int dad_amb_ok = (adc_dad_ambient > 300 && adc_dad_ambient < 3900);
    int tps_ok     = (adc_tps > 200 && adc_tps < 4000);

    // --- ШАГ 2: СИСТЕМА ДИАГНОСТИКИ ОБД И СТРАТЕГИЯ ВЫЖИВАНИЯ (Limp Home) ---
    uint8_t old_errors = error_registry; // Сохраняем предыдущий статус ошибок
    error_registry = ERR_NONE;           // Обнуляем перед новой проверкой

    if (!dad_man_ok) { error_registry |= ERR_DAD_MANIFOLD; }
    if (!dad_amb_ok) { error_registry |= ERR_DAD_AMBIENT; }
    if (!tps_ok) { error_registry |= ERR_TPS_BROKEN; }
if (main_injector_fault == 1) { error_registry |= ERR_INJECTOR_1; }
// АВТО-ЛОГГЕР: Выжигаем в EEPROM только если сбой СВЕЖИЙ
if (error_registry != old_errors && error_registry != ERR_NONE) {
write_error_to_eeprom(error_registry);
}
// Выбор аварийных режимов расчета давления
if (dad_man_ok && dad_amb_ok) {
// РЕЖИМ МАКСИМАЛЬНОЙ ТОЧНОСТИ: оба датчика живы, считаем ВАШ честный перепад барокоррекции!
p_manifold = convert_adc_to_kpa(adc_dad_manifold);
p_ambient = convert_adc_to_kpa(adc_dad_ambient);
p_delta = p_ambient - p_manifold; // Нагрузка на ДВС, независимая от высоты в горах
if (p_delta < 0) p_delta = 0;
}
else if (!dad_man_ok && tps_ok) {
// Умер ДАД коллектора: аварийный расчет по ДПДЗ (Alpha-N)
tps = convert_adc_to_tps(adc_tps);
p_ambient = dad_amb_ok ? convert_adc_to_kpa(adc_dad_ambient) : 100;
p_manifold = 20 + (tps * 0.8f);
p_delta = p_ambient - p_manifold;
if (p_delta < 0) p_delta = 0;
}
else {
// Полный апокалипсис: ослепли оба ДАД. Фиксируем безопасный средний перепад
p_delta = 60; p_manifold = 40; tps = 20;
}
temperature = convert_adc_to_temp(adc_dt); // Переводим показания температуры в градусы °C
// --- ШАГ 3: ЛОГИКА ДИНАМИЧЕСКОГО ОТСКОКА ПО ДЕТОНАЦИИ (Пьезо-ДД) ---
if (adc_knock > KNOCK_THRESHOLD) {
knock_correction -= KNOCK_RETARD;
if (knock_correction < -12.0f) knock_correction = -12.0f; // Ограничение отскока угла
}
else {
knock_correction += KNOCK_RECOVERY;
if (knock_correction > 0.0f) knock_correction = 0.0f;
}
// --- ШАГ 4: МАТЕМАТИКА ИНТЕРПОЛЯЦИИ КАРТ 8х8 ---
// Нагрузочную ось LOAD_AXIS кормим вашим точным барокорректированным перепадом p_delta!
ve_coefficient = get_interpolated_value(VE_MAP, rpm, p_delta);
float base_uoz = get_interpolated_value(IGN_MAP, rpm, p_delta);
uoz = base_uoz + knock_correction; // Итоговый УОЗ с защитой от детонации
// --- ШАГ 5: СИСТЕМА ОБРАТНОЙ СВЯЗИ ПО ДАТЧИКУ КИСЛОРОДА (Лямбда-регулирование) ---
// Работает только на прогретом моторе и НЕ на полном дросселе (не "тапка в пол")
if (temperature >= 70 && tps < 80) {
float dk_voltage = (float)adc_dk * (5.0f / 4095.0f); // Переводим АЦП в Вольты ДК
if (dk_voltage < 0.4f) {
o2_trim += 0.01f; if (o2_trim > 1.20f) o2_trim = 1.20f; // Бедно -> обогащаем смесь (макс +20%)
} else if (dk_voltage > 0.5f) {
o2_trim -= 0.01f; if (o2_trim < 0.80f) o2_trim = 0.80f; // Богато -> обедняем смесь (макс -20%)
}
} else {
o2_trim = 1.00f; // В режиме прогрева или полной мощности лямбда-петля разомкнута
}
// --- ШАГ 6: ВЗРОСЛЫЙ ФИЗИЧЕСКИЙ РАСЧЕТ ТОПЛИВА С КОНСТАНТОЙ ФОРСУНКИ ---
// Формула Идеального Газа: Базовое время впрыска привязано к производительности форсунки
base_fuel = (p_manifold * 180.0f) / (INJECTOR_FLOW / 60.0f);
float warmup_coeff = get_warmup_coefficient(temperature); // Коррекция "электронного подсоса"
float dead_time = get_interpolated_dead_time(battery_voltage); // Плавный интерполированный лаг форсунки от вольтажа
// Итоговая длительность впрыска топлива (миллисекунды)
total_fuel = (base_fuel * ve_coefficient * warmup_coeff * o2_trim) + dead_time;
// Режимы отсечек по топливу
if (rpm >= REV_LIMIT || (tps == 0 && rpm > 1500)) { total_fuel = 0.0f; } // Жесткая отсечка или ПХХ

// --- ОБНОВЛЕННЫЙ ШАГ 7: УПРАВЛЕНИЕ ИСКРОЙ С ДИНАМИЧЕСКИМ НАКОПЛЕНИЕМ ---
    
    // 1. Проверяем напряжение сети через наш вольтметр и плавно рассчитываем идеальное время накопления!
    // Если в сети 10.5В (крутим стартер) — функция вернет около 4.5 мс. Если 14.2В (идем на глиссере) — вернет около 2.9 мс.
    float current_dwell_ms = get_interpolated_dwell_time(battery_voltage); 

    // 2. Переводим миллисекунды в микросекундные тики таймера (1 мс = 100 тиков)
    uint32_t dwell_ticks = (uint32_t)(current_dwell_ms * 100.0f); 

    // Проверяем фазу коленвала по ДПКВ
    if (current_cylinder_pair == 14) { 
        // --- КАТУШКА А (Цилиндры 1 и 4) ---
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // Подали ток в коммутатор А
        HAL_Delay_us(dwell_ticks * 10);                       // Выдерживаем строго рассчитанное время накопления
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // БУМ! Искра идеальной мощности!
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); 
    } 
    else if (current_cylinder_pair == 23) { 
        // --- КАТУШКА Б (Цилиндры 2 и 3) ---
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // Подали ток в коммутатор Б
        HAL_Delay_us(dwell_ticks * 10);                       // Выдерживаем время накопления
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // БУМ! Искра идеальной мощности!
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); 
    }

// --- ШАГ 8: УМНЫЙ OLED ДИСПЛЕЙ КАТЕРА (ПРИБОРКА И ЭКРАН СБОЕВ OBD) ---
u8g2_ClearBuffer(&u8g2); u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
if (error_registry == ERR_NONE) {
// Всё исправно — выводим параметры лодки
sprintf(str_rpm, "RPM: %d", rpm);
sprintf(str_fuel, "INJ: %.2f ms", total_fuel);
sprintf(str_uoz, "ADV: %.1f deg", uoz);
u8g2_DrawStr(&u8g2, 5, 15, "- MOTOR RUNNING -");
u8g2_DrawStr(&u8g2, 5, 32, str_rpm);
u8g2_DrawStr(&u8g2, 5, 48, str_fuel);
u8g2_DrawStr(&u8g2, 5, 64, str_uoz);
}
else {
// Авария! Экран выдает точные текстовые коды неисправностей водителю
u8g2_DrawStr(&u8g2, 5, 15, "⚠️ CHECK ENGINE");
u8g2_SetFont(&u8g2, u8g2_font_6x10_tf); int y = 28;
if (error_registry & ERR_DAD_MANIFOLD) { u8g2_DrawStr(&u8g2, 5, y, "FAIL 01: MANIFOLD MAP"); y += 10; }
if (error_registry & ERR_DAD_AMBIENT) { u8g2_DrawStr(&u8g2, 5, y, "FAIL 02: AMBIENT MAP"); y += 10; }
if (error_registry & ERR_TPS_BROKEN) { u8g2_DrawStr(&u8g2, 5, y, "FAIL 04: TPS SENSOR"); y += 10; }
if (error_registry & ERR_INJECTOR_1) { u8g2_DrawStr(&u8g2, 5, y, "FAIL 08: INJECTOR 1 CLOG"); y += 10; }
sprintf(str_fuel, "LIMP MODE. INJ %d", active_injector);
u8g2_DrawStr(&u8g2, 5, 64, str_fuel);
}
u8g2_SendBuffer(&u8g2); // Отправка графики на экран SH1106 (ножки PB8/PB9)
// --- ШАГ 9: СБРОС СТОРОЖЕВОГО ПСА (WATCHDOG) ---
// Если ядро зависнет, пес сделает RESET через 50 мс
HAL_IWDG_Refresh(&hiwdg);
HAL_Delay(100); // Такт обновления расчетов ЭБУ
/* USER CODE END WHILE */
}
}
