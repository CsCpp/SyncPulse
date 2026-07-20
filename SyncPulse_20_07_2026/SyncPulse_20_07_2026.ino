#include <EEPROM.h>
#include <U8glib.h>
#include "rus8x13B.h"
#include "rus7x13B.h"

/// <summary>
/// Перечисление всех возможных состояний (режимов работы) интерфейса и устройства.
/// </summary>
enum AppMode { 
  MODE_MAIN_SCREEN,     // Главный экран (настройка параметров импульсов)
  MODE_SETTINGS_MENU,   // Меню системных настроек
  MODE_EDIT_SHIFT,      // Режим редактирования программного сдвига детектора нуля
  MODE_SLOT_SAVE,       // Режим сохранения профиля сварки в EEPROM
  MODE_SLOT_LOAD,       // Режим загрузки профиля сварки из EEPROM
  MODE_WELDING,         // Активный процесс сварки (интерфейс блокируется)
  MODE_ERROR            // Экран критической ошибки (нет сигнала детектора нуля)
};

/// <summary>
/// Структура для хранения всех пользовательских настроек профиля сварки.
/// </summary>
struct WeldingConfig {
  uint8_t prePower;       // Мощность предварительного импульса (в процентах: 30-99)
  uint8_t preTime;        // Длительность предварительного импульса (в миллисекундах)
  uint8_t mainPower;      // Мощность основного импульса (в процентах: 30-99)
  uint8_t mainTime;       // Длительность основного импульса (в миллисекундах)
  uint8_t pauseTime;      // Длительность паузы между импульсами (в миллисекундах)
  uint8_t zeroCrossShift; // Корректирующий сдвиг фазы детектора (0-100%, где 100 = 10000 мкс)
};

// ==============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ И ОБЪЕКТЫ
// ==============================================================================

// Текущая конфигурация настроек, загруженная в оперативную память
WeldingConfig currentConfig = {40, 20, 80, 50, 50, 76}; 

// Состояние интерфейса
AppMode currentAppMode = MODE_MAIN_SCREEN; // Текущий активный экран
uint8_t selectedMainField = 0;             // Индекс выбранного поля на главном экране (0-4)
uint8_t selectedMenuItem = 0;              // Индекс выбранного пункта в меню настроек (0-3)
uint8_t selectedMemorySlot = 1;            // Выбранная ячейка памяти EEPROM для сохранения/загрузки (1-5)
bool isScreenUpdateRequired = true;        // Флаг необходимости перерисовки содержимого дисплея

// Переменные для работы с энкодером (помечены volatile, т.к. изменяются в прерывании)
volatile bool isEncoderChanged = false;    // Флаг регистрации вращения энкодера
volatile int encoderDirection = 0;         // Направление вращения энкодера (1 - вправо, -1 - влево)
int encoderStepAccumulator = 0;            // Накопитель шагов энкодера для реализации делителя "2 щелчка = 1 шаг"

// Переменные кнопок и таймеров
uint8_t currentEncoderBtnState = HIGH;     // Текущее логическое состояние кнопки энкодера
uint8_t previousEncoderBtnState = HIGH;    // Предыдущее логическое состояние кнопки энкодера для отслеживания фронтов
unsigned long encoderBtnPressStartTime = 0;// Метка времени (millis) начала нажатия кнопки энкодера
unsigned long pedalPressStartTime = 0;     // Метка времени (millis) начала нажатия педали сварки
unsigned long fanActivationTime = 0;       // Метка времени (millis) последнего включения вентилятора
bool isPedalProcessed = false;             // Флаг, предотвращающий повторное срабатывание педали до ее отпускания
unsigned int zeroCrossThreshold = 500;     // Вычисленное пороговое значение АЦП для перехода через ноль

// Назначение аппаратных выводов (пинов)
const uint8_t PIN_SENSOR_ZERO_CROSS = A6;  // Аналоговый пин для чтения сигнала детектора нуля
const uint8_t PIN_WELDING_TRIAC = 10;      // Цифровой вывод для управления оптомистором (силовой частью)
const uint8_t PIN_COOLING_FAN = 9;         // Цифровой вывод для включения вентилятора охлаждения
const uint8_t PIN_WELD_PEDAL = 8;          // Цифровой вход для педали/кнопки запуска сварки
const uint8_t PIN_BEEPER = 5;              // Цифровой вывод для управления звуковым излучателем (пищалка)
const uint8_t PIN_LED_GREEN = 6;           // Вывод управления зеленым светодиодом (индикатор работы)
const uint8_t PIN_LED_BLUE = 7;            // Вывод управления синим светодиодом (индикатор готовности)
const uint8_t PIN_ENCODER_A = 3;           // Канал A энкодера (подключен к аппаратному прерыванию INT1)
const uint8_t PIN_ENCODER_B = 2;           // Канал B энкодера
const uint8_t PIN_ENCODER_BUTTON = 4;      // Кнопка, встроенная в энкодер

// Системные константы
const unsigned long WELD_DEBOUNCE_MS = 20; // Время антидребезга для педали сварки (в миллисекундах)
const unsigned long FAN_TIMEOUT_MS = 60000;// Время автоматического отключения вентилятора (60 секунд)

// Объект дисплея SSD1306 (подключение по I2C)
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

// ==============================================================================
// ПРОТОТИПЫ И РЕАЛИЗАЦИИ ФУНКЦИЙ
// ==============================================================================

/// <summary>
/// Безопасная функция микросекундной задержки.
/// Стандартная функция Arduino delayMicroseconds() переполняется и работает некорректно 
/// при значениях больше 16383. Данная функция разбивает крупные задержки для безопасности.
/// </summary>
/// <param name="us">Требуемое время задержки в микросекундах.</param>
void delaySafeMicroseconds(unsigned int us) {
  if (us > 15000) { 
    delayMicroseconds(10000); 
    delayMicroseconds(us - 10000); 
  } else { 
    delayMicroseconds(us); 
  }
}

/// <summary>
/// Обработчик аппаратного прерывания (ISR) для энкодера.
/// Считывает состояние регистров порта D, определяет направление вращения 
/// и выставляет флаги для последующей обработки в главном цикле.
/// </summary>
void handleEncoderInterrupt() {
  static uint8_t oldState = 0;
  // Прямое чтение регистров для максимального быстродействия
  uint8_t pinA = (PIND & 0x08) >> 3; // Чтение D3
  uint8_t pinB = (PIND & 0x04) >> 2; // Чтение D2
  uint8_t currentState = (pinA << 1) | pinB;
  
  if (oldState != currentState) {
    if ((pinA ^ pinB)) {
      encoderDirection = 1; 
    } else {
      encoderDirection = -1; 
    }
    isEncoderChanged = true; 
    oldState = currentState;
  }
}

/// <summary>
/// Основной рабочий процесс сварки.
/// Отвечает за жесткие тайминги, фазовое управление симистором и 
/// формирование предварительного и основного импульсов.
/// </summary>
void executeWeldingCycle() {
  // Отключаем прерывания энкодера, чтобы они не сбивали жесткие тайминги сварки
  detachInterrupt(1); 
  
  // Компенсируем задержку аппаратного детектора нуля
  unsigned int shiftUs = (unsigned int)currentConfig.zeroCrossShift * 100;
  delaySafeMicroseconds(shiftUs);

  // Лямбда-функция для генерации пачки импульсов заданного времени и мощности
  auto runPulse = [&](uint8_t power, uint8_t timeMs) {
    int cycles = timeMs / 10; // Вычисление количества 10-миллисекундных полупериодов
    unsigned int targetDelayUs = map(power, 30, 99, 7000, 500);
    
    // Динамическая длительность управляющего импульса: <50% = 1мс, >=50% = 2мс
    unsigned int pulseWidthUs = (power < 50) ? 1000 : 2000; 
    unsigned int restTimeUs = 10000 - targetDelayUs - pulseWidthUs;

    for (int h = 0; h < cycles; h++) {
      delaySafeMicroseconds(targetDelayUs);      
      digitalWrite(PIN_WELDING_TRIAC, HIGH);  
      delayMicroseconds(pulseWidthUs); 
      digitalWrite(PIN_WELDING_TRIAC, LOW);   
      delaySafeMicroseconds(restTimeUs);         
    }
  };

  // 1. Отработка предварительного импульса
  if (currentConfig.preTime > 0) {
    runPulse(currentConfig.prePower, currentConfig.preTime); 
  }
  
  // 2. Отработка паузы (строго кратно 10мс, чтобы не сбить фазу)
  if (currentConfig.pauseTime > 0) {
    int pauseCycles = currentConfig.pauseTime / 10;
    for (int p = 0; p < pauseCycles; p++) {
      delaySafeMicroseconds(10000);
    }
  }
  
  // 3. Отработка основного сварочного импульса
  if (currentConfig.mainTime > 0) {
    runPulse(currentConfig.mainPower, currentConfig.mainTime);
  }
  
  // Восстанавливаем работу энкодера
  attachInterrupt(1, handleEncoderInterrupt, CHANGE);
}

/// <summary>
/// Функция отрисовки графического пользовательского интерфейса.
/// Формирует изображение на экране в зависимости от текущего режима работы.
/// </summary>
void renderScreen() {
  u8g.firstPage();
  do {
    if (currentAppMode == MODE_MAIN_SCREEN) {
      // Отрисовка сетки главного экрана
      u8g.drawHLine(0, 45, 128); 
      u8g.drawVLine(64, 0, 45); 
      u8g.drawVLine(42, 45, 19); 
      u8g.drawVLine(85, 45, 19); 
      
      // Вывод значений мощности крупным шрифтом
      u8g.setFont(u8g_font_fub30n);
      u8g.setPrintPos(3, 38); u8g.print(currentConfig.prePower);
      u8g.setPrintPos(67, 38); u8g.print(currentConfig.mainPower);
      
      // Вывод символов процента
      u8g.setFont(rus8x13B);
      u8g.setPrintPos(48, 38); u8g.print("%"); 
      u8g.setPrintPos(112, 38); u8g.print("%");
      
      // Вывод значений времени
      u8g.setFont(rus7x13B);
      u8g.setPrintPos(3, 62); u8g.print(currentConfig.preTime); u8g.print("мс");
      u8g.setPrintPos(45, 62); u8g.print(currentConfig.pauseTime); u8g.print("мс");
      u8g.setPrintPos(88, 62); u8g.print(currentConfig.mainTime); u8g.print("мс");
      
      // Отрисовка рамки выделения активного поля
      switch(selectedMainField) {
        case 0: u8g.drawFrame(0, 0, 64, 46); break; 
        case 1: u8g.drawFrame(64, 0, 64, 46); break;
        case 2: u8g.drawFrame(0, 45, 43, 19); break; 
        case 3: u8g.drawFrame(42, 45, 44, 19); break; 
        case 4: u8g.drawFrame(85, 45, 43, 19); break; 
      }
    } 
    else if (currentAppMode == MODE_ERROR) {
      // Экран критической ошибки аппаратной части
      u8g.setFont(rus8x13B);
      u8g.drawStr(10, 20, "ОШИБКА!");
      u8g.drawStr(10, 40, "Нет сигнала 0");
      u8g.drawStr(10, 55, "Нажми кнопку...");
    }
    else if (currentAppMode == MODE_SETTINGS_MENU || currentAppMode == MODE_EDIT_SHIFT) {
      // Экран системных настроек
      u8g.setFont(rus8x13B);
      u8g.drawStr(5, 15, selectedMenuItem == 0 ? "> Сдвиг:" : "  Сдвиг:");
      u8g.setPrintPos(70, 15); u8g.print(currentConfig.zeroCrossShift);
      if (currentAppMode == MODE_EDIT_SHIFT) {
        u8g.drawFrame(65, 3, 32, 15); 
      }
      
      u8g.drawStr(5, 30, selectedMenuItem == 1 ? "> Сохранить" : "  Сохранить");
      u8g.drawStr(5, 45, selectedMenuItem == 2 ? "> Загрузить" : "  Загрузить");
      u8g.drawStr(5, 60, selectedMenuItem == 3 ? "> Выход" : "  Выход");
    } 
    else if (currentAppMode == MODE_SLOT_SAVE || currentAppMode == MODE_SLOT_LOAD) {
      // Экран выбора ячейки памяти EEPROM
      u8g.setFont(rus8x13B);
      u8g.drawStr(10, 20, currentAppMode == MODE_SLOT_SAVE ? "Сохранить:" : "Загрузить:");
      u8g.drawStr(30, 40, "Ячейка "); 
      u8g.setPrintPos(90, 40); u8g.print(selectedMemorySlot);
      u8g.drawFrame(85, 28, 20, 15);
    } 
    else if (currentAppMode == MODE_WELDING) {
      // Заглушка, отображаемая во время активной сварки
      u8g.setFont(rus7x13B);
      u8g.setScale2x2(); 
      u8g.drawStr(3, 20, "*Сварка*");
      u8g.undoScale();
    }
  } while( u8g.nextPage() );
}

/// <summary>
/// Инициализация аппаратной части микроконтроллера при включении.
/// Настройка портов ввода/вывода, прерываний, параметров АЦП и загрузка данных из EEPROM.
/// </summary>
void setup() {
  pinMode(PIN_WELDING_TRIAC, OUTPUT); digitalWrite(PIN_WELDING_TRIAC, LOW);
  pinMode(PIN_COOLING_FAN, OUTPUT);   digitalWrite(PIN_COOLING_FAN, LOW);
  pinMode(PIN_BEEPER, OUTPUT);        digitalWrite(PIN_BEEPER, LOW);
  pinMode(PIN_WELD_PEDAL, INPUT); 
  pinMode(PIN_LED_GREEN, OUTPUT);     digitalWrite(PIN_LED_GREEN, LOW); 
  pinMode(PIN_LED_BLUE, OUTPUT);      digitalWrite(PIN_LED_BLUE, HIGH);
  
  pinMode(PIN_ENCODER_A, INPUT_PULLUP); 
  pinMode(PIN_ENCODER_B, INPUT_PULLUP); 
  pinMode(PIN_ENCODER_BUTTON, INPUT_PULLUP);
  
  // Назначаем прерывание 1 (что соответствует пину D3 на Arduino Uno/Nano)
  attachInterrupt(1, handleEncoderInterrupt, CHANGE); 

  // Загрузка первичных настроек из 0-й ячейки EEPROM
  EEPROM.get(0, currentConfig);
  
  // Проверка целостности данных. Если прочитан мусор - восстанавливаем дефолт.
  if (currentConfig.prePower < 30 || currentConfig.prePower > 99 || currentConfig.preTime < 20 || currentConfig.preTime > 240) { 
    WeldingConfig defaultConfig = {40, 20, 80, 50, 50, 76}; 
    currentConfig = defaultConfig; 
    EEPROM.put(0, currentConfig); 
  }
  
  // Ускорение работы АЦП для более частой оцифровки синусоиды детектора нуля
  ADCSRA = (ADCSRA & 0xF8) | 0x04; 
}

/// <summary>
/// Главный бесконечный цикл программы.
/// Обрабатывает пользовательский ввод (энкодер, кнопка, педаль), управляет экранами 
/// и запускает алгоритм сварки при наступлении условий.
/// </summary>
void loop() {
  unsigned long currentMillis = millis();

  // Логика прерывистого звукового сигнала при ошибке синхронизации фазы
  if (currentAppMode == MODE_ERROR) {
      if ((currentMillis / 150) % 2 == 0) {
        digitalWrite(PIN_BEEPER, HIGH);
      } else {
        digitalWrite(PIN_BEEPER, LOW);
      }
  }

  // --- ОБРАБОТКА ВРАЩЕНИЯ ЭНКОДЕРА ---
  if (isEncoderChanged) {
    encoderStepAccumulator += encoderDirection;
    isEncoderChanged = false; 

    // Реализация делителя шагов (2 физических щелчка = 1 программный шаг)
    if (abs(encoderStepAccumulator) >= 2) {
      int effectiveDirection = (encoderStepAccumulator > 0) ? 1 : -1;
      encoderStepAccumulator = 0; 

      // Изменение параметров в зависимости от текущего активного экрана
      if (currentAppMode == MODE_MAIN_SCREEN) {
        switch(selectedMainField) {
          case 0: currentConfig.prePower = constrain(currentConfig.prePower + (effectiveDirection * 5), 30, 99); break;       
          case 1: currentConfig.mainPower = constrain(currentConfig.mainPower + (effectiveDirection * 5), 30, 99); break;    
          case 2: currentConfig.preTime = constrain(currentConfig.preTime + (effectiveDirection * 20), 20, 240); break;      
          case 3: currentConfig.pauseTime = constrain(currentConfig.pauseTime + (effectiveDirection * 20), 20, 240); break;  
          case 4: currentConfig.mainTime = constrain(currentConfig.mainTime + (effectiveDirection * 20), 20, 240); break;    
        }
      } 
      else if (currentAppMode == MODE_SETTINGS_MENU) {
        selectedMenuItem = constrain(selectedMenuItem + effectiveDirection, 0, 3);
      }
      else if (currentAppMode == MODE_EDIT_SHIFT) {
        currentConfig.zeroCrossShift = constrain(currentConfig.zeroCrossShift + effectiveDirection, 0, 100);
      }
      else if (currentAppMode == MODE_SLOT_SAVE || currentAppMode == MODE_SLOT_LOAD) {
        selectedMemorySlot = constrain(selectedMemorySlot + effectiveDirection, 1, 5);
      }
      
      isScreenUpdateRequired = true; 
    }
  }

  // --- ОБРАБОТКА КНОПКИ ЭНКОДЕРА ---
  currentEncoderBtnState = digitalRead(PIN_ENCODER_BUTTON);
  // Фиксация момента нажатия кнопки (переход HIGH -> LOW)
  if (currentEncoderBtnState == LOW && previousEncoderBtnState == HIGH) {
    encoderBtnPressStartTime = millis();
  }
  // Обработка логики при отпускании кнопки (переход LOW -> HIGH)
  if (currentEncoderBtnState == HIGH && previousEncoderBtnState == LOW) {
    unsigned long pressDuration = millis() - encoderBtnPressStartTime;
    
    // Если мы находимся в состоянии ошибки, любое нажатие сбрасывает ошибку
    if (currentAppMode == MODE_ERROR) {
        currentAppMode = MODE_MAIN_SCREEN;
        digitalWrite(PIN_BEEPER, LOW);
        isScreenUpdateRequired = true;
    } 
    // Короткое нажатие (от 20 до 1000 мс)
    else if (pressDuration > 20 && pressDuration < 1000) {
      if (currentAppMode == MODE_MAIN_SCREEN) { 
        selectedMainField++; 
        if (selectedMainField > 4) selectedMainField = 0; 
      } 
      else if (currentAppMode == MODE_SETTINGS_MENU) {
        if (selectedMenuItem == 0) currentAppMode = MODE_EDIT_SHIFT;
        else if (selectedMenuItem == 1) currentAppMode = MODE_SLOT_SAVE;
        else if (selectedMenuItem == 2) currentAppMode = MODE_SLOT_LOAD;
        else if (selectedMenuItem == 3) { 
          EEPROM.put(0, currentConfig); 
          currentAppMode = MODE_MAIN_SCREEN; 
        }
      } 
      else if (currentAppMode == MODE_EDIT_SHIFT) {
        currentAppMode = MODE_SETTINGS_MENU;
      }
      else if (currentAppMode == MODE_SLOT_SAVE) { 
        // Формула адресации ячеек: слот * размер_структуры(6 байт) * 2
        EEPROM.put(selectedMemorySlot * sizeof(WeldingConfig) * 2, currentConfig); 
        EEPROM.put(0, currentConfig); 
        currentAppMode = MODE_MAIN_SCREEN; 
      }
      else if (currentAppMode == MODE_SLOT_LOAD) { 
        EEPROM.get(selectedMemorySlot * sizeof(WeldingConfig) * 2, currentConfig); 
        EEPROM.put(0, currentConfig); 
        currentAppMode = MODE_MAIN_SCREEN; 
      }
    } 
    // Длинное нажатие (более 1 секунды) - вход в системное меню
    else if (pressDuration >= 1000 && currentAppMode == MODE_MAIN_SCREEN) { 
      currentAppMode = MODE_SETTINGS_MENU; 
      selectedMenuItem = 0; 
    }
    isScreenUpdateRequired = true;
  }
  previousEncoderBtnState = currentEncoderBtnState;

  // --- ОБРАБОТКА ТАЙМАУТА ВЕНТИЛЯТОРА ---
  if(fanActivationTime > 0 && (currentMillis - fanActivationTime) > FAN_TIMEOUT_MS) { 
    digitalWrite(PIN_COOLING_FAN, LOW); 
    fanActivationTime = 0; 
  }

  // --- ОБРАБОТКА ПЕДАЛИ/КНОПКИ СВАРКИ ---
  if(digitalRead(PIN_WELD_PEDAL) && !isPedalProcessed) {
    if(pedalPressStartTime == 0) pedalPressStartTime = currentMillis;
    
    // Проверка антидребезга педали
    if(currentMillis - pedalPressStartTime > WELD_DEBOUNCE_MS){
      
      // Включение индикации и охлаждения перед сваркой
      digitalWrite(PIN_LED_GREEN, HIGH); 
      digitalWrite(PIN_LED_BLUE, LOW); 
      digitalWrite(PIN_COOLING_FAN, HIGH); 
      fanActivationTime = currentMillis;
      
      AppMode previousMode = currentAppMode; 
      currentAppMode = MODE_WELDING; 
      renderScreen(); 
      isPedalProcessed = true;
      digitalWrite(PIN_BEEPER, HIGH);
      
      // Блок калибровки и проверки сигнала детектора нуля (100 мс)
      unsigned int sensorMin = 1023, sensorMax = 0; 
      unsigned long calibStart = millis();
      while (millis() - calibStart < 100) {
        unsigned int val = analogRead(PIN_SENSOR_ZERO_CROSS);
        if (val < sensorMin) sensorMin = val; 
        if (val > sensorMax) sensorMax = val;
      }
      digitalWrite(PIN_BEEPER, LOW);
      
      // Если амплитуда сигнала слишком мала - значит детектор отключен или неисправен
      if (sensorMax - sensorMin < 50) {
        currentAppMode = MODE_ERROR; 
        digitalWrite(PIN_LED_BLUE, HIGH); 
        digitalWrite(PIN_LED_GREEN, LOW); 
        isScreenUpdateRequired = true;
      } else {
          // Динамический расчет порога перехода через ноль
          zeroCrossThreshold = sensorMin + (sensorMax - sensorMin) / 2;
          
          // Жесткая синхронизация: ожидание смены полупериода
          bool initialPhaseState = (analogRead(PIN_SENSOR_ZERO_CROSS) > zeroCrossThreshold);
          while ((analogRead(PIN_SENSOR_ZERO_CROSS) > zeroCrossThreshold) == initialPhaseState);
          
          // Вызов основного процесса формирования импульсов
          executeWeldingCycle(); 
          
          // Восстановление состояния интерфейса после успешной сварки
          currentAppMode = previousMode; 
          digitalWrite(PIN_LED_BLUE, HIGH); 
          digitalWrite(PIN_LED_GREEN, LOW); 
          isScreenUpdateRequired = true;
      }
    }
  } 
  // Сброс флага педали после ее отпускания
  else if(!digitalRead(PIN_WELD_PEDAL)) {
    isPedalProcessed = false;
    pedalPressStartTime = 0; // Сбрасываем таймер нажатия
  }
  
  // Обновление экрана, если флаг перерисовки взведен
  if (isScreenUpdateRequired) { 
    renderScreen(); 
    isScreenUpdateRequired = false; 
  }
}