//#define TEST; // раскомментировать для запуска в тестовом режиме
bool debug = 0; // Serial.print если = 1
// ----------------------------------

byte br_min = 0; // яркость выключенного состояния
byte br_max = 255; // яркость включения в ручном режиме
byte br_half = 128 - 80; // яркость включения в авто режиме
byte dc_pwr = 12; // напряжение блока питания
byte led_pwr = 12; // напряжение светодиодной ленты

byte led_speed = 20; // плавность изменения яркости (чем больше значение, тем плавнее)
uint32_t speed_timer = 0; // вспомогательный таймер

byte led_pin = 3;
byte sensor_pin = 4;

bool cir = 0; // используется в тестовом режиме

byte br = 0;  // текущая яркость
byte br_target = 0; // назначенная яркость
bool led_state = false; // статус ленты (вкл/выкл)

bool btn_pressed = false; // признак нажатия кнопки
bool mode_auto = true; // режим автоматического управления с датчика движения
bool btn_block = false; // блокировка обработкки нажатия кнопки
uint32_t btn_timer = 0; // вспомогательный таймер
uint32_t btn_delay = 500; // после нажатия на кнопку ее обработка блокируется на это количество миллисекунд
int btn_value = 0; // значение кода кнопки

bool sensor_command = 0; // команда с датчика движения
uint32_t half_timer_start = 0; // вспомогательный таймер
uint32_t half_delay_on = 10000; // время непрерывного отсутствия команды от датчика движения (мс)

// ----------------------------------

#include <RCSwitch.h>
RCSwitch mySwitch = RCSwitch();

// ==========================================================================================
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  if (debug == 1)  Serial.begin(9600);

  pinMode(led_pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sensor_pin, INPUT);

#ifndef TEST
  if (debug == 1)Serial.println("test mode is off");

  for (int i = 0; i < 3; i++ )
  {
    while (br < br_half)
    {
      br++;
      analogWrite(led_pin, br_correct(br));
      delay(5);
    }

    while (br > br_min)
    {
      br--;
      analogWrite(led_pin, br_correct(br));
      delay(5);
    }
  }
  
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2

#else
  if (debug == 1) Serial.println("test mode is on");

#endif
}

// ==========================================================================================

void loop() {
#ifdef TEST  // ---------------------------------------------  ТЕСТОВЫЙ РЕЖИМ
  if (cir == false) {
    digitalWrite(LED_BUILTIN, HIGH);

    while (br < br_max) {
      br++;
      analogWrite(led_pin, br_correct(br));
      delay(10);
    }
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);

    while (br > br_min) {
      br--;
      analogWrite(led_pin, br_correct(br));
      delay(10);
    }
  }

  delay(2000);        // delay in between reads for stability
  cir = ! cir;

#else  // ---------------------------------------------------  БОЕВОЙ РЕЖИМ

  if (mySwitch.available())
  {
    btn_value = mySwitch.getReceivedValue();
    if (debug == 1) Serial.println("btn_value = " + String(btn_value));

    if (btn_block)
    {
      if (millis() - btn_timer > btn_delay)
      {
        btn_block = false;
      }
    }

    if (btn_block == false)
    {
      if (btn_value == 10434) // 600258
      {
        btn_block = true;
        btn_timer = millis();
        if (btn_pressed) btn_pressed = false;
        else btn_pressed = true;
        if (debug == 1) Serial.println("btn_pressed = " + String(btn_pressed));
      }
      else
      {
        if (debug == 1) Serial.println("btn_value = unknown");
      }

    }
    else
    {
      if (debug == 1) Serial.println("btn_block");
    }
    mySwitch.resetAvailable();
  }


  if (btn_pressed)
  {
    if (led_state == false || br_target == br_half)
    {
      led_state = true;
      br_target = br_max;
      mode_auto = false;
    }
    else
    {
      led_state = false;
      br_target = br_min;
      mode_auto = true;
    }
    btn_pressed = false;
    if (debug == 1) Serial.println("led_state = " + String(led_state));
    if (debug == 1) Serial.println("br_target = " + String(br_target));
    if (debug == 1) Serial.println("mode_auto = " + String(mode_auto));


  }

  /*
    if (mode_auto)
    {
      if (led_status == false)
      {

        led_status = true;

      }
    }
    }
    else
    {
    timer reset
    }
  */


  // Обработка датчикка движения ---------------------

  if (mode_auto == true)
  {
    sensor_command = digitalRead(sensor_pin);

    if (sensor_command == HIGH) // включаем сразу
    {
      br_target = br_half;
      led_state = true;
      half_timer_start = millis();
    }

    if (led_state == true && sensor_command == LOW) // выключаем только после задержки
    {
      if (millis() - half_timer_start > half_delay_on)
      {
        br_target = br_min;
        led_state = false;
      }
    }
  }


  // Управляем яркостью ----------------------------

  if (millis() - speed_timer >= led_speed)
  {
    speed_timer = millis();
    analogWrite(led_pin, br_correct(change_br()));
  }

  if (led_state == 0) digitalWrite(LED_BUILTIN, LOW);
  else digitalWrite(LED_BUILTIN, HIGH);

#endif

} // end loop



// ==========================================================================================

byte br_correct (byte val) // конвертирует значение яркости, когда напряжение блока питания выше напряжения ленты
{
  //map(val, 0, 255, 0, 255);
  //map(val, 0, dc_pwr, 0, dc_pwr);
  constrain(val, br_min, br_max);
  val = val * led_pwr / dc_pwr;
  //constrain(val, 0, 255);
  //if (debug == 1) Serial.println("br_correct = " + String(val));
  return (val);
}



byte change_br()
{
  if (br < br_target) br++;
  if (br > br_target) br--;
  //if (debug == 1) Serial.println("change_br = " + String(br));
  return (br);
}
