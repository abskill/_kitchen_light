//#define TEST; // раскомментировать для запуска в тестовом режиме

#define NANO; // Закомментировать для NodeMCU
bool debug = 0; // Serial.print если = 1

#ifdef NANO
#define main_led_pin 3 // пин управления основной лентой
#define second_led_pin 6 // пин управления дополнительной лентой
#define sensor_pin 4 // пин к датчику движения
#define light_pin 5 // пин к датчику света
#else
//...
#endif

// ----------------------------------

byte br_min = 0; // яркость выключенного состояния
byte br_max = 255; // яркость включения в ручном режиме
byte br_half = 8; // яркость включения в авто режиме
//byte dc_pwr = 9; // напряжение блока питания
//byte main_led_pwr = 9; // напряжение светодиодной ленты
//byte second_led_pwr = 5; // напряжение светодиодной ленты

byte led_speed1 = 20; // плавность изменения яркости (чем больше значение, тем плавнее)
byte led_speed2 = 60; // плавность изменения яркости (чем больше значение, тем плавнее)
byte led_speed = led_speed1; // плавность изменения яркости (чем больше значение, тем плавнее)
uint32_t speed_timer = 0; // вспомогательный таймер

byte second_led_speed = 20; // плавность изменения яркости (чем больше значение, тем плавнее)
uint32_t speed_timer2 = 0; // вспомогательный таймер

bool cir = 0; // используется в тестовом режиме

byte br = 0;  // текущая яркость
byte br_target = 0; // назначенная яркость
bool led_state = false; // статус ленты (вкл/выкл)

byte br2 = 0;  // текущая яркость
byte br2_target = 0; // назначенная яркость

bool btn_pressed = false; // признак нажатия кнопки
bool mode_auto = true; // режим автоматического управления с датчика движения
bool btn_block = false; // блокировка обработкки нажатия кнопки
uint32_t btn_timer = 0; // вспомогательный таймер
uint32_t btn_delay = 500; // после нажатия на кнопку ее обработка блокируется на это количество миллисекунд
int btn_value = 0; // значение кода кнопки

bool low_light = true; // уровень освещенности с датчика света
uint32_t delay_light = 3000; // для исключения обработки дребезга используется это время задержки перехода состояния low_light
uint32_t timer_light = 0; // вспомогательный таймер

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

  pinMode(main_led_pin, OUTPUT);
  pinMode(second_led_pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sensor_pin, INPUT);
  pinMode(light_pin, INPUT);

#ifndef TEST
  if (debug == 1) Serial.println("test mode is off");

  for (int i = 0; i < 2; i++ )
  {
    br = 0;
    while (br < 64)
    {
      br++;
      analogWrite(main_led_pin, br); //br_correct(br));
      analogWrite(second_led_pin, br);
      delay(5);
    }

    while (br > 0)
    {
      br--;
      analogWrite(main_led_pin, br); //br_correct(br));
      analogWrite(second_led_pin, br);
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
      analogWrite(main_led_pin, br); //br_correct(br));
      delay(10);
    }
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);

    while (br > br_min) {
      br--;
      analogWrite(main_led_pin, br); //br_correct(br));
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
    if (led_state == false || br_target == br_half) // включение
    {
      led_state = true;
      br_target = br_max;
      br2_target = br_max;
      mode_auto = false;
    }
    else // выклюючение
    {
      led_state = false;
      br_target = br_min;
      mode_auto = true;
      half_timer_start = millis();  // сброс таймера выключения в авторежиме
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

    if (digitalRead(light_pin) == low_light)  timer_light = millis();
    else
    {
      if (millis() - timer_light > delay_light)  low_light = ! low_light;
    }


    /*
      if (low_light == true && digitalRead(light_pin) == true)  timer_light = millis();
      else
      {
      if (millis() - timer_light > delay_light) low_light = false;
      //else low_light=true;
      }

      if (low_light == false && digitalRead(light_pin) == false)  timer_light = millis();
      else
      {
      if (millis() - timer_light > delay_light) low_light = true;
      //else low_light=false;
      }
    */



    if (sensor_command == true)  // включаем
    {
      br2_target = br_max;
      half_timer_start = millis();

      if (low_light == true) // если в комнате темно
      {
        br_target = br_half;
        led_state = true;
      }
    }

    if (sensor_command == LOW) // выключаем только после задержки
    {

      if (millis() - half_timer_start > half_delay_on)
      {
        br_target = br_min;
        //br2_target = br_min;
        led_state = false;
      }

      if (millis() - half_timer_start > half_delay_on * 1.2)
      {
        br2_target = br_min;
        //led_speed = led_speed1;
      }

    }
  }


  // Определяем быстроту изменения яркости --------

  if (abs(br - br_target) >= br_half)
  {
    if (br > br_target) {
      led_speed = led_speed1 / 3;
    } else {
      led_speed = led_speed1;
    }
  }
  else
  {
    //    led_speed = led_speed2;
    if (br > br_target) {
      led_speed = led_speed2 / 2;
    } else {
      led_speed = led_speed2;
    }

  }


  // Управляем яркостью ----------------------------

  if (millis() - speed_timer >= led_speed)
  {
    speed_timer = millis();

    br = change_br(br, br_target);
    analogWrite(main_led_pin, br); //br_correct(br));

    // br2 = change_br(br2, br2_target);
    // analogWrite(second_led_pin, br2);
  }

  if (millis() - speed_timer2 >= second_led_speed)
  {
    speed_timer2 = millis();

    //br = change_br(br, br_target);
    //analogWrite(main_led_pin, br); //br_correct(br));

    br2 = change_br(br2, br2_target);
    analogWrite(second_led_pin, br2);
  }


  if (led_state == 0) digitalWrite(LED_BUILTIN, LOW);
  else digitalWrite(LED_BUILTIN, HIGH);

#endif

} // end loop



// ==========================================================================================
/*
  byte br_correct (byte val) // конвертирует значение яркости, когда напряжение блока питания выше напряжения ленты
  {
  //map(val, 0, 255, 0, 255);
  //map(val, 0, dc_pwr, 0, dc_pwr);
  constrain(val, br_min, br_max);
  val = val * main_led_pwr / dc_pwr;
  //constrain(val, 0, 255);
  //if (debug == 1) Serial.println("br_correct = " + String(val));
  return (val);
  }
*/


byte change_br(byte brightness, byte brightness_target)
{
  if (brightness < brightness_target) brightness++;
  if (brightness > brightness_target) brightness--;
  if (debug == 1) Serial.println("change_br = " + String(brightness));
  return (brightness);
}
