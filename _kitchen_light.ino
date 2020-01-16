

#define nodeMCU; // Закомментировать, если сборка для NANO
bool debug = 0; // Serial.print если = 1


#ifndef nodeMCU // условная компиляция для Nano ----------

#define main_led_pin 3 // пин управления основной лентой
#define second_led_pin 6 // пин управления дополнительной лентой
#define sensor_pin 4 // пин к датчику движения
#define light_pin 5 // пин к датчику света

#define btn_code 10434 // код кнопки радиоканала

int br_max = 255; // яркость включения в ручном режиме
//byte led_speed1 = 20; // плавность изменения яркости (чем больше значение, тем плавнее)
//byte led_speed2 = 60; // плавность изменения яркости (чем больше значение, тем плавнее)
//byte second_led_speed = 20; // плавность изменения яркости (чем больше значение, тем плавнее)

#else // Условная компиляция для NodeMCU -----------------

bool OTA_on = false;
bool MQTT_on = false;
/*
  #define main_led_pin D1 // пин управления основной лентой
  #define second_led_pin D3 // пин управления дополнительной лентой
  #define sensor_pin D4 // пин к датчику движения
  #define light_pin D0 // пин к датчику света
*/

#define main_led_pin D5 //D8 //D4 //D5 // пин управления основной лентой
#define second_led_pin D6 // пин управления дополнительной лентой
#define sensor_pin D7 // пин к датчику движения
#define light_pin D1 // пин к датчику света

#define btn_code 600258 // код кнопки радиоканала

int br_max = 1023; // яркость включения в ручном режиме

//byte led_speed1 = 1; // плавность изменения яркости (чем больше значение, тем плавнее)
//byte led_speed2 = 100; // плавность изменения яркости (чем больше значение, тем плавнее)
//byte second_led_speed = 1; // плавность изменения яркости (чем больше значение, тем плавнее)

uint32_t t_min_max = 2000; // время изменения яркости от br_min до br_max (миллисекунды)
float br_step = 1; // начальное значение шага изменения яркости

uint32_t t2_min_max = 1500; // время изменения яркости от br_min до br_max (миллисекунды)
float br2_step = 1; // начальное значение шага изменения яркости


uint32_t mqtt_timer = 0; // вспомогательный таймер
uint32_t mqtt_delay = 500; // интервал обмена с MQTT сервером

#include <ESP8266WiFi.h> //Библиотека для работы с WIFI 
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

#include <ArduinoOTA.h> // Библиотека для OTA-прошивки

const char* ssid = "EnergaiZer"; //Имя точки доступа WIFI
const char* password = "ferromed"; //пароль точки доступа WIFI

#include <PubSubClient.h>
const char *mqtt_server = "m21.cloudmqtt.com"; // Имя сервера MQTT
const int mqtt_port = 16195; // Порт для подключения к серверу MQTT
const char *mqtt_user = "jrthidnj"; // Логи для подключения к серверу MQTT
const char *mqtt_pass = "5ZYKoip4ef_S"; // Пароль для подключения к серверу MQTT

#define BUFFER_SIZE 100

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);

int tm = 300;
float temp = 0;

//ADC_MODE(ADC_VCC); // A0 будет считытвать значение напряжения VCC


uint16_t delay_of_trying_to_connect_wf = 5000;
uint16_t timer_of_trying_to_connect_wf = millis();
bool wf_is_connected = false;
bool mqtt_is_connected = false;
bool wf_stop = false;


#endif

// ----------------------------------

byte br_min = 0; // яркость выключенного состояния
byte br_half = 13; // яркость включения в авто режиме

//byte led_speed1 = 20; // плавность изменения яркости (чем больше значение, тем плавнее)
//byte led_speed2 = 60; // плавность изменения яркости (чем больше значение, тем плавнее)
//byte led_speed = led_speed1; // плавность изменения яркости (чем больше значение, тем плавнее)
//uint32_t speed_timer = 0; // вспомогательный таймер

//byte second_led_speed = 20; // плавность изменения яркости (чем больше значение, тем плавнее)
//uint32_t speed_timer2 = 0; // вспомогательный таймер

//bool cir = 0; // используется в тестовом режиме

float br = 0;  // текущая яркость
int br_target = 0;  // назначенная яркость
bool led_state = false; // статус ленты (вкл/выкл)

float br2 = 0;  // текущая яркость
int br2_target = 0;  // назначенная яркость

bool btn_pressed = false;  // признак нажатия кнопки
bool mode_auto = true;  // режим автоматического управления с датчика движения
bool btn_block = false;  // блокировка обработкки нажатия кнопки
uint32_t btn_timer = 0;  // вспомогательный таймер

uint32_t btn_delay = 500;  // после нажатия на кнопку ее обработка блокируется на это количество миллисекунд
int btn_value = 0;  // значение кода кнопки

bool low_light = true;  // уровень освещенности с датчика света (нач.значение)

uint32_t delay_light = 1000;  // для исключения обработки дребезга используется это время задержки перехода состояния low_light
uint32_t timer_light = 0;  // вспомогательный таймер

//uint32_t delay_main_led = 1000;  // задержка включения основной ленты в авто режиме

bool sensor_command = 0;  // начальное значение команды с датчика движения

uint32_t half_delay_on = 10000;  // время непрерывного отсутствия команды от датчика движения (мс) main_led
uint32_t half_delay_on2 = 15000;  // время непрерывного отсутствия команды от датчика движения (мс) second_led
uint32_t half_timer_start = 0;  // вспомогательный таймер

uint32_t d_time = 0;  // длительность одно цикла (исп-ся для расчета br_step)
uint32_t time_old = millis();

// ----------------------------------

#include <RCSwitch.h>
RCSwitch mySwitch = RCSwitch();

// ==========================================================================================
// the setup routine runs once when you press reset:

void setup() {

  // устанавливаем режим пинов
  pinMode(main_led_pin, OUTPUT);
  pinMode(second_led_pin, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sensor_pin, INPUT);
  pinMode(light_pin, INPUT);

  // сигнализируем о включении питания
  analogWrite(main_led_pin, br_half);
  delay(500);
  analogWrite(main_led_pin, br_min);
  delay(500);
  analogWrite(main_led_pin, br_half);
  delay(500);
  analogWrite(main_led_pin, br_min);
  analogWrite(second_led_pin, br_min);
  delay(500);

#ifdef nodeMCU // Условная компиляция для NodeMCU --------
try_to_connect_wf(wf_is_connected);
/*
  // включаем wi-fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(5000);
    ESP.restart();
  }

  // подключаем возможность прошивки по воздуху
  ArduinoOTA.setHostname("ESP8266-00001"); // Задаем имя сетевого порта
  //ArduinoOTA.setPassword((const char *)"0000"); // Задаем пароль доступа для удаленной прошивки
  ArduinoOTA.begin(); // Инициализируем OTA
*/
#endif // ---------------------------------


  // initialize serial communication at 9600 bits per second:
  if (debug == 1)  Serial.begin(9600);


  // инициализируем приемник rf433
#ifdef nodeMCU
  mySwitch.enableReceive(4);  // Receiver on interrupt 4 => that is pin D2
#else
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin GPIO 2
#endif


}

// ==========================================================================================

void loop() {


#ifdef nodeMCU
   if (wf_stop==false) {
   if (wf_is_connected==false) {
     if ((millis()-timer_of_trying_to_connect_wf)>delay_of_trying_to_connect_wf) {
     // try_to_connect_wf(wf_is_connected);
     WiFi.mode(WIFI_STA);  WiFi.begin(ssid, password);  delay(500);  if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
    wf_is_connected=false;
    timer_of_trying_to_connect_wf=millis();
    }
    else
    {
    wf_is_connected=true;
    }
      }
      
      }
  
   if (wf_is_connected==true) {
   if (MQTT) {
   mqtt_call();
   }
 //  mqtt_read(OTA, MQTT) // прием с сервера
   if (OTA==flase && MQTT==false) {
    wf_stop=true;
    STOP_WF()
   }
   else {
    if (OTA) {
     ArduinoOTA.handle(); // ожидание старта прошивки
    }
   
    
   }
  } // if (!wf_stop)
#endif  
  
  
  // считываем код нажатой кнопки или 0
  if (mySwitch.available())
  {
    btn_value = mySwitch.getReceivedValue();
    mySwitch.resetAvailable();
    if (debug == 1) Serial.println("btn_value = " + String(btn_value));
  }
  else
  {
    btn_value = 0;
  }



  if (btn_value != 0) // если пришла команла о нажатии кнопки на пульте
  {
    if (btn_block) // если включена блокировка управления по кнопке
    {
      if (millis() - btn_timer > btn_delay) //если время блокировки истекло
      {
        btn_block = false; // снимаем блокировку
      }
    }

    if (btn_block == false) // блокировки кнопки нет
    {
      if (btn_value == btn_code) // если код кнопки верный
      {
        // маргнём second_led в качестве подтверждения
        if (br2 == br_min)  br2 = br_max - 1;
        else   br2 = br_min + 1;

        btn_block = true; // включаем блокировку кнопки
        btn_timer = millis(); // сбрасываем таймер блокировки
        if (btn_pressed) btn_pressed = false; // инвертируем статус признака нажатия кнопки
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
  }





  if (btn_pressed) // Если есть признак нажатия кнопки
  {
    if (led_state == false || br_target == br_half) // включение света
    {
      led_state = true;
      br_target = br_max;
      br2_target = br_max;
      mode_auto = false; // блокируем авто режим
    }
    else // выключение света
    {
      led_state = false;
      br_target = br_min;
      mode_auto = true; // включаем авто режим
      half_timer_start = millis();  // сброс таймера выключения в авторежиме
    }
    btn_pressed = false; // снимаем признак нажатия кнопки
    if (debug == 1) Serial.println("led_state = " + String(led_state));
    if (debug == 1) Serial.println("br_target = " + String(br_target));
    if (debug == 1) Serial.println("mode_auto = " + String(mode_auto));


  }

  // Обработка датчика движения ---------------------

  sensor_command = digitalRead(sensor_pin); // читаем состояние признака наличия движения


  if (digitalRead(light_pin) == low_light)  timer_light = millis(); // пока состояние с пина датчика света совпадает со значением переменной low_light подсчитываем время
  else
  {
    if (millis() - timer_light > delay_light)  low_light = ! low_light; // инвертируем значение low_light, если есть устойчивое состояние с пина датчика света
  }


  // Управление в автоматическом режиме
  if (mode_auto == true)
  {

    if (sensor_command == true)  // включаем
    {
      if (debug == 1) Serial.println("sensor_command = " + String(sensor_command));

      br2_target = br_max;
      half_timer_start = millis();

      if (low_light == true) // если в комнате темно
      {
        //br_target = br_half;
        led_state = true;
      }
    }

    if (sensor_command == false) // выключаем только после задержки
    {

      if (millis() - half_timer_start > half_delay_on)
      {
        br_target = br_min;
        //br2_target = br_min;
        led_state = false;
      }

      if (millis() - half_timer_start > half_delay_on2)
      {
        br2_target = br_min;
        //led_speed = led_speed1;
      }

    }


    // отложенное включение main_led
    if (led_state && round(br2) == br_max)
    {
      br_target = br_half;
    }

  }


  // Управляем яркостью ----------------------------

  // расчитываем шаг
  br2_step = (float)d_time * (br_max - br_min) / t2_min_max;
  //if (br2 > br_max * 0.8) br2_step = br2_step * 10;

  br_step = (float)d_time * (br_max - br_min) / t_min_max;
  if (br < br_half) br_step = br_step / 30;


  // меняем яркость main_led
  if (round(br) != br_target)
  {
    if (br < br_target) br = br + br_step;
    else if (br > br_target) br = br - br_step;
    br = constrain(br, br_min, br_max);
    if (debug == 1) Serial.println("br = " + String(round(br)));

    analogWrite(main_led_pin, round(br));
    //yield();
  }

  // меняем яркость second_led
  if (round(br2) != br2_target)
  {
    if (br2 < br2_target) br2 = br2 + br2_step;
    else if (br2 > br2_target) br2 = br2 - br2_step;
    br2 = constrain(br2, br_min, br_max);
    if (debug == 1) Serial.println("br2 = " + String(round(br2)));

    analogWrite(second_led_pin, round(br2));
    //yield();

  }




  /*
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
  */

  // управление встроенным светодиодом
  /*
    #ifndef nodeMCU
    if (led_state == 0) digitalWrite(LED_BUILTIN, LOW);
    else digitalWrite(LED_BUILTIN, HIGH);
    #endif
  */


  

  // nodeMCU ---------------------------
/*
  #ifdef nodeMCU
  if (OTA_on) ArduinoOTA.handle(); // Всегда готовы к прошивке
  if (MQTT_on) mqtt_call();
#endif
*/


  d_time = millis() - time_old; // длительность цикла
  time_old = millis();

} // end loop



// ==========================================================================================


float change_br(float brightness, int brightness_target, float brightness_step)
{
  //  float brightness_step = 0;
  //  if (brightness < br_half) brightness_step = (float)br_step / 10;
  //  else brightness_step = br_step;

  if (brightness < brightness_target) brightness = (float)brightness + brightness_step;
  if (brightness > brightness_target) brightness = (float)brightness - brightness_step;
  //constrain(brightness, br_min, br_max);
  return (brightness);
}



void mqtt_call()
{
  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED && millis() - mqtt_timer > mqtt_delay)
  {
    mqtt_timer = millis();
    if (!client.connected())
    {
      //Serial.print("Connecting to MQTT server ");
      //Serial.print(mqtt_server);
      //Serial.println("...");
      if (client.connect(MQTT::Connect("arduinoClient2").set_auth(mqtt_user, mqtt_pass)))
      {
        //Serial.println("Connected to MQTT server ");
        client.set_callback(callback);
        // подписываемся под топики
        client.subscribe("led_state");
        client.subscribe("mode_auto");
        //client.subscribe("br_target");
        client.subscribe("sensor_command");
        client.subscribe("low_light");
        client.subscribe("btn_value");
        //client.subscribe("vcc");
        client.subscribe("d_time");
      }
      else
      {
        //Serial.println("Could not connect to MQTT server");
      }
    }

    if (client.connected()) {
      client.loop();
      refreshData();
    }
  }
}


// Функция отправки показаний
void refreshData() {
  client.publish("led_state", String(led_state));
  client.publish("mode_auto", String(mode_auto));
  client.publish("br_target", String(br_target));
  client.publish("br", String(br));
  client.publish("sensor_command", String(sensor_command));
  client.publish("low_light", String(low_light));
  client.publish("btn_value", String(btn_value));
  //client.publish("vcc", String(ESP.getVcc())); // считаем напряжение на VCC (через пин A0)
  client.publish("d_time", String(d_time));

  delay(1);
}



// Функция получения данных от сервера
void callback(const MQTT::Publish& pub)
{
  String payload = pub.payload_string();
  String topic = pub.topic();

  //Serial.print(pub.topic()); // выводим в сериал порт название топика
  //Serial.print(" => ");
  //Serial.println(payload); // выводим в сериал порт значение полученных данных


  // проверяем из нужного ли нам топика пришли данные
  if (topic == "br_target")
  {
    //br_target= payload.toInt();

  }

}



bool try_to_connect_wf(wf_connected)
{

if (debug==1) {Serial.println("try_to_connect_wf /result="+"xxx")");}
return(xxx);
}

/*
bool try_to_connect_mqtt(mqtt_connected)
{

if (debug==1) {Serial.println("try_to_connect_mqtt /result="+"xxx")");}
return(xxx);
}
*/
