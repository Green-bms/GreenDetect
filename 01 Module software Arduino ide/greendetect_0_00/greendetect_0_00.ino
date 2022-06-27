
/*
  Green Detect module


 Wireless Sensor Module with ESP-NOW protocol
 
 The module works according to the following steps:

1. sensor signal acquisition

2. receiving data from the previous form

3. data transmission to the next module

4. Deep sleep mode (30 seconds)


  Versions:

  - Ver. 0.00 date 27/06/2022 created by Sergio Ghirardelli (first release)

*/



#include <ESP8266WiFi.h>
#include <espnow.h>
#include "functions_5.h"
#include <EEPROM.h>
#include <Wire.h>
#include "SparkFunHTU21D.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include "MCP3X21.h"  






const uint8_t mcp3221_address = 0x4D;
const uint16_t ref_voltage = 3300;  // in mV
uint16_t MCP3221_channel;

MCP3221 mcp3221(mcp3221_address);



HTU21D htu21d_sensor;

unsigned long t, t0, t1, sample;

int i, n, z, k, x; //counters

//SR04 sensor
long duration, distance;
int distance_int;
int test_dist;

//htu21d sensor
float htu21d_temp, htu21d_hum;
int htu21d_temp_int, htu21d_hum_int;
byte htu21d_temp_byte, htu21d_hum_byte;

bool ir_input; //IR sensor
bool module_test; //module test

//supercap voltage measuring
int a0_channel, vx, vx_lim, vx_perc;
byte soc;

// Onewire DS18B20 temperature sensor
const int oneWireBus = 12;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
float onewire_temp;
int onewire_temp_int;

byte first_code; // code written in case of first use

//variables for program modes
unsigned long u_elapsed, u_count, da_elapsed, da_count;
unsigned long both_elapsed, both_count;
unsigned long net_elapsed, net_count;
bool osc, but_u, but_da, u, da, both, both_p, both_mem, prog, prog_p, prog_mem, prog_n, prog_mem2, osc_n, osc_mem, prog1, prog1_n, prog_mem3;
bool u_n, u_mem, da_n, da_mem, wdt_test;
int u_number, da_number, new_address, u_led_preset, u_led_elapsed, da_led_preset, da_led_elapsed, address, sleep_time, new_sleep_time, sensor_type, new_sensor_type, gain_offset, prog_step, deep_set;
byte last_set, fault_mode;


//variables used for transmission
byte send_step;
bool wait_n, wait_n_mem;
int app_time;
unsigned long lastTime;
unsigned long timerDelay;
unsigned long comp_time;
byte val1, val2, val3;
bool wait, wait_q;
unsigned long wait_elapsed, wait_count;
bool gateway_send;
bool deltime, deltime_q;
unsigned long deltime_elapsed, deltime_count;

unsigned long time_start, time_stop, life_start, life_stop; //variables to measure task time
unsigned long deep_time; // deep sleep time
bool send_ok;
bool pippo;

// Starting receiver address set
uint8_t broadcastAddress[] = {0x16, 0x16, 0x16, 0x16, 0x16, 0x00};

// Starting sensor module address
uint8_t newMACAddress[] = {0x16, 0x16, 0x16, 0x16, 0x16, 0x00};

// data structure
typedef struct struct_data {
  byte battery_soc[60];
  byte data1[60];
  byte data2[60];
  byte data3[60];
  byte sync_time;
  byte life;
  byte number;
} struct_data;

// structures
struct_data data_send;
struct_data data_rec;
struct_data data_internal;

//variables used to check previous module fault
bool timeout, timeout_q;
unsigned long timeout_elapsed, timeout_count, timeout_set;

//task time management
bool task, task_q;
unsigned long task_elapsed, task_count;
unsigned long advance_time;
long offset_time;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  //Serial.println("Last Packet Send Status: ");
  if (sendStatus == 0) {
      send_ok = true;
  }
  else {
   // Serial.println("Delivery fail");
      send_ok = false;
  }
}


// Callback function that will be executed when data is received. Received data is copied in internal data variables.
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&data_rec, incomingData, sizeof(data_rec));

  if ((address > 1) && (address <= 60)) {

    for ( i = 1; i < address; i++) {

      data_internal.battery_soc[i] = data_rec.battery_soc[i];
      data_internal.data1[i] = data_rec.data1[i];
      data_internal.data2[i] = data_rec.data2[i];
      data_internal.data3[i] = data_rec.data3[i];
    }

    data_internal.sync_time = data_rec.sync_time;
    data_internal.life = data_rec.life;
   
  }



  if (address == 99) {
    data_internal.number = data_rec.number;
   
    for ( i = 1; i <= data_internal.number; i++) {

      data_internal.battery_soc[i] = data_rec.battery_soc[i];
      data_internal.data1[i] = data_rec.data1[i];
      data_internal.data2[i] = data_rec.data2[i];
      data_internal.data3[i] = data_rec.data3[i];
    }

    data_internal.sync_time = data_rec.sync_time;
    data_internal.life = data_rec.life;
    
    gateway_send = true; 
    serial_transmission();

  }


}


void setup() {
  life_start = micros();
  
  // disable WIFI
  WiFi.mode(WIFI_OFF);

  //pin configuration
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(15, OUTPUT); //mosfet enable
  pinMode(12, INPUT); // Sensor Digital input (SR04 echo, IR, etc)
  pinMode(13, INPUT_PULLUP); // Button 1
  pinMode(14, INPUT_PULLUP); // Button 2
  pinMode(0, OUTPUT); //led 1



  //reading from EEprom, if first code is different from 114 it means that Eeprom is empty
  EEPROM.begin(9);
  first_code = EEPROM.read(3);

  if (first_code == 114) {
    address = EEPROM.read(0);
    last_set = EEPROM.read(2);
    sleep_time = EEPROM.read(4);
    sensor_type = EEPROM.read(6);
    fault_mode = EEPROM.read(8);
  }
  else {
    address = 0;
    last_set = 0;
    sleep_time = 50;
    sensor_type = 0;
    fault_mode = 0;
  }

  EEPROM.end();


 

  //Sensor Digital output (SR04 trig)
  if ((address != 99) && (sensor_type == 1)) {
    pinMode (4, OUTPUT);
  }

  //sensors supply mosfet activation
  digitalWrite(15, HIGH);

  //Onewire DS18B20 sensor setup
  if ((address != 99) && (sensor_type == 3)) {
    sensors.setWaitForConversion(false);
    sensors.requestTemperatures();
  }


  //MCP3221 ADC setup
  #if defined(ESP8266) || defined(ESP32)
  Wire.begin(SDA, SCL);
  mcp3221.init(&Wire);
  #else
  mcp3221.init();
  #endif


  //delay time for sensors start up
  delay (100);

  //variables initialization
  new_address = address;
  newMACAddress [5] = address;
  x = 0;
  k = 0;
  wdt_test = 0;
  lastTime = 0;
  timerDelay = 10;
  sample = 0;
  gateway_send = false;
  data_internal.life = 0;
  wait = true;
  task = false;
  timeout = false;
  send_step = 0;
  distance = 0;
  deep_set = 30;
  ir_input = false ;
  pippo = false;
  
  
  //module address management
  if (address == 0) {
    broadcastAddress [5] = 0x0; //module not addressed
  }
  else {
    if (last_set) {
      broadcastAddress [5] = 0x63; //when the module is the last, the recipient address is 0x63
    }
    else {
      broadcastAddress [5] = address + 1; //set the recipient address to the next address of the group
    }
  }


  //if both buttons are pressed, set the program mode
  if ((!digitalRead(13)) && (!digitalRead(14))) {
    prog_step = 1;
    prog = true;
    u_number = 1;
    da_number = 1;
  }
  else {
    prog_step = 0;
    prog = false;
    u_number = 0;
    da_number = 0;
  }

  //if only button2 is pressed, a memory reset is done
  if ((digitalRead(13)) && (!digitalRead(14))) {
    first_code = 0;
    EEPROM.begin(9);
    EEPROM.write(3, first_code);
    EEPROM.commit();
    EEPROM.end();
    delay (10);
    ESP.restart();
  }


  // Init Serial Monitor
  Serial.begin(115200);





  //Sensors reading
  if (address != 99) {

    //supercap voltage
    a0_channel = analogRead(A0);
    vx = map (a0_channel, 0, 1023, 0, 3100);
    vx_lim = constrain(vx, 1500, 3000);
    vx_perc = map (vx_lim, 1500, 3000, 0, 60);
    soc = byte (vx_perc);

    //sensor test button set the bit 7 of soc byte
    if (!digitalRead(13)) {
      module_test = true;
    }
    if (module_test) {
      bitSet(soc, 7);
    }

     //ADC analog input 
     MCP3221_channel = mcp3221.read();
    
    //sensor type = 0 --> repeater
    if (sensor_type == 0) {      
      val1 = 255;
      val2 = 255;
      val3 = 255;      
    }

    //sensor type = 1 --> SR04 ultrasonic distance sensor
    if (sensor_type == 1) {      
      digitalWrite(4, LOW);
      delayMicroseconds(2);
      digitalWrite(4, HIGH);
      delayMicroseconds(10);
      digitalWrite(4, LOW);
      duration = pulseIn(12, HIGH, 20000);
      distance = duration / 58.2;
      distance_int = int (distance);
      val1 = distance_int & 0xff;
      val2 = (distance_int >> 8);
      val3 = 0;    
    }

    //sensor type = 2 --> HTU21D temperature + humidity I2C sensor
    if (sensor_type == 2) {      
      htu21d_sensor.begin();
      htu21d_temp = htu21d_sensor.readTemperature();
      htu21d_hum = htu21d_sensor.readHumidity();
      float temp_x10 = htu21d_temp * 10;
      htu21d_temp_int = int (temp_x10);
      htu21d_hum_int = int (htu21d_hum);
      htu21d_hum_byte = byte (htu21d_hum_int);
      val1 = htu21d_temp_int & 0xff;
      val2 = (htu21d_temp_int >> 8);
      val3 = htu21d_hum_byte;
      if (!digitalRead(12)) {
        ir_input = true ;
      }
      if (ir_input) {
        bitSet(soc, 6);
      }
      
    }

    //sensor type = 3 --> Onewire DS18B20 temperature sensor
    if (sensor_type == 3) {
      
      onewire_temp = sensors.getTempCByIndex(0);
      float temp_x10 = onewire_temp * 10;
      onewire_temp_int = int (temp_x10);
      val1 = onewire_temp_int & 0xff;
      val2 = (onewire_temp_int >> 8);
      val3 = 0;
     
    }


    //sensor type = 4 --> Soil moisture sensor
   
    if (sensor_type == 4) {      
      val1 = MCP3221_channel & 0xff;
      val2 = (MCP3221_channel >> 8);
      val3 = 0;           
      }

    //to measure sensor type reading time 
    /*time_start = micros();
    
    time_stop = micros() - time_start; 
    Serial.println ("jk");
    Serial.println (time_stop); */



  }

  //sensors supply mosfet deactivation
  digitalWrite(15, LOW);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(10); 
  wifi_set_macaddr(STATION_IF, &newMACAddress[0]);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  send_ok = false;
  deltime = false;
  deltime_q = false;
}




void loop() {

  

  /*Push buttons managements*/
  but_u = !digitalRead(13); //digital channels reading
  but_da = !digitalRead(14);
  u = TON(but_u, 50, u_elapsed, u_count); //buttons must be pressed for at least 50 msec
  da = TON(but_da, 50, da_elapsed, da_count);
  both = TON ((u and da), 3000, both_elapsed, both_count);//both buttons are pressed for 3 seconds
  both_p = PosTrig(both, both_mem); //positive edge of both variable
  prog_p = PosTrig(prog, prog_mem); //positive edge of prog: when program mode is set
  prog_n = NegTrig(prog, prog_mem2); //negative edge of prog: when program mode is reset
  prog1_n = NegTrig(prog1, prog_mem3); //negative edge of prog1: when program mode 1 is reset
  u_n = NegTrig(u, u_mem); //negative edge of u: when SW1 button is released
  da_n = NegTrig (da, da_mem); //negative edge of da: when SW2 button is released

  //oscillator for blinking
  if (millis() - t >= sample)
  {
    osc = !osc;
    t += sample;
  }


  /*Program mode or Normal mode calling*/
  if (prog_step == 1) {
    program();
  }

  else if (prog_step == 2) {
    prog_last();
  }


  else if (prog_step == 3) {
    prog_time();
  }

  else if (prog_step == 4) {
    prog_type();
  }

  else if (prog_step == 0) {
    normal();
  }


  
  test_dist =  data_internal.data2[1] << 8 | data_internal.data1[1];


  
  

  //serial monitor
/*  Serial.println (MCP3221_channel);
   Serial.print (" ");
    Serial.println (var);*/



}// end of loop



//normal mode
void normal () {


  if (prog_p) { // executed one time at the end of program mode. The new address set is stored and microcontroller reset

    digitalWrite(LED_BUILTIN, LOW); // both leds on to show the end of program mode
    digitalWrite(0, HIGH);
    address = new_address;
    sensor_type = new_sensor_type;


    EEPROM.begin(9);
    EEPROM.write(0, address);
    EEPROM.write(2, last_set);
    EEPROM.write(6, sensor_type);
    EEPROM.write(8, 0);

    if (first_code != 114) { //first EEprom address write
      first_code = 114;
      EEPROM.write(3, first_code);
    }
   /* if (address == 1) { //first EEprom address write
      sleep_time = new_sleep_time;
      EEPROM.write(4, sleep_time);
    }

    if ((address > 1) && (address <= 60)) { //first EEprom address write
      sleep_time = 60;
      EEPROM.write(4, sleep_time);
    }*/

    sleep_time = new_sleep_time;
    EEPROM.write(4, sleep_time);

    EEPROM.commit();
    EEPROM.end();

    newMACAddress [5] = address;
    n = address;
    delay(3000); //three seconds of freezing before starting the normal mode
    digitalWrite(0, LOW);
    ESP.restart();
  }


  if (both_p) {    //in case of pressing of both buttons for 3 seconds, system starts program mode
    prog = true;
    prog_step = 1;
  }

  //oscillator for gateway sending
  if (millis() - t >= 10)
  {
      if ((address==99) && (!gateway_send)) {
        Serial.println ("%");     
      }
    t += 10;
  }

  //task time management
  /*task_q = TON (!task, 120, task_elapsed, task_count);
    if (!task_q) {normal_sensor();}
    else {normal_send();}*/
  normal_send();

} // end of Normal mode




/*Program Mode to set the sensor module esp-now address*/
void program() {

  if (prog_p) {// executed one time at the start of program mode
    sample = 300;// sample time of leds blinking management (in msec)
    digitalWrite(15, HIGH);
    /*decrement of tens and units numbers*/
    u_number--;
    da_number--;
    u_led_elapsed = 0;
    da_led_elapsed = 0;
    digitalWrite(LED_BUILTIN, LOW); // both leds on to show the end of program mode
    digitalWrite(0, HIGH);
    delay(3000); //three seconds of freezing before starting the program mode
    digitalWrite(0, LOW);
    prog = false;
  }

  if (both_p) {    //in case of pressing of both buttons for 3 seconds, system soes to next program step
    if (new_address == 99) {
      prog = true;
      prog_step = 4;
    }
    else {
      prog = true;
      prog_step = 2;
    }

  }

  if (u_n) { //increment of units value after SW1 button release
    u_number++;
  }
  if (da_n) { //increment of tens value after SW2 button release
    da_number++;
  }
  if (u_number > 9) { //restart of units value
    u_number = 0;
  }
  if (da_number > 9) { //restart of tens value
    da_number = 0;
  }
  new_address = (da_number * 10) + u_number; //new address number calculation

  /*preset values of leds blink mangement */
  u_led_preset = u_number + 2;
  da_led_preset = da_number + 2;
  osc_n = NegTrig(osc, osc_mem);



  if (osc)
  {
    if ((u_led_elapsed >= 1) && (u_led_elapsed <= u_number)) { //blinking of Units led to show the set value
      digitalWrite(0, HIGH);
    }
    else  {
      digitalWrite(0, LOW);
    }

    if ((da_led_elapsed >= 1) && (da_led_elapsed <= da_number)) { //blinking of Tens led to show the set value
      digitalWrite(LED_BUILTIN, LOW);
    }
    else  {
      digitalWrite(LED_BUILTIN, HIGH);
    }

  }
  else
  {
    digitalWrite(0, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (osc_n) //led blinking counters increment or reset every negative edge of the oscillator
  {
    u_led_elapsed++;
    if (u_led_elapsed > u_led_preset) {
      u_led_elapsed = 0;
    }
    da_led_elapsed++;
    if (da_led_elapsed > da_led_preset) {
      da_led_elapsed = 0;
    }
  }

}


/*Program step to set the sensor module as last of the network*/
void prog_last() {

  if (prog_p) { // executed one time at the end of program mode
    sample = 50; //led flashlight sample time (in msec)
    digitalWrite(LED_BUILTIN, LOW); // both leds on to show the end of program mode
    digitalWrite(0, HIGH);
    last_set = 0;
    delay(3000); //three seconds of freezing before starting the normal mode
    digitalWrite(0, LOW);
    prog = false;
  }


  if (both_p) {    //in case of pressing of both buttons for 3 seconds, system returnes to normal mode
    prog = true;
    prog_step = 4;
  }



  if (da_n) {
    last_set = 1; //set module as last of the whole battery pack
  }
  if (u_n) {
    last_set = 0; //set module as not the last of the whole battery pack
  }

  //if module is set as the last, builtin led flashlight, else led1 flashlights
  if (osc) {

    if (last_set == 1) {
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(0, LOW);
    }
    else {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(0, HIGH);
    }

  }

  else {

    digitalWrite(0, LOW);
    digitalWrite(LED_BUILTIN, HIGH);

  }
}


/*Program Mode to set the synchronization time for all the sensors (only if address is set to 1)*/
void prog_time() {

  if (prog_p) {// executed one time at the start of program mode
    sample = 300;// sample time of leds blinking management (in msec)

    /*decrement of tens and units numbers*/
    u_number = -1;
    da_number = -1;
    u_led_elapsed = 0;
    da_led_elapsed = 0;
    digitalWrite(LED_BUILTIN, LOW); // both leds on to show the end of program mode
    digitalWrite(0, HIGH);
    delay(3000); //three seconds of freezing before starting the program mode
    digitalWrite(0, LOW);
    prog = false;
  }

  if (both_p) {    //in case of pressing of both buttons for 3 seconds, system returnes to normal mode
    prog = true;
    prog_step = 0;
  }

  if (u_n) { //increment of units value after SW1 button release
    u_number++;
  }
  if (da_n) { //increment of tens value after SW2 button release
    da_number++;
  }
  if (u_number > 9) { //restart of units value
    u_number = 0;
  }
  if (da_number > 9) { //restart of tens value
    da_number = 0;
  }
  new_sleep_time = (da_number * 10) + u_number; //new address number calculation

  /*preset values of leds blink mangement */
  u_led_preset = u_number + 2;
  da_led_preset = da_number + 2;
  osc_n = NegTrig(osc, osc_mem);



  if (osc)
  {
    if ((u_led_elapsed >= 1) && (u_led_elapsed <= u_number)) { //blinking of Units led to show the set value
      digitalWrite(0, HIGH);
    }
    else  {
      digitalWrite(0, LOW);
    }

    if ((da_led_elapsed >= 1) && (da_led_elapsed <= da_number)) { //blinking of Tens led to show the set value
      digitalWrite(LED_BUILTIN, LOW);
    }
    else  {
      digitalWrite(LED_BUILTIN, HIGH);
    }

  }
  else
  {
    digitalWrite(0, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (osc_n) //led blinking counters increment or reset every negative edge of the oscillator
  {
    u_led_elapsed++;
    if (u_led_elapsed > u_led_preset) {
      u_led_elapsed = 0;
    }
    da_led_elapsed++;
    if (da_led_elapsed > da_led_preset) {
      da_led_elapsed = 0;
    }
  }

}



/*Program Mode to set the sensor type*/
void prog_type() {

  if (prog_p) {// executed one time at the start of program mode
    sample = 300;// sample time of leds blinking management (in msec)

    /*decrement of tens and units numbers*/
    u_number = -1;
    da_number = -1;
    u_led_elapsed = 0;
    da_led_elapsed = 0;
    digitalWrite(LED_BUILTIN, LOW); // both leds on to show the end of program mode
    digitalWrite(0, HIGH);
    delay(3000); //three seconds of freezing before starting the program mode
    digitalWrite(0, LOW);
    prog = false;
  }

  if (both_p) {    //in case of pressing of both buttons for 3 seconds, system returnes to normal mode

    if (new_address == 1) {
      prog = true;
      prog_step = 3;
    }
    else {
      prog = true;
      prog_step = 0;
    }

   

  }

  if (u_n) { //increment of units value after SW1 button release
    u_number++;
  }
  if (da_n) { //increment of tens value after SW2 button release
    da_number++;
  }
  if (u_number > 9) { //restart of units value
    u_number = 0;
  }
  if (da_number > 9) { //restart of tens value
    da_number = 0;
  }
  new_sensor_type = (da_number * 10) + u_number; //new address number calculation

  /*preset values of leds blink mangement */
  u_led_preset = u_number + 2;
  da_led_preset = da_number + 2;
  osc_n = NegTrig(osc, osc_mem);



  if (osc)
  {
    if ((u_led_elapsed >= 1) && (u_led_elapsed <= u_number)) { //blinking of Units led to show the set value
      digitalWrite(0, HIGH);
    }
    else  {
      digitalWrite(0, LOW);
    }

    if ((da_led_elapsed >= 1) && (da_led_elapsed <= da_number)) { //blinking of Tens led to show the set value
      digitalWrite(LED_BUILTIN, LOW);
    }
    else  {
      digitalWrite(LED_BUILTIN, HIGH);
    }

  }
  else
  {
    digitalWrite(0, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (osc_n) //led blinking counters increment or reset every negative edge of the oscillator
  {
    u_led_elapsed++;
    if (u_led_elapsed > u_led_preset) {
      u_led_elapsed = 0;
    }
    da_led_elapsed++;
    if (da_led_elapsed > da_led_preset) {
      da_led_elapsed = 0;
    }
  }

}


// first task
void normal_sensor() {
  digitalWrite(LED_BUILTIN, HIGH);
  
  }


// transmission task
void normal_send() {

  
  
  digitalWrite(LED_BUILTIN, LOW);
  

   

  

  //address 1 transmission
  if (address == 1) {

    
    
    wait_q = TON (wait, 1, wait_elapsed, wait_count);
    if (wait_q) {
      wait = false;
    }

    deltime_q = TON (deltime, 5000, deltime_elapsed, deltime_count);
    if (deltime_q) {
      ESP.deepSleep(27e6);
    }
      
    if (!wait) {

      if ((millis() - lastTime) > timerDelay) {
        if (!send_ok) {
          data_internal.life = 1;
          deltime = true;
          }
        else {
         
          data_internal.life++;
          deltime = false;
          }
        
        if (data_internal.life > 7) {
          data_internal.life = 0;
          wait = true;

        }
        esp_now_send(broadcastAddress, (uint8_t *) &data_send, sizeof(data_send));

        if (data_internal.life == 1) {
          data_send.life = data_internal.life;
          }
        else {
          data_send.battery_soc[1] = soc;
          data_send.data1[1] = val1;
          data_send.data2[1] = val2;
          data_send.data3[1] = val3;
          
          data_send.life = data_internal.life;

          
          
          if (last_set==1) {
            data_send.sync_time = deep_set;
            data_send.number = address;}
          
          if (last_set==0) {
                  data_send.sync_time = sleep_time;
                  }


          if (data_internal.life == 0) {
              deep_time = (deep_set * 1000000)+ offset_time;
          
              ESP.deepSleep(30500000);
              }
          }
        
        lastTime = millis();
      }


    }
  }

  // address from 2 to 60 transmission
  else if ((address > 1) && (address <= 60)) {

    //define the advance time according to sensor time reading 
    if (sensor_type == 0) {advance_time = 5;}
    else if (sensor_type == 1) {advance_time = 5659;}
    else if (sensor_type == 2) {advance_time = 72658;}
    else if (sensor_type == 3) {advance_time = 27628;}
    else if (sensor_type == 4) {advance_time = 9;}
    if (fault_mode == 0) {
    
    
    
      if (data_internal.life == 0) {
        timeout = true;
      }
      else {
        timeout = false;
        if (!pippo) {
        life_stop = micros() - life_start;
        unsigned long sync_temp  = (unsigned long)  data_internal.sync_time;
        comp_time =  sync_temp * 100000;        
        offset_time = life_stop - comp_time;
        if (offset_time >= 10000000) {offset_time = 0;}
        if ((offset_time > 1000000) && (offset_time < 10000000)) {offset_time = 1000000;}
             
        deep_time = (deep_set * 1000000) + offset_time;
            
        /*Serial.println ("");
        Serial.println (comp_time);
        Serial.println (life_stop);*/
        pippo = true;
        }
      }
      //timeout_set = (sleep_time + 11) * 1000;
      timeout_set = (address * 500) + 121000;
      timeout_q = TON (timeout, timeout_set, timeout_elapsed, timeout_count);

      if (timeout_q) {
        fault_mode = 1;
        EEPROM.begin(9);
        EEPROM.write(8, fault_mode);
        EEPROM.commit();
        EEPROM.end();

      }
      
      deltime_q = TON (deltime, 5000, deltime_elapsed, deltime_count);
      if (deltime_q) {
          unsigned long deep_deltime = (deep_set * 1000000) - 3000000;
          ESP.deepSleep(deep_deltime);
        }
        
      if (data_internal.life >= 5) {
        
        
        
        
        if ((millis() - lastTime) > timerDelay) {
          if (!send_ok) {
            send_step = 1;
            deltime = true;
            }
          else {
            send_step++;
            deltime = false;
            }
          
          if (send_step > 7) {
            data_internal.life = 0;
            send_step = 0;
          }
          
          esp_now_send(broadcastAddress, (uint8_t *) &data_send, sizeof(data_send));

          if (send_step == 1) {
          data_send.life = send_step;
              }
        
          else { 

                for ( k = 1; k < address; k++) {
                  data_send.battery_soc[k] = data_internal.battery_soc[k];
                  data_send.data1[k] = data_internal.data1[k];
                  data_send.data2[k] = data_internal.data2[k];
                  data_send.data3[k] = data_internal.data3[k];
                }
      
                data_send.battery_soc[address] = soc;
                data_send.data1[address] = val1;
                data_send.data2[address] = val2;
                data_send.data3[address] = val3;
                
                data_send.life = send_step;
                if (last_set==1) {
                  data_send.number = address;
                  data_send.sync_time = deep_set;
                  }
                if (last_set==0) {
                  data_send.sync_time = data_internal.sync_time;
                  }
                if (send_step == 0) {
                 
                 
                  
                  ESP.deepSleep(deep_time);
                }
            }
          lastTime = millis();
        }
      }

    }

    // address from 2 to 60 transmission in case of previous module fault
    if (fault_mode == 1) {
      digitalWrite(0, HIGH);
      wait_q = TON (wait, 1, wait_elapsed, wait_count);
      if (wait_q) {
        wait = false;
      }

      deltime_q = TON (deltime, 5000, deltime_elapsed, deltime_count);
      if (deltime_q) {
      ESP.deepSleep(27e6);
      }
      
      if (!wait) {

        if ((millis() - lastTime) > timerDelay) {
          if (!send_ok) {
            data_internal.life = 1;
            deltime = true;
            }
          else {
            data_internal.life++;
            deltime = false;
            }
          
          if (data_internal.life > 7) {
            data_internal.life = 0;
            wait = true;
          }
          esp_now_send(broadcastAddress, (uint8_t *) &data_send, sizeof(data_send));

          if (data_internal.life == 1) {
              data_send.life = data_internal.life;
            }

          else {
          
              data_send.battery_soc[address] = soc;
              data_send.data1[address] = val1;
              data_send.data2[address] = val2;
              data_send.data3[address] = val3;
          
              data_send.life = data_internal.life;
              if (last_set==1) {
                data_send.number = address;
                data_send.sync_time = deep_set;
                }
              if (last_set==0) {
                data_send.sync_time = data_internal.sync_time;
                }
          
              if (data_internal.life == 0) {
                deep_time = (deep_set * 1000000) - advance_time + offset_time;
           
                ESP.deepSleep(30500000);
                }
            }
          lastTime = millis();
        }
      }


    }


  }


}//end of normal_send


//data transmission (address = 99)
void serial_transmission() {
  if (data_internal.life >= 5) {
      for ( x = 1; x <= data_internal.number; x++) {
      Serial.println ("#" + (String (x)));
      Serial.println ("&1" + (String (data_internal.battery_soc[x])));
      Serial.println ("&2" + (String (data_internal.data1[x])));
      Serial.println ("&3" + (String (data_internal.data2[x])));
      Serial.println ("&4" + (String (data_internal.data3[x])));
      }
      Serial.println ("&5" + (String (data_internal.sync_time)));
      Serial.println ("$"); // end of transmission
      gateway_send = false; 
   
  }

}
