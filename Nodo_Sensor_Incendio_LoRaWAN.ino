
/*Nodo LoRa ABP Arduino Pro Mini 3v3 Atmega328p 8MHz

  Banda de frecuencia US915 MHz
  Sub Banda 1 (903.9 - 905.3) ---> Configuración del Gateway Single Channel ESP
  Canal 0 (de la sub banda 1) ---> Configuración del Gateway Single Channel ESP
  Canal en el que envia 903.9 MHz (canal 8 de 72)

  Radio HopeRF RFM95w (SX1276) 915MHz 

  Libreria: LMiC v1.5 *Modificada*
    Modificación: lmic.cpp LMIC_disableChannel(u1_t channel) 
      //LMIC.channelMap[channel/4] &= ~(1<<(channel&0xF)); 
      LMIC.channelMap[channel/16] &= ~(1<<(channel&0xF));

  Conexión de pines (Arduino Pro Mini):
   _____________________________
  |   RFM95w   |Arduino Pro Mini|
  |____________|________________|
  |GND         |GND             |
  |3.3v        |VCC (3.3v)      |
  |MISO        |12              |
  |MOSI        |11              |
  |SCK         |13              |
  |NSS         |10              |
  |DIO0        |4               |
  |DIO1        |5               |
  |DIO2        |7               |
  |____________|________________|

  Conexión de pines (TTGO Pro Mini_LORA - T-DEER):

    Puente entre los pines D5 y IO1 (Necesario para que envie mas de un paquete)
*/


//#define WAIT_SECS 120   //60 Segundos en Arduino Pro Mini
//#define WAIT_SECS 60  //60 Segundos en TTGO Pro Mini_LORA - T-DEER
//#define WAIT_SECS 15  //15 Segundos en TTGO Pro Mini_LORA - T-DEER
//#define WAIT_SECS 900  //900 Segundos (15 min) en TTGO Pro Mini_LORA - T-DEER
#define WAIT_SECS 300  //300 Segundos (5 min) en TTGO Pro Mini_LORA - T-DEER
//#define NO_WAIT 1  //15 Segundos en TTGO Pro Mini_LORA - T-DEER


#include "lmic.h"
#include "hal/hal.h"

//#include <ArduinoJson.h>
#include <BME280I2C.h>
#include <Wire.h>

#define sensorLlama A0  //Pin para el sensor analogico de llamas
#define sensorHumo A1   //Pin para el sensor analogico de humo (monoxido de carbono)
#define intIncendio 3   //Pin para interrupción por fuego  
#define waitDetect 1000 //Tiempo de espera para reenvio de alerta por fuego
#define waitSend 25000  //Tiempo de espera para reenvio de mensajes

//---------------------------------------------------------
// Declaracion de variables
//---------------------------------------------------------

// Contador de tramas
//int count=0;
//char buffer[50];
//String output;
bool det = false;
unsigned long previousMillis = 0;

// LoRaWAN Application identifier (AppEUI)
// No usado
static const u1_t APPEUI[8] PROGMEM = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// LoRaWAN DevEUI
// No usado
static const u1_t DEVEUI[8] PROGMEM  = { 0x42, 0x42, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// LoRaWAN NwkSKey, Network Session Key 
// Network Session Key The Things Network
unsigned char NwkSkey[16] =     { 0x1a, 0xf9, 0xf1, 0x46, 0x91, 0xe8, 0x36, 0xea, 0x43, 0xdd, 0xfc, 0xbb, 0x5c, 0x5f, 0x11, 0x19 };

// LoRaWAN AppSKey, Application Session Key
// Application Session Key para desencriptación de mensajes en The Things Network
unsigned char AppSkey[16] =   { 0x7a, 0xa6, 0xae, 0xeb, 0x4d, 0x81, 0xfd, 0xfd, 0x33, 0x2d, 0xbb, 0xc9, 0xf6, 0x43, 0xa1, 0xd1 };


#define msbf4_read(p)   (u4_t)((u4_t)(p)[0]<<24 | (u4_t)(p)[1]<<16 | (p)[2]<<8 | (p)[3])

// LoRaWAN end-device address (DevAddr)
unsigned char DevAddr[4] = { 0x01, 0x17, 0x90, 0x36 };

//SENSORES

//SENSORES
BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   0x76 // I2C address. I2C specific.
);
BME280I2C bme(settings);


// ----------------------------------------------------------------------------
// APPLICATION CALLBACKS
// ----------------------------------------------------------------------------

// Desabilitar todos los canales, excepto el canal definido en el if de ForceTxSingleChannelDr()
// Setear el SF en LMIC_setDrTxpow() al final de esta función.
// Esto solo afecta a los mensajes enviados (uplinks). 

void forceTxSingleChannelDr() {
    for(int j=0; j<71; j++) { // US i<71 canales
        if(j != 8) {
            LMIC_disableChannel(j);
        }
    }
    // Set data rate (SF) and transmit power for uplink
    LMIC_setDrTxpow(DR_SF10, 14);
}

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, NwkSkey, 16);
}

int debug=1;
uint8_t mydata[64];
//char mydata[64];
static osjob_t sendjob;

// ----------------------------------------------------------------------------
// Pin mapping (Arduino Pro Mini)
// ----------------------------------------------------------------------------

// lmic_pinmap pins = {
//   .nss = 10,          // Conectado a D10 (Arduino Pro Mini)
//   .rxtx = 0,          // No conectar al RFM92/RFM95
//   .rst = 0,           // No conectar al RFM92/RFM95
//   .dio = {4, 5, 7},   // Especificación de pines DIO0, DIO1, DIO2 (RFM95)
//                       // Conectado a D4, D5, D7 (Arduino Pro Mini)
// };

// ----------------------------------------------------------------------------
// Pin mapping (TTGO Pro Mini_LORA 20180926 - T-DEER) 
// ----------------------------------------------------------------------------

lmic_pinmap pins = {
  .nss = 10,          // Conectado a D10 (TTGO Pro Mini_LORA)
  .rxtx = 0,          // No conectar al RFM92/RFM95
  .rst = 9,           // Conectado a D9 (TTGO Pro Mini_LORA)
  .dio = {2, 5, 0},   // Especificación de pines DIO0, DIO1, DIO2 (RFM95)
                      // Conectado a D2, D5, No Conectar (TTGO Pro Mini_LORA)
};

// ----------------------------------------------------------------------------
// Declaración de funciones
// ----------------------------------------------------------------------------

void onEvent (ev_t ev) {

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          //Serial.print("EV_TXCOMPLETE, tiempo: ");
          ////Serial.print((millis() / 1000)/2); //Pro Mini
          //Serial.print((millis() / 1000));
          //Serial.println(" Segundos");
          if(LMIC.dataLen) { // Datos recibidos en la ventana de recepción (RX slot) luego del envio de datos (TX)
              //Serial.println("Data Received");
          }
          break;
       default:
          break;
    }
}

void do_send(osjob_t* j){
    delay(1);                         // delay agregado para la comunicación //serial
      //Serial.print("Tiempo: ");
      //Serial.print((millis() / 1000)/2); //Pro Mini
      //Serial.print((millis() / 1000));
      //Serial.println(" Segundos");
      //Muestra el canal TX 
      //Serial.print("Enviado. Canal: ");
      //Serial.println(LMIC.txChnl);
      //Serial.print("Opmode check: ");
      //Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      //Serial.println("OP_TXRXPEND, not sending");
    } 
    else {

        //Lectura de los sensores
    //long MonoxidoCarbono = 5000;
    int16_t Llama;
    int16_t MonoxidoCarbono;
    float Temperatura, Humedad, Presion;
    unsigned long currentMillis = millis();
  
    previousMillis = currentMillis;
    Llama = analogRead(sensorLlama);
    MonoxidoCarbono = analogRead(sensorHumo);

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_hPa);

    bme.read(Presion, Temperatura, Humedad, tempUnit, presUnit);

    //Transformación de tipos - float -> int/long
    //int c = int(MonoxidoCarbono);
    int8_t t = int8_t(Temperatura);
    int8_t h = int8_t(Humedad);
    int16_t p = int16_t(Presion);
    int8_t d;
    if (det){
      d = 1;
    }else{
      d = 0;
    }
    Serial.println(d);
    det = false;
    uint8_t buffer[64];
          //  co|tem|hum|pre|fue|det
    sprintf(buffer, "%d|%d|%d|%d|%d|%d", MonoxidoCarbono, t, h, p, Llama, d);
    //sprintf(buffer, "{\"co\" : %d,\"tem\" : %d,\"hum\" : %d,\"pres\" : %ld,\"det\" : %d}", c, t, h, p, d);

   
    strcpy((char *) mydata,buffer); //Mensaje que se envia a TTN
    LMIC_setTxData2(1, mydata, strlen((char *)mydata), 0);
    
    // Schedule a timed job to run at the given timestamp (absolute system time)
    os_setTimedCallback(j, os_getTime()+sec2osticks(WAIT_SECS), do_send);
    
    }

         
}

void deteccion(){
    det = true;
    Serial.println("D");
}

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(intIncendio), deteccion, RISING);

  Wire.begin();
  bme.begin();

  // Inicializacion de LMiC
  os_init();
  // Definición de la banda de frecuencia a utilizar (No se si cumple alguna función esto aquí)
  #if defined(CFG_us915)
  LMIC.freq = 903900000;
  #endif
  // Resetea el estado de la MAC. Sesión o datos pendientes de envio serán descartados
  LMIC_reset();
  // Establece la sesión con los parametros suministrados en la sección "Declaración de Variables"
  LMIC_setSession (0x1, msbf4_read(DevAddr), (uint8_t*)NwkSkey, (uint8_t*)AppSkey);
  // Desabilita la adaptación de SF (configuración para nodos móviles)
  LMIC_setAdrMode(0);
  // Disabilita link check validation
  LMIC_setLinkCheckMode(0);
  // Desabilita beacon tracking
  LMIC_disableTracking ();
  // Desabilita la recepcion periodica de datos (configuración para dispositivos LoRaWAN Clase A)
  LMIC_stopPingable();
  // Establece el canal, SF y potencia del transmisor para el envio de datos
  forceTxSingleChannelDr(); //reemplazar valores en la definición de la función forceTxSingleChannelDr() en la sección "Declaración de funciones"
  
#if defined(__AVR__)
  //Serial.println("AVR arch");
#elif defined(ARDUINO_ARCH_ESP8266)
  //Serial.println("ESP arch");
#elif defined(__MKL26Z64__)
  //Serial.println("Teensy arch");
#else
  //Serial.println("WARNING. Unknown Arch");
#endif

}

// ----------------------------------------------------------------------------
// LOOP
// ----------------------------------------------------------------------------
void loop() {

  do_send(&sendjob);
  while(1) {
    os_runloop_once(); 
    delay(100);
  }
}
