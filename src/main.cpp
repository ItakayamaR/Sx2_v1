/*
  Programa para pruebas con los modulos lora RFM98W, RA-02 (433) y RN2903 (915)
  Empresa: Diacsa
  Autor: Ismael Takayama
    
    Se usaron las librerias Lora.h de Sandeep Mistry y rn2xx3.h de jpmeijers
    Datasheets:
    RFM98W: https://cdn.sparkfun.com/assets/learn_tutorials/8/0/4/RFM95_96_97_98W.pdf
    RA-02: https://docs.ai-thinker.com/_media/lora/docs/c048ps01a1_ra-02_product_specification_v1.1.pdf
    RN2903: https://ww1.microchip.com/downloads/en/DeviceDoc/RN2903-Data-Sheet-DS50002390J.pdf

    User guide:
    RN2903: https://ww1.microchip.com/downloads/en/DeviceDoc/RN2903%20LoRa%20Technology%20Module%20Command%20Reference%20User%20Guide-40001811B.pdf 

*/

#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include "Pines.h"
#include "LoRa.h"
#include "rn2xx3.h"

//Definiciones para la libreria
#define LORA_BW               125E3         //Bandwith
#define LORA_SP               12            //Spreading Factor
#define LORA_CHANNEL          915E6   
#define LORA_SYNCWORD         0x12          
#define LORA_CR               5             //Coding rate (4/x)
#define LORA_PL               8             //Preamble length (x+4)
#define LORA_ADDRESS          4
#define LORA_SEND_TO_ADDRESS  2


byte MODO = 0;
byte MODO_ANT = 0;

byte e;
char message_received[100];
char message_sent[]="hola";
int delay_time=5;
int counter=0;

//Creamos una instancia de rnx2xx3 para la comunicación con el módulo 3
rn2xx3 myLora(Serial1);

//Funciones usadas en el programa
void Ini_module_spi(byte m); 
void Ini_module3();
void End_modules();
void EnableDevice(byte m);
uint8_t send_message(uint8_t module, char *message, uint8_t seconds, boolean control=false);
uint8_t receive_message(uint8_t module, char seconds, boolean control=false);

void setup()
{
  delay(10);

  //Inicializamos los pines de LED y selección
  pinMode(LED, OUTPUT);
  pinMode(SEL1, INPUT);
  pinMode(SEL2, INPUT);

  //Inicializamos pines del módulo 1 (RFM98W)
  pinMode(DIO0_1, INPUT);
  pinMode(DIO1_1, INPUT);
  pinMode(DIO2_1, INPUT);
  pinMode(RST1, OUTPUT);
  pinMode(SS1, OUTPUT);
  digitalWrite(SS1,HIGH);

  //Inicializamos pines del módulo 2 (RA-02)
  pinMode(DIO0_2, INPUT);
  pinMode(DIO1_2, INPUT);
  pinMode(DIO2_2, INPUT);
  pinMode(RST2, OUTPUT);
  pinMode(SS2, OUTPUT);
  digitalWrite(SS2,HIGH);

  //Inicializamos pines del módulo 3 (RN2903)
  pinMode(FLAG, INPUT);
  pinMode(RST3, OUTPUT);

  // Abrimos puerto serial a computadora
  Serial.begin(115200); 

  //Iniciamos los modulos en reset
  digitalWrite(RST1,0);
  digitalWrite(RST2,0);
  digitalWrite(RST3,0);
  
}

void loop(void)
{ 
  uint8_t status;
  // Leemos el modo
  MODO = ( (digitalRead(SEL2)<<1) + digitalRead(SEL1) );
  
  if (MODO_ANT != MODO){
    EnableDevice(MODO);         //Habilitamos el modulo según la posición de los jumpers
    MODO_ANT=MODO;
    counter=0;                  //Reiniciamos el contador
  }
  
  //Comentar o descomentar para los módulos en modo de transmisión/recepción
  //status=send_message(MODO, message_sent, delay_time, true); (Modulo de emision, mensaje a enviar, delay entre mensajes, con/sin mensaje de confirmación)
  status=receive_message(MODO, 20, true);

  //Serial.println(status);
  delay(100);
}

uint8_t send_message(uint8_t module, char *message, uint8_t seconds, boolean control){
  uint8_t status = 0;
  uint8_t i = 0;
  if (module==1 || module==2){
    // Enviamos un mensaje   
    Serial.println("Start sending message"); 
     //Prendemos el led
    digitalWrite(LED,1);
    LoRa.beginPacket();
    LoRa.print("N°: ");
    LoRa.print(counter);
    LoRa.print(" ");
    LoRa.print("Msg: ");
    LoRa.print(message);
    LoRa.endPacket();
    digitalWrite(LED,0);

    Serial.print("Message sent: ");
    Serial.println(message);
    Serial.print("Message n° ");
    Serial.println(counter);
    status=1;

    if (control == true){
      LoRa.receive();
      for(i=0; i<10; i++) {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
          Serial.print(".");
          String LoRaData = LoRa.readString();
          if (LoRaData=="OK"){
            Serial.println("Message confirmed");
            break;
          }
        } else {
          Serial.println("Waiting for confirmation");
        } 
        delay(1000);
        if (i==10) Serial.println("No confirmation");
      }  
    }  

  } else if (module == 3){
    
  }
  Serial.println(""); 
  counter++;
  delay(seconds*1000); 
  return status;
}

uint8_t receive_message(uint8_t module, char seconds, boolean control){
  uint8_t i=0;
  uint8_t status=0;
  if (module==1 || module==2){
    LoRa.receive();
    //Esperamos a recibir un mensaje
    while(i < seconds ){
      int packetSize = LoRa.parsePacket();
      if (packetSize) {
        // received a packet
        Serial.print("Received packet: '");
        // read packet
        while (LoRa.available()) { 
          Serial.print(LoRa.readString()); 
        }

        // Imprimimos el RSSI
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
        
        if(control == true){
          Serial.println("Sending confirmation"); 
          LoRa.beginPacket();
          LoRa.print("OK");
          LoRa.endPacket();
        }

        //Prendemos el led por 0.5s
        digitalWrite(LED,1);
        delay(500);
        digitalWrite(LED,0);

        status=1;
        break;

      } else{
        Serial.println("Waiting for message");
      }
      delay(1000);
      i++;
    }

  } else if (module == 3){
    
  }
  if (status == 0) { Serial.println("No message received"); }
  Serial.println("");
  return status;
}

void Ini_module_spi(byte m) 
{
  //Inicializamos SPI en los pines correspondientes
  if (m==1){
    LoRa.setPins(SCK, MISO, MOSI, SS1, RST1, DIO0_1);

  } else if (m==2) {
    LoRa.setPins(SCK, MISO, MOSI, SS2, RST2, DIO0_2);
  }

  //Seteamos la frecuencia deseada y los canales  y esperamos que se configure
  while (!LoRa.begin(LORA_CHANNEL)) {
    Serial.println(".");
    delay(500);
  }

  LoRa.setSyncWord(LORA_SYNCWORD);              //Seteamos la dirección de sincronización
  LoRa.setSpreadingFactor(LORA_SP);             //Seteamos el Spreading Factor (SP)
  LoRa.setSignalBandwidth(LORA_BW);             //Seteamos El ancho de banda
  LoRa.setCodingRate4(LORA_CR);                 //Seteamos el Coding rate (4/(x))
  LoRa.setPreambleLength(LORA_PL);              //Seteamos la longitud del preambulo (x+4)


  // Mensaje de comprobación
  Serial.println(F("Module configured finished"));
  Serial.println();

}

void Ini_module3()
{
  // Abrimos puerto serial del módulo 3 (RN2903)
  Serial1.begin(9600, SERIAL_8N1, RX, TX);

}

void End_modules(){
  Serial1.end();
  LoRa.end();
  digitalWrite(RST1,1);
  digitalWrite(RST2,1);
  digitalWrite(RST3,1);
}

void EnableDevice(byte m){
  End_modules();
  
  switch(m)
  {
    case 0:
      Serial.println("modo 0");
      break;

    case 1:
      Serial.println("modo 1");
      Ini_module_spi(m);
      break;
    case 2:
      Serial.println("modo 2");
      Ini_module_spi(m);
      break;

    case 3:
      Serial.println("modo 3");
      Ini_module3(); 
      break;
  }
}