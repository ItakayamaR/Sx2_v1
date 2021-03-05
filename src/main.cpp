#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include "Pines.h"
#include "LoRa_E32.h"
#include "LoRa.h"


//Definiciones para la libreria
#define LORA_BW               125E3
#define LORA_SP               12
#define LORA_CHANNEL          915E6
#define LORA_SYNCWORD         0x12
#define LORA_ADDRESS          4
#define LORA_SEND_TO_ADDRESS  2

byte MODO = 0;
byte MODO_ANT = 0;

byte e;
char message_received[100];
char message_sent[]="hola";
int delay_time=5;
int counter=0;

//Configuramos la clase para el módulo 3
LoRa_E32 E32_433(RX, TX, &Serial1, AUX, M0, M1);  // e32 TX e32 RX 

void Ini_module_spi(byte m); 
void Ini_module3();
void End_module3();
void End_module_spi();
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

  //Inicializamos pines del módulo 1
  pinMode(DIO0_1, INPUT);
  pinMode(DIO1_1, INPUT);
  pinMode(DIO2_1, INPUT);
  pinMode(RST1, OUTPUT);
  pinMode(SS1, OUTPUT);
  digitalWrite(SS1,HIGH);

  //Inicializamos pines del módulo 2
  pinMode(DIO0_2, INPUT);
  pinMode(DIO1_2, INPUT);
  pinMode(DIO2_2, INPUT);
  pinMode(RST2, OUTPUT);
  pinMode(SS2, OUTPUT);
  digitalWrite(SS2,HIGH);

  //Iniciamos comunicación para el Módulo 3
  E32_433.begin();

  // Abrimos comunicaciones para observar 
  Serial.begin(115200); 

  //Iniciamos los modulos en reset
  digitalWrite(RST1,0);
  digitalWrite(RST2,0);
  digitalWrite(M0,1);
  digitalWrite(M1,1);
  
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
    /*
    LoRa.print("N°: ");
    LoRa.print(counter);
    LoRa.print(" ");
    LoRa.print("Msg: ");
    LoRa.print(message);*/

    LoRa.print("A");
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
    //enviamos un mensaje
    char *Count = (char*)malloc(40);
    sprintf(Count, "N°: %u, Msg: ", counter);


    //enviamos un mensaje
    digitalWrite(LED,1);
    ResponseStatus rs = E32_433.sendMessage(strcat(Count,message));
    digitalWrite(LED,0);
    if (rs.getResponseDescription() == "Success"){
      Serial.println("Messaje sent");
      Serial.println(message);
      Serial.print("Messaje N°: ");
      Serial.println(counter);
      status=1;
    } else{
      Serial.println("Error");
    }
    free(Count); 

    if(control == true){
      for (i=0; i < 10; i++) {
        if (E32_433.available() > 1){
          ResponseContainer rs = E32_433.receiveMessage();
          if (rs.status.getResponseDescription() == "Success"){
            if (rs.data == "OK")
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
    //Esperamos a recibir un mensaje
    while (i < seconds) {
      if (E32_433.available() > 1){
        ResponseContainer rs = E32_433.receiveMessage();
        if (rs.status.getResponseDescription() == "Success"){
          Serial.print("Received packet: '");
          Serial.println(rs.data);

          //Enviamos confirmación
          if (control==true){
            Serial.println("Sending confirmation"); 
            E32_433.sendMessage("OK");
          }

          //Prendemos el led por 0.5s
          digitalWrite(LED,1);
          delay(500);
          digitalWrite(LED,0);
          status=1;
          break;
        } 
      } else {
          Serial.println("Waiting for message");
      }
      delay(1000);
      i++;
    }

    
    Serial.println(""); 
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

  LoRa.setSyncWord(LORA_SYNCWORD);        //Seteamos la dirección de sincronización
  LoRa.setSpreadingFactor(LORA_SP);             //Seteamos el Spreading Factor (SP)
  LoRa.setSignalBandwidth(LORA_BW);         //Seteamos El ancho de banda
  LoRa.setCodingRate4(5);                 //Seteamos el Coding rate (4/(x-4))
  LoRa.setPreambleLength(8);              //Seteamos la longitud del preambulo (x+4)


  // Mensaje de comprobación
  Serial.println(F("Module configured finished"));
  Serial.println();

}


void Ini_module3(){
  digitalWrite(M0,1);
  digitalWrite(M1,1);
  
  ResponseStructContainer c;
	c = E32_433.getConfiguration();
	// It's important get configuration pointer before all other operation
	Configuration configuration = *(Configuration*) c.data;
	Serial.println(c.status.getResponseDescription());
	Serial.println(c.status.code);

	printParameters(configuration);
	configuration.ADDL = 0x0;
	configuration.ADDH = 0x1;
	configuration.CHAN = 0x19;

	configuration.OPTION.fec = FEC_1_ON;
	configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
	configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
	configuration.OPTION.transmissionPower = POWER_30;
	configuration.OPTION.wirelessWakeupTime = WAKE_UP_1250;

	configuration.SPED.airDataRate = AIR_DATA_RATE_000_03;
	configuration.SPED.uartBaudRate = UART_BPS_9600;
	configuration.SPED.uartParity = MODE_00_8N1;

	// Set configuration changed and set to not hold the configuration
	ResponseStatus rs = E32_433.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
	Serial.println(rs.getResponseDescription());
	Serial.println(rs.code);
	printParameters(configuration);
	c.close();
}

void End_module3(){
  digitalWrite(M0,1);
  digitalWrite(M1,1);  
}

void End_module_spi(){
  LoRa.end();
  digitalWrite(RST1,1);
  digitalWrite(RST2,1);
}

void EnableDevice(byte m){
  End_module_spi();
  End_module3();
  
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