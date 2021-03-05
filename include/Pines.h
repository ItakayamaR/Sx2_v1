/*! 
    Pines usados para la comunicación con los módulos Lora
 */

//Definimos pines para la comunicación SPI
#define MISO 12         
#define MOSI 13
#define SCK 14

//Definimos pines para el modulo 1 (RFM95W)
#define DIO0_1 26
#define DIO1_1 25
#define DIO2_1 33
#define RST1 27
#define SS1 32

//Definimos pines para el modulo 2 (LORA1276V2.0)
#define DIO0_2 19
#define DIO1_2 21
#define DIO2_2 22
#define RST2 19
#define SS2 23

//Definimos pines para el modulo 3 (E32-433T30D)
#define RX 4
#define TX 16
#define AUX 15
#define M0 5
#define M1 17

//Definimos pines para selección y led
#define SEL1 36
#define SEL2 39
#define LED  2
