/*
    Pines usados para la comunicación con los módulos Lora
*/

//Definimos pines para la comunicación SPI
#define MISO 12         
#define MOSI 13
#define SCK 14

//Definimos pines para el modulo 1 (RFM98W)
#define DIO0_1 25
#define DIO1_1 23
#define DIO2_1 32
#define RST1 26
#define SS1 37

//Definimos pines para el modulo 2 (RA-02)
#define DIO0_2 21
#define DIO1_2 22
#define DIO2_2 23
#define RST2 19
#define SS2 18

//Definimos pines para el modulo 3 (RN2903-I/RM)
#define RX 4
#define TX 16
#define CTS 15
#define FLAG 17
#define RST3 5

//Definimos pines para selección y led
#define SEL1 36
#define SEL2 39
#define LED  2
