// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <LoRa.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

#if (ESP8266 || ESP32)
    #define ISR_PREFIX ICACHE_RAM_ATTR
#else
    #define ISR_PREFIX
#endif

LoRaClass::LoRaClass() :
  _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
  _spi(&LORA_DEFAULT_SPI),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _sck(LORA_DEFAULT_SCK_PIN), _miso(LORA_DEFAULT_MISO_PIN), _mosi(LORA_DEFAULT_MOSI_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL),
  _onTxDone(NULL)
{
  // Seteamos tiempo de espera en 0 (luego se dará un tiempo basado en número de símbolos)
  setTimeout(0);
}

// Función usada para configurar el módulo. Se setea la frecuencia y la potencia de transmisión.
int LoRaClass::begin(long frequency)
{
  // Seteamos el pin de SS (slave select) del SPI
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);

  //Damos un reset al módulo
  if (_reset!= -1) {
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, HIGH);
    delay(10);
    // perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }

  // Empezamos SPI con los pines (SPI usa modo 0, predeterminado en ESP32)
  _spi->begin(_sck, _miso, _mosi);

  // Se chequea la versión (0x12) Además, sirve para comprobar la comunicación mediante SPI
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    //Si el número de versión es distinta, sale de la función
    //Un error común es que devuelva 0, indicando un error en la transmisión por SPI
    Serial.println(readRegister(REG_VERSION));
    return 0; 
  }

  // Ponemos en modo de sleep (Los registros deben ser modificados solo en modo Sleep o Standby)
  sleep();

  // Se setea la frecuencia
  setFrequency(frequency);

  // Escribimos la dirección base de los punteros de transmisión y recepción en el buffer
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // Seteamos el LNA (low-noise amplifier) a corriente aumentada (recomendado) 
  // Solo sirve para frecuencias superiores a 526
  // Para esto ponemos a 1 el bit 2 del registro REG_LNA (0x0c) 
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // Seteamos auto AGC (Control automático de ganancias)
  // Ponemos a 1 el bit 3 del registro REG_MODEM_CONFIG_3
  writeRegister(REG_MODEM_CONFIG_3, readRegister(REG_MODEM_CONFIG_3) | 0x04);

  // Seteamos la potencia de transmisión a 20dbm (máximo)
  setTxPower(20);

  // Regresamos a modo standby
  idle();

  return 1;
}

// Función para terminar la comunicación con el módulo usado.
void LoRaClass::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  _spi->end();
}

/*Función para configurar el inicio de una transmisión. Se define modo implicito/explicito
y se resetea la dirección base del buffer y la cantidad de bytes del mensaje*/
int LoRaClass::beginPacket(int implicitHeader)
{
  //Si existe un mensaje transmitiendo, saale de la función
  if (isTransmitting()) {
    return 0;
  }

  // Ponemos el módulo en estado Standby (idle)
  idle();

  //Definimos el modo en el que nos encontrarmos (por defecto modo explicito)
  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // Reseteamos la dirección del FIFO y la cantidad de bytes del mensaje
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}


//Función para empezar el envío de mensajes
int LoRaClass::endPacket(bool async) // Por defecto se configura en modo asincrónico
{
  // Si nos encontramos en modo asincrónico 
  // Configuramos el pin DIO0 para que se ponga a 1 cuando se termine de enviar un mensaje
  if ((async) && (_onTxDone)){
      writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
  }
  // Ponemos en modo transmisión. Al terminar de enviar el mensaje, el modo pasará automáticamente a idle
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  //delay(500);

  //Si no nos encontrarmos en modo asincronico, esperamos a una bandera para saber que el mensaje ha sido enviado
  if (!async) {
    // Esperamos mientras el mensaje se termina de enviar. el bit 3 del registro REG_IRQ_FLAGS debe ponerse a 1
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) 
    {
      //Serial.print(".");
      //Serial.println(readRegister(REG_IRQ_FLAGS));
      //Serial.println(readRegister(REG_OP_MODE));
      //Serial.println("Sending message");
      //yield();
      //delay(1000);
    }
    
    // Limpiamos las  banderas
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }
  return 1;
}

//Función que retorna verdadero si un mensaje se encuentra en transmisión
bool LoRaClass::isTransmitting()
{
  //Consulta si se encuentra en modo TX. Cuando el mensaje se termina de enviar automáticamente pasa a modo STBY
  if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return true;
  }

  //Limpia la bandera de mensaje transmitido
  if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

int LoRaClass::parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  //Serial.println("Llega aqui"); 
  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  //Serial.println(irqFlags); 

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

int LoRaClass::packetRssi()
{
  return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRaClass::packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LoRaClass::packetFrequencyError()
{
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & B111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

  if (readRegister(REG_FREQ_ERROR_MSB) & B1000) { // Sign bit is on
     freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return static_cast<long>(fError);
}

int LoRaClass::rssi()
{
  return (readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

//Función para escribir un mensaje en el FIFO
size_t LoRaClass::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

//Función para escribir un mensaje en el FIFO 
size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  // Leemos la cantidad de bytes presentes en el FIFO
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // Chequeamos que la nueva cantidad más la cantidad existente no sobrepase la longitud máxima del mensaje
  // En caso se sobrepase, solo se escriban bytes hasta el límite permitido
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // Escribimos la datat en el registro
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // Actualizamos el número de bytes en el registro REG_PAYLOAD_LENGTH
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

int LoRaClass::available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
  if (!available()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}

int LoRaClass::peek()
{
  if (!available()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = readRegister(REG_FIFO);

  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

void LoRaClass::flush()
{
}

void LoRaClass::onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}

void LoRaClass::onTxDone(void(*callback)())
{
  _onTxDone = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}

void LoRaClass::receive(int size)
{

  writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}


//Funcion que cambia el modo del modulo a Standby. 
void LoRaClass::idle()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/*Funcion que cambia el modo del modulo a sleep. Los modos se cambian escribiendo los bits 2-0 del registro
REG_OP_MODE(0x01) el valor del modo deseado 
000 --> SLEEP
001 --> STDBY
010 --> Frequency synthesis TX (FSTX)
011 --> Transmit (TX)
100 --> Frequency synthesis RX (FSRX)
101 --> Receive continuous (RXCONTINUOUS)
110 --> receive single (RXSINGLE)
111 --> Channel activity detection (CAD) 
Además, se debe poner el bit 7 del mismo registro a 1, para indicar que se trata de modos LoRa*/
void LoRaClass::sleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

//Función para cambiar la potencia de transmisión.
void LoRaClass::setTxPower(int level, int outputPin)
{
  //Si se utiliza el pin de transmisión PA_HF/LF (alta eficiencia, menor amplificación (hasta 14 dBm))
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }
    //Se escribe el nivel deseado bajo la formula Pout=15-(15-level)
    writeRegister(REG_PA_CONFIG, 0x70 | level);

  //Si se utiliza el pin de transmisión PA_BOOST (Permite una amplificación de hasta 20 dBm)
  //Pin conectado por default en la mayoría de módulos
  } else {
    // Si se escoge una potencia mayor a 17, se debe activar un registro adicional
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }
      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // Registro adicional para potencias de 20 dBm
      writeRegister(REG_PA_DAC, 0x87);
      // Se adapta el nivel de protección de sobrecorriente (OCP) al nivel de poder actual
      setOCP(240);

    //Si se escoge una potencia menor a 17, se escribe en el registro el valor deseado -2
    } else {
      if (level < 2) {
        level = 2;
      }
      //Valor por default para pin PA_HF/LF o potencias menores a +17dBm
      writeRegister(REG_PA_DAC, 0x84);
      // Se adapta el nivel de protección de sobrecorriente (OCP) al nivel de poder actual
      setOCP(100);
    }

    // En el registro REG_PA_CONFIG (0x09) se setea a 1 el bit 7 para indicar que se usa el pin PA_BOOST
    // En los bits 3-0 se escribe el valor de corriente calculado
    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

/*Función que setea la frecuencia, modificando los registros REG_FRF_MSB, REG_FRF_MID y REG_FRF_LSB
para hallar el valor a setear en los registros, se usa la siguiente formula:
valor_registro=frecuencia*2^19 / 32*10^6 */
void LoRaClass::setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LoRaClass::getSpreadingFactor()
{
  return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

//Función para setear el spreading factor (SF)
void LoRaClass::setSpreadingFactor(int sf)
{
  //Se definen los límites
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  // Si seteamos el SF a 6 (máxima velocidad) se deben setear los siguientes valores en los registros
  // REG_SYNC_WORD(0x31) y REG_DETECTION_THRESHOLD(0x37)
  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  // Se escribe el spreading factor en los bits (7-4) del registro REG_MODEM_CONFIG_2 (0x1e)
  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));

  // Seteamos un registro especial si la duración de un símbolo excede 16 ms
  setLdoFlag();
}

long LoRaClass::getSignalBandwidth()
{
  byte bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }

  return -1;
}

//Función para setear el ancho de canal
void LoRaClass::setSignalBandwidth(long sbw)
{
  int bw;
  //Comparamos el valor dado con una tabla establecida
  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  // Se escribe en los bits (4-7) del registro REG_MODEM_CONFIG_1(0x1d) el valor calculado 
  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));

  // Seteamos un registro especial si la duración de un símbolo excede 16 ms
  setLdoFlag();
}

/* Función para setear el bit 3 del registro REG_MODEM_CONFIG_3(0x26) si la duración del un símbolo 
excede los 16 ms. Esto activa la optimización de velocidad de datos bajo LowDataRateOptimize que 
incrementa la robustez ante variaciones en frecuencia durante la duracón del la transmisión y 
recepcion de mensajes */ 
void LoRaClass::setLdoFlag()
{
  // Seccion 4.1.1.5 datasheet Sx1276 (función para calcular la duración de canal en base al WB y SF)
  long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

  // Sección 4.1.1.6 datasheet Sx1276
  boolean ldoOn = symbolDuration > 16;

  uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
  bitWrite(config3, 3, ldoOn);
  writeRegister(REG_MODEM_CONFIG_3, config3);
}

// Función para setear el coding rate (de 4/5 a 4/8)
void LoRaClass::setCodingRate4(int denominator)
{
  // Delimitamos los límites
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  /*001 -> 4/5
    010 -> 4/6
    011 -> 4/7
    100 -> 4/8 */
  int cr = denominator - 4;
  
  //Escribimos en los bits (3-1) del registro REG_MODEM_CONFIG_1 el valor calculado
  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

// Seteamos la longitud de preambulo (desde 6+4 hasta 65535+4)
void LoRaClass::setPreambleLength(long length)
{  
  //Existen dos registros para esto, REG_PREAMBLE_LSB para los bits 0-7 y REG_PREAMBLE_MSB para los bits 8-15
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/*Seteamos la dirección de sincronización. Esta sirve para definir una dirección común a todos
los dispositivos de una red 
Esta se escribe en el registro REG_SYNC_WORD(0x39) */
void LoRaClass::setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

//Habilitamos el envío de crc seteando un 1 en el bit 2 del registro REG_MODEM_CONFIG_2 (0x1e)
void LoRaClass::enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

//Deshabilitamos el envío de crc poniendo a 0 el bit 2 del registro REG_MODEM_CONFIG_2 (0x1e)
void LoRaClass::disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

//Invierte las señales I e Q de la modulación de Lora
void LoRaClass::enableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x66);
  writeRegister(REG_INVERTIQ2, 0x19);
}

void LoRaClass::disableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x27);
  writeRegister(REG_INVERTIQ2, 0x1d);
}


/* Función que habilita la protección contra sobrecorrientes (Overload current protection)
Se usaa las siguientes formulas:
Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) 
Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to 240 mA)
Default Imax = 100mA */
void LoRaClass::setOCP(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }
  // En el registro REG_OCP (0x0b) se pone a 1 el bit 5 para habilitar el OCR, y en ls  bis 0-4 el valor de opctrim calculado
  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRaClass::setGain(uint8_t gain)
{
  // check allowed range
  if (gain > 6) {
    gain = 6;
  }
  
  // set to standby
  idle();
  
  // set gain
  if (gain == 0) {
    // if gain = 0, enable AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);
  } else {
    // disable AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x00);
	
    // clear Gain and set LNA boost
    writeRegister(REG_LNA, 0x03);
	
    // set gain
    writeRegister(REG_LNA, readRegister(REG_LNA) | (gain << 5));
  }
}

byte LoRaClass::random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

void LoRaClass::setPins(int8_t sck, int8_t miso, int8_t mosi, int ss, int reset, int dio0)
{
  _sck = sck;
  _miso = miso;
  _mosi = mosi;
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

void LoRaClass::setSPI(SPIClass& spi)
{
  _spi = &spi;
}

void LoRaClass::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void LoRaClass::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

/*Función para defnir el modo explicito 
El modo explicito añade una cabecera al m mensaaje que contiene información sobre el número de bytes,
el coding rate y si se usó un CRC */
void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  // El modo explicito se define poniendo a 0 el bit 0 del registro REG_MODEM_CONFIG_1(0x1d)
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

/*Función para defnir el modo implpicito
El modo explicito añade no incluye cabecera, por lo cual los datos del número de bytes, el coding rate y
el uso de crc debe estár defiinidos de antemano en el receptor */
void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  // El modo explicito se define seteando a 1 el bit 0 del registro REG_MODEM_CONFIG_1(0x1d)
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRaClass::handleDio0Rise()
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {

    if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
      // received a packet
      _packetIndex = 0;

      // read packet length
      int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

      // set FIFO address to current RX address
      writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

      if (_onReceive) {
        _onReceive(packetLength);
      }
    }
    else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
      if (_onTxDone) {
        _onTxDone();
      }
    }
  }
}

/*Para indicar lectura de registro, el primer bit a enviar debe ser 0
Este bit se pone a 0 en esta función */
uint8_t LoRaClass::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

/*Para indicar escritura de registro, el primer bit a enviar debe ser 1
este bit se setea en esta función*/
void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

//Función que transfiere un byte con la dirección, y retorna un byte con el valor de la dirección o escribe un byte en la dirección dada
//Dependiendo del valor del bit 7 del primer byte.
uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;

  digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);
  _spi->transfer(address);
  response = _spi->transfer(value);
  _spi->endTransaction();

  digitalWrite(_ss, HIGH);

  return response;
}

ISR_PREFIX void LoRaClass::onDio0Rise()
{
  LoRa.handleDio0Rise();
}

LoRaClass LoRa;
