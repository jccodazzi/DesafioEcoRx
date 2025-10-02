/* DesafioEcoRx V0.02
 *
 * Funcionalidades implementadas:
 * - Recibe paquetes LoRa en 433 MHz (configuración LoRa en el archivo).
 * - Soporta recepción de paquete binario TelemetryPkt (struct empaquetada, little-endian).
 * - Copia segura del payload en OnRxDone y procesamiento en loop() (evita operaciones pesadas en IRQ).
 * - Decodifica TelemetryPkt y muestra campos en TFT: pkt_num, elapsed_s, lat, lon, speed,
 *   distance_cm, voltage_mv, current_ma, energy_ah_tenth y RSSI.
 * - Imprime por Serial los valores decodificados en formato legible.
 * - Envía ACK al transmisor con formato "ACK:<pkt_num>" desde loop() (OnRxDone solo marca bandera).
 * - Control de estado con flags: lora_idle, sendAckPending, rxReady.
 * - Manejo básico de errores y timeouts: OnRxTimeout, OnRxError, OnTxTimeout.
 * - Diseñado para interoperar con DesafioEcoTx (misma definición TelemetryPkt).
 *
 * Notas:
 * - Evitar llamar Radio.Send u operaciones de UI desde callbacks; todo envío/IO se hace en loop().
 * - Asegurarse que el emisor use el mismo packing/endianness para TelemetryPkt.
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"               // TFT

#define RF_FREQUENCY                                433000000 // Hz
#define TX_OUTPUT_POWER                             18        // dBm

#define LORA_BANDWIDTH                              0
#define LORA_SPREADING_FACTOR                       12
#define LORA_CODINGRATE                             4
#define LORA_PREAMBLE_LENGTH                        24
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 128

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t lastRssi;
int16_t rxSize;

volatile bool lora_idle = true; // volatile porque se usa en IRQ/callbacks

// Envío de ACK desde loop()
volatile bool sendAckPending = false;
volatile unsigned long ackPktNum = 0;

// Indica que hay dato recibido para procesar/mostrar en loop()
volatile bool rxReady = false;

HT_st7735 tft;                      // objeto TFT

// Nuevo: seguimiento de ACK en proceso / último ACK enviado (para mostrar estado)
volatile unsigned long ackBeingSent = 0; // pkt num que estamos enviando ahora mismo
volatile unsigned long lastAckSent = 0;  // último pkt num cuyo ACK fue transmitido

// Telemetry packet definition (debe coincidir con el transmisor)
typedef struct __attribute__((packed)) {
    uint16_t pkt_num;         // Número de paquete (16 bits)
    uint16_t elapsed_s;       // Tiempo transcurrido en segundos (16 bits)
    int32_t  lat;             // Latitud en microgrados (1e-6)
    int32_t  lon;             // Longitud en microgrados (1e-6)
    uint8_t  speed;           // Velocidad (8 bits)
    uint32_t distance_cm;     // Distancia recorrida en centímetros (32 bits)
    uint16_t voltage_mv;      // Voltaje en milivoltios (16 bits)
    uint16_t current_ma;      // Corriente en mA (16 bits)
    uint16_t energy_ah_tenth; // Energía total en Ah/10 (16 bits)
} TelemetryPkt;

// Prototipos
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    // TFT init
    tft.st7735_init();
    tft.st7735_fill_screen(ST7735_BLACK);					// Pantalla en negro
    tft.st7735_write_str(0, 0, (String)"GPS+LoRa433MHz");	// Título
    tft.st7735_write_str(0, 20, (String)"Iniciando...");	// Mensaje
    tft.st7735_write_str(0, 40, (String)" DesafioEcoRx ");	// Version
    tft.st7735_write_str(0, 60, (String)"    V 0.02    ");	// Version
    delay(3000);
    tft.st7735_fill_screen(ST7735_BLACK);					// Pantalla en negro
    tft.st7735_write_str(0, 0, (String)"DesEco RX 0.02");	// Cabecera fija
    tft.st7735_write_str(0, 20, (String)"Paq:");            // Numero de paquete
    tft.st7735_write_str(0, 40, (String)"ACK:");            // Estado ACK
    tft.st7735_write_str(0, 60, (String)"RSSI:");           // RSSI
    // Inicializar estados
    lastRssi = 0;
    rxSize = 0;

    // Callbacks radio
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                               LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                               LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                               0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetTxConfig( MODEM_LORA,
                       TX_OUTPUT_POWER,
                       0,
                       LORA_BANDWIDTH,
                       LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE,
                       LORA_PREAMBLE_LENGTH,
                       LORA_FIX_LENGTH_PAYLOAD_ON,
                       true,
                       0,
                       0,
                       LORA_IQ_INVERSION_ON,
                       3000 );
}

void loop()
{
  // Enviar ACK si está pendiente (se construye aquí para evitar Radio.Send en IRQ)
  if (sendAckPending && lora_idle) {
      unsigned long pkt = ackPktNum; // copia local
      char ackBuf[32];
      int ackLen = snprintf(ackBuf, sizeof(ackBuf), "ACK:%lu", pkt);
      if (ackLen > 0) {
          Serial.printf("Sending ACK from loop -> %s\n", ackBuf);
          lora_idle = false;
          sendAckPending = false;
          ackBeingSent = pkt;            // marcar ACK en proceso
          Radio.Send((uint8_t*)ackBuf, ackLen);
      } else {
          sendAckPending = false;
      }
  }

  if(lora_idle) {
    lora_idle = false;
    Radio.Rx(0); // escuchar
  }

  Radio.IrqProcess();

  // Si hay dato recibido, procesar/decodificar y mostrar en TFT / Serial
  if (rxReady) {
      // Hacer copias locales de las variables volátiles
      unsigned long localPkt;
      int16_t localRssi;
      unsigned long localAckBeingSent;
      unsigned long localLastAckSent;
      bool localSendAckPending;

      noInterrupts();
      localPkt = ackPktNum;
      localRssi = lastRssi;
      localAckBeingSent = ackBeingSent;
      localLastAckSent = lastAckSent;
      localSendAckPending = sendAckPending;
      rxReady = false;
      interrupts();

      // Mostrar SOLO número de paquete y estado de comunicación en TFT
      char line1[32];
      snprintf(line1, sizeof(line1), "%lu", localPkt);
      tft.st7735_fill_rectangle(42, 20, 118, 20, ST7735_BLACK);         // limpiar área
      tft.st7735_write_str(42, 20, (String)line1);                      // escribir número de paquete
      
      char status[32];
      if (localAckBeingSent != 0 && localAckBeingSent == localPkt) {
          snprintf(status, sizeof(status), "Enviando ");
      } else if (localLastAckSent != 0 && localLastAckSent == localPkt) {
          snprintf(status, sizeof(status), "Transm.  ");
      } else if (localSendAckPending) {
          snprintf(status, sizeof(status), "OK       ");
      } else {
          snprintf(status, sizeof(status), "No ACK   ");
      }
      tft.st7735_fill_rectangle(42, 40, 118, 20, ST7735_BLACK);         // limpiar área
      tft.st7735_write_str(42, 40, (String)status);                     // escribir estado

      // Mostrar RSSI en la última línea del TFT
      char rssiStr[32];
      snprintf(rssiStr, sizeof(rssiStr), "%d dBm", localRssi);
      tft.st7735_fill_rectangle(53, 60, 107, 20, ST7735_BLACK);         // limpiar área RSSI
      tft.st7735_write_str(53, 60, (String)rssiStr);


      // También enviar por Serial (compacto)
      Serial.printf("Pkt:%lu RSSI:%d %s\n", localPkt, localRssi, status);
  }
}

// =====================
// Callbacks
// =====================
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    // Guardar copia segura del payload para procesar en loop()
    uint16_t copyLen = (size < (BUFFER_SIZE - 1)) ? size : (BUFFER_SIZE - 1);
    memcpy(rxpacket, payload, copyLen );
    rxpacket[copyLen] = '\0';

    lastRssi = rssi;
    rxSize = copyLen;

    Radio.Sleep();
    Serial.printf("\r\nreceived packet (len %d) RSSI %d\n", rxSize, lastRssi);

    // Extraer número de paquete rápido para ACK (primeros 2 bytes little-endian)
    if (copyLen >= 2) {
        uint16_t pktNum = (uint16_t)rxpacket[0] | ((uint16_t)rxpacket[1] << 8);
        ackPktNum = pktNum;
    } else {
        ackPktNum = 0;
    }

    // Marcar para envio de ACK desde loop() y para decodificar en loop()
    sendAckPending = true;
    rxReady = true;

    // permitir loop controlar radio
    lora_idle = true;
}

void OnTxDone(void)
{
    Serial.println("ACK TX done");
    // Si acabábamos de enviar un ACK, marcar último ACK enviado
    if (ackBeingSent != 0) {
        lastAckSent = ackBeingSent;
        ackBeingSent = 0;
    }
    Radio.Rx(0);
    lora_idle = true;
}

void OnTxTimeout(void)
{
    Serial.println("TX Timeout");
    Radio.Sleep();
    lora_idle = true;
}

void OnRxTimeout(void)
{
    Serial.println("RX Timeout");
    lora_idle = true;
}

void OnRxError(void)
{
    Serial.println("RX Error");
    lora_idle = true;
}