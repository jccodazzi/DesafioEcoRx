/* Heltec Automation Receive communication test example
 *
 * Function:
 * 1. Receive the same frequency band lora signal program
 *  
 * Description:
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */

#include "LoRaWan_APP.h"
#include "Arduino.h"


#define RF_FREQUENCY                                433000000 // Hz

#define TX_OUTPUT_POWER                             18        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                        24         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t txNumber;

int16_t rssi,rxSize;

bool lora_idle = true;

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    
    txNumber=0;
    rssi=0;
  
	RadioEvents.RxDone = OnRxDone;	
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                               LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                               LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                               0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
}



void loop()
{
  if(lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess( );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi_in, int8_t snr )
{
    // Guardar tamaño y contenido del payload (manteniendo buffer existente)
    rxSize = size;
    if (size >= BUFFER_SIZE) {
        // evitar overflow: truncar
        memcpy(rxpacket, payload, BUFFER_SIZE-1);
        rxpacket[BUFFER_SIZE-1] = '\0';
        rxSize = BUFFER_SIZE-1;
    } else {
        memcpy(rxpacket, payload, size);
        rxpacket[size] = '\0';
    }

    // Actualizar variable global de RSSI y poner el radio a sleep temporalmente (comportamiento original)
    rssi = rssi_in;
    Radio.Sleep();

    // Mostrar paquete recibido
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);

    // Intentar extraer el número de paquete enviado (txNumber).
    // El transmisor envía el identificador con formato "Pkt:<numero>" al inicio del payload.
    long extractedTxNum = -1;
    char *p = strstr(rxpacket, "Pkt:");
    if (p) {
        p += 4; // avanzar después de "Pkt:"
        extractedTxNum = strtol(p, NULL, 10);
    } else {
        // Fallback: buscar el último token numérico en la cadena
        char *lastSpace = strrchr(rxpacket, ' ');
        if (lastSpace) {
            extractedTxNum = strtol(lastSpace + 1, NULL, 10);
        }
    }

    // Si encontramos un número válido, preparamos y encolamos un ACK: "ACK:<txNumber>"
    if (extractedTxNum >= 0) {
        int n = snprintf(txpacket, BUFFER_SIZE, "ACK:%ld", extractedTxNum);
        if (n < 0) n = 0;
        if (n >= BUFFER_SIZE) {
            Serial.println("WARN: ACK truncado");
        } else {
            Serial.printf("Enviando ACK: \"%s\"\r\n", txpacket);

            // Marcar radio ocupado y enviar. OnTxDone / OnTxTimeout deben estar asignados
            lora_idle = false;
            Radio.Send((uint8_t *)txpacket, strlen(txpacket));
            // Confiamos en OnTxDone/OnTxTimeout para volver a poner lora_idle = true
        }
    } else {
        Serial.println("No se pudo extraer txNumber; no se envía ACK");
        lora_idle = true;
    }
}


// =====================================================
// Callbacks añadidos para transmisión (ACK)
// =====================================================
void OnTxDone(void) {
    Serial.println("OnTxDone: ACK transmitido.");
    lora_idle = true;
}

void OnTxTimeout(void) {
    Serial.println("OnTxTimeout: timeout en TX, forzando estado idle.");
    Radio.Sleep();
    lora_idle = true;
}
