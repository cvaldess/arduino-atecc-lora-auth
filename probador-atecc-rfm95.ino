/*
  Placa A - Probador con ATECC508A y RFM95 LoRa
  
  Descripción: 
  - Recibe desafíos por LoRa desde el Verificador
  - Firma los desafíos usando ATECC508A
  - Envía las firmas de vuelta por LoRa
  
  Hardware:
  - Arduino + RFM95 + ATECC508A
  
  Conexiones RFM95:
  CS -> Pin 8, INT -> Pin 3, RST -> Pin 4
  
  Conexiones ATECC508A:
  SDA -> A4, SCL -> A5 (I2C)
*/

#include <SparkFun_ATECCX08a_Arduino_Library.h>
#include <Wire.h>
#include <Crypto.h>
#include <SHA256.h>
#include <SPI.h>
#include <RH_RF95.h>

// Pines para Adafruit Feather M0 LoRa (RFM95 integrado)
#define RFM95_CS    8
#define RFM95_INT   3  
#define RFM95_RST   4
#define RF95_FREQ 433.775  // Frecuencia específica

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Instancia del chip CryptoAuth.
ATECCX08A crypto;

// Clave para autenticación (slot 0).
const uint8_t keyId_auth = 0;

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  Serial.begin(115200);   // Para el Serial Monitor (USB nativo en M0)
  while (!Serial) {
    delay(100);
  }
  // Nota: El Feather M0 usa Serial nativo USB, no Serial1

  // Inicializar RFM95
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Configurar frecuencia
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Configurar potencia de transmisión
  rf95.setTxPower(5, false);

  Serial.println("Placa A (Firmador) - Inicializando ATECC508A...");

  Wire.begin();
  if (!crypto.begin()) {
    Serial.println("Fallo en la inicialización. Revisa la conexión I2C.");
    while (1);
  }

  Serial.println("Chip ATECC508A inicializado.");
  Serial.println("Listo para firmar desafíos de la Placa B por LoRa.");
}

void loop() {
  // Verificar si hay mensajes LoRa disponibles
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len)) {
      // Asegurar terminación de cadena
      buf[len] = '\0';
      String challengeString = String((char*)buf);
      challengeString.trim();

      Serial.print("Desafío recibido por LoRa (RSSI: ");
      Serial.print(rf95.lastRssi(), DEC);
      Serial.print("): ");
      Serial.println(challengeString);

      // Convierte el desafío de String a un array de bytes.
      byte challengeBytes[256];
      challengeString.getBytes(challengeBytes, challengeString.length() + 1);
      
      // Calcula el hash SHA-256 del desafío.
      SHA256 hash_algo;
      uint8_t hash[32];
      hash_algo.reset();
      hash_algo.update(challengeBytes, challengeString.length());
      hash_algo.finalize(hash, 32);

      // Firma el hash con la clave privada del chip.
      // La función correcta es createSignature() y solo acepta dos argumentos.
      if (crypto.createSignature(hash, keyId_auth)) {
        Serial.print("Firma generada: ");
        String signatureHex = "";
        for (int i = 0; i < 64; i++) {
          if (crypto.signature[i] < 0x10) {
            Serial.print("0");
            signatureHex += "0";
          }
          Serial.print(crypto.signature[i], HEX);
          signatureHex += String(crypto.signature[i], HEX);
        }
        Serial.println();

        // Envía la firma a la Placa B por LoRa
        if (rf95.send((uint8_t*)signatureHex.c_str(), signatureHex.length())) {
          Serial.println("Firma enviada por LoRa a la Placa B.");
          rf95.waitPacketSent(); // Esperar a que se complete la transmisión
        } else {
          Serial.println("Error al enviar firma por LoRa.");
        }
        
      } else {
        Serial.println("Fallo al firmar el hash.");
        
        // Enviar mensaje de error por LoRa
        String errorMsg = "ERROR_SIGNATURE";
        rf95.send((uint8_t*)errorMsg.c_str(), errorMsg.length());
        rf95.waitPacketSent();
      }
    } else {
      Serial.println("Error al recibir mensaje LoRa");
    }
  }
  
  delay(100); // Pequeña pausa para no saturar el loop
}