/*
  Placa A - Probador con ATECC508A, RFM95 LoRa y GPS
  
  Descripción: 
  - Recibe desafíos por LoRa desde el Verificador
  - Firma los desafíos usando ATECC508A
  - Envía las firmas de vuelta por LoRa
  - Obtiene datos de ubicación GPS
  
  Hardware:
  - Arduino + RFM95 + ATECC508A + Adafruit Ultimate GPS FeatherWing
  
  Conexiones RFM95:
  CS -> Pin 8, INT -> Pin 3, RST -> Pin 4
  
  Conexiones ATECC508A:
  SDA -> A4, SCL -> A5 (I2C)
  
  Conexiones GPS FeatherWing:
  Conecta directamente al header del Feather M0
  RX -> Pin 1, TX -> Pin 0
*/

#include <SparkFun_ATECCX08a_Arduino_Library.h>
#include <Wire.h>
#include <Crypto.h>
#include <SHA256.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>

// Pines para Adafruit Feather M0 LoRa (RFM95 integrado)
#define RFM95_CS    8
#define RFM95_INT   3  
#define RFM95_RST   4
#define RF95_FREQ 433.775  // Frecuencia específica

// GPS usando Serial1 en Feather M0
#define GPSSerial Serial1

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Instancia del chip CryptoAuth.
ATECCX08A crypto;

// Instancia del GPS
Adafruit_GPS GPS(&GPSSerial);

// Clave para autenticación (slot 0).
const uint8_t keyId_auth = 0;

// Variables para datos GPS
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;
int satellites = 0;
bool gpsFix = false;
String gpsTime = "";
String gpsDate = "";

// Timer para actualización GPS
unsigned long lastGPSTime = 0;
const unsigned long GPS_UPDATE_INTERVAL = 1000; // Actualizar GPS cada segundo

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
  
  // Inicializar GPS
  Serial.println("Inicializando GPS...");
  GPSSerial.begin(9600);
  GPS.begin(9600);
  
  // Configurar GPS para enviar solo los datos que necesitamos
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  
  Serial.println("GPS inicializado. Esperando señal de satélites...");
  Serial.println("Listo para firmar desafíos de la Placa B por LoRa y obtener datos GPS.");
}

void loop() {
  // Actualizar datos GPS cada segundo
  if (millis() - lastGPSTime > GPS_UPDATE_INTERVAL) {
    updateGPS();
    lastGPSTime = millis();
    
    // Mostrar datos GPS cada 5 segundos
    static unsigned long lastGPSPrint = 0;
    if (millis() - lastGPSPrint > 5000) {
      printGPSData();
      lastGPSPrint = millis();
    }
  }
  
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

        // Crear respuesta con firma y datos GPS
        String response = "SIGNATURE:" + signatureHex;
        if (gpsFix) {
          response += "|GPS:" + getGPSDataJSON();
        }
        
        // Envía la firma y datos GPS a la Placa B por LoRa
        if (rf95.send((uint8_t*)response.c_str(), response.length())) {
          Serial.println("Firma y datos GPS enviados por LoRa a la Placa B.");
          Serial.println("Respuesta completa: " + response);
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

// Función para actualizar datos GPS
void updateGPS() {
  // Leer datos del GPS
  char c = GPS.read();
  
  // Si hay una sentencia completa disponible
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return; // Fallo en el parsing, salir
    }
  }
  
  // Actualizar variables GPS
  if (GPS.fix) {
    gpsFix = true;
    latitude = GPS.latitude;
    longitude = GPS.longitude;
    altitude = GPS.altitude;
    satellites = GPS.satellites;
    
    // Formatear tiempo y fecha
    gpsTime = String(GPS.hour, DEC) + ":" + 
              String(GPS.minute, DEC) + ":" + 
              String(GPS.seconds, DEC);
    gpsDate = String(GPS.day, DEC) + "/" + 
              String(GPS.month, DEC) + "/" + 
              String(GPS.year, DEC);
  } else {
    gpsFix = false;
  }
}

// Función para imprimir datos GPS
void printGPSData() {
  Serial.println("=== DATOS GPS ===");
  if (gpsFix) {
    Serial.print("Ubicación: ");
    Serial.print(latitude, 6);
    Serial.print(", ");
    Serial.println(longitude, 6);
    Serial.print("Altitud: ");
    Serial.print(altitude);
    Serial.println(" m");
    Serial.print("Satélites: ");
    Serial.println(satellites);
    Serial.print("Tiempo: ");
    Serial.println(gpsTime);
    Serial.print("Fecha: ");
    Serial.println(gpsDate);
  } else {
    Serial.println("Sin señal GPS - buscando satélites...");
    Serial.print("Satélites visibles: ");
    Serial.println(satellites);
  }
  Serial.println("================");
}

// Función para obtener datos GPS como JSON
String getGPSDataJSON() {
  String json = "{";
  json += "\"fix\":" + String(gpsFix ? "true" : "false") + ",";
  json += "\"latitude\":" + String(latitude, 6) + ",";
  json += "\"longitude\":" + String(longitude, 6) + ",";
  json += "\"altitude\":" + String(altitude) + ",";
  json += "\"satellites\":" + String(satellites) + ",";
  json += "\"time\":\"" + gpsTime + "\",";
  json += "\"date\":\"" + gpsDate + "\"";
  json += "}";
  return json;
}