/*
  Placa B - Verificador con ATECC508A y RFM95 LoRa
  
  Descripción: 
  - Envía desafíos por LoRa al Probador
  - Recibe firmas y datos GPS por LoRa
  - Verifica las firmas usando la clave pública del Probador
  - Procesa y muestra datos de ubicación GPS
  
  Hardware:
  - Arduino + RFM95 + ATECC508A
  
  Conexiones RFM95:
  CS -> Pin 8, INT -> Pin 3, RST -> Pin 4
  
  Conexiones ATECC508A:
  SDA -> A4, SCL -> A5 (I2C)
  
  IMPORTANTE: Actualizar el array publicKey[] con la clave real del Probador
*/

#include <SparkFun_ATECCX08a_Arduino_Library.h>
#include <Wire.h>
#include <Crypto.h> // Incluimos la librería Crypto.
#include <SHA256.h> // Se encuentra dentro de la librería Crypto.
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

// Clave para la que se verificará la firma (slot 0).
const uint8_t keyId_auth = 0;

// Variables para datos GPS recibidos
float receivedLatitude = 0.0;
float receivedLongitude = 0.0;
float receivedAltitude = 0.0;
int receivedSatellites = 0;
bool receivedGpsFix = false;
String receivedGpsTime = "";
String receivedGpsDate = "";

// Aquí se guardará la clave pública de la Placa A.
// Por favor, pega la clave pública real de la Placa A aquí.
const byte publicKey[64] = {
  0x38, 0x8E, 0xEA, 0x9F, 0xCD, 0xCA, 0x0C, 0xE5, 0x7C, 0x98, 0x08, 0xA0, 0xDE, 0xA8, 0xA4, 0x60, 
  0xA9, 0xAE, 0x30, 0x4A, 0x8C, 0xE9, 0x8C, 0xDC, 0xBE, 0xAF, 0xA6, 0xF8, 0x28, 0xC9, 0xD5, 0xC0, 
  0xB6, 0x4F, 0x80, 0x27, 0xAA, 0x51, 0xBD, 0x49, 0x08, 0x0B, 0x01, 0x26, 0x3F, 0x17, 0x9B, 0xAB, 
  0x0A, 0xCF, 0x08, 0xF7, 0x7C, 0xB5, 0xEE, 0xA0, 0x8C, 0x54, 0x69, 0xCB, 0x1B, 0x3F, 0x40, 0x68
};

/**
 * @brief Verifica una firma recibida con la clave pública del probador.
 * @param challengeBytes El desafío original en un array de bytes.
 * @param challengeLength La longitud del desafío.
 * @param signatureString La firma en formato de cadena hexadecimal.
 * @return true si la firma es válida, false en caso contrario.
 */
bool verifySignature(byte* challengeBytes, int challengeLength, String signatureString) {
  // Calculamos el hash SHA-256 del desafío.
  SHA256 hash_algo;
  uint8_t hash[32];
  hash_algo.reset();
  hash_algo.update(challengeBytes, challengeLength);
  hash_algo.finalize(hash, 32);

  // Convierte la firma de una cadena hexadecimal a un array de bytes.
  byte signature[64];
  for (int i = 0; i < 64; i++) {
    String hexByte = signatureString.substring(i * 2, (i * 2) + 2);
    signature[i] = (byte)strtol(hexByte.c_str(), NULL, 16);
  }

  // Llama a la función de la librería para verificar la firma, usando el hash.
  if (crypto.verifySignature(hash, signature, (uint8_t*)publicKey)) {
    return true;
  }
  
  return false;
}

/**
 * @brief Extrae un valor JSON de una cadena JSON simple
 * @param jsonString La cadena JSON completa
 * @param key La clave a buscar
 * @return El valor de la clave como String
 */
String extractJSONValue(String jsonString, String key) {
  String searchKey = "\"" + key + "\":";
  int startPos = jsonString.indexOf(searchKey);
  if (startPos == -1) return "";
  
  startPos += searchKey.length();
  int endPos = jsonString.indexOf(",", startPos);
  if (endPos == -1) {
    endPos = jsonString.indexOf("}", startPos);
  }
  if (endPos == -1) return "";
  
  return jsonString.substring(startPos, endPos);
}

/**
 * @brief Procesa los datos GPS recibidos en formato JSON
 * @param gpsJsonString La cadena JSON con los datos GPS
 */
void processGPSData(String gpsJsonString) {
  Serial.println("=== PROCESANDO DATOS GPS ===");
  
  // Extraer valores del JSON
  String fixStr = extractJSONValue(gpsJsonString, "fix");
  String latStr = extractJSONValue(gpsJsonString, "latitude");
  String lonStr = extractJSONValue(gpsJsonString, "longitude");
  String altStr = extractJSONValue(gpsJsonString, "altitude");
  String satStr = extractJSONValue(gpsJsonString, "satellites");
  String timeStr = extractJSONValue(gpsJsonString, "time");
  String dateStr = extractJSONValue(gpsJsonString, "date");
  
  // Convertir y almacenar valores
  receivedGpsFix = (fixStr == "true");
  receivedLatitude = latStr.toFloat();
  receivedLongitude = lonStr.toFloat();
  receivedAltitude = altStr.toFloat();
  receivedSatellites = satStr.toInt();
  
  // Limpiar comillas de tiempo y fecha
  timeStr.replace("\"", "");
  dateStr.replace("\"", "");
  receivedGpsTime = timeStr;
  receivedGpsDate = dateStr;
  
  // Mostrar datos procesados
  if (receivedGpsFix) {
    Serial.println("✅ SEÑAL GPS VÁLIDA");
    Serial.print("📍 Ubicación: ");
    Serial.print(receivedLatitude, 6);
    Serial.print(", ");
    Serial.println(receivedLongitude, 6);
    Serial.print("📏 Altitud: ");
    Serial.print(receivedAltitude);
    Serial.println(" m");
    Serial.print("🛰️ Satélites: ");
    Serial.println(receivedSatellites);
    Serial.print("🕐 Tiempo: ");
    Serial.println(receivedGpsTime);
    Serial.print("📅 Fecha: ");
    Serial.println(receivedGpsDate);
    
    // Calcular distancia aproximada (ejemplo: desde coordenadas de referencia)
    float refLat = 40.7128; // Nueva York como ejemplo
    float refLon = -74.0060;
    float distance = calculateDistance(receivedLatitude, receivedLongitude, refLat, refLon);
    Serial.print("📊 Distancia desde referencia: ");
    Serial.print(distance, 2);
    Serial.println(" km");
  } else {
    Serial.println("❌ SIN SEÑAL GPS");
    Serial.print("🛰️ Satélites visibles: ");
    Serial.println(receivedSatellites);
  }
  Serial.println("=============================");
}

/**
 * @brief Calcula la distancia entre dos puntos usando la fórmula de Haversine
 * @param lat1 Latitud del punto 1
 * @param lon1 Longitud del punto 1
 * @param lat2 Latitud del punto 2
 * @param lon2 Longitud del punto 2
 * @return Distancia en kilómetros
 */
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371; // Radio de la Tierra en km
  
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon/2) * sin(dLon/2);
  
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

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

  Serial.println("Placa B (Verificador) - Inicializando ATECC508A...");

  Wire.begin();
  if (!crypto.begin()) {
    Serial.println("Fallo en la inicialización. Revisa la conexión I2C.");
    while (1);
  }

  Serial.println("Chip ATECC508A inicializado.");
  Serial.println("Clave pública de la Placa A cargada.");
  Serial.println("Listo para verificar firmas y procesar datos GPS...");
}

void loop() {
  // Genera un desafío de ejemplo y lo convierte a un array de bytes.
  const char* challenge_str = "Hola desde la Placa B!";
  byte challengeBytes[256];
  int challengeLength = strlen(challenge_str);
  memcpy(challengeBytes, challenge_str, challengeLength);

  Serial.print("Enviando desafío por LoRa a la Placa A: ");
  Serial.println(challenge_str);

  // Enviar el desafío a través de LoRa
  if (rf95.send((uint8_t*)challenge_str, challengeLength)) {
    Serial.println("Desafío enviado por LoRa");
    rf95.waitPacketSent(); // Esperar a que se complete la transmisión
  } else {
    Serial.println("Error al enviar desafío por LoRa");
    delay(5000);
    return;
  }

  // Esperar respuesta por LoRa
  Serial.println("Esperando respuesta por LoRa...");
  
  // Timeout para esperar respuesta (10 segundos)
  unsigned long startTime = millis();
  bool responseReceived = false;
  
  while (millis() - startTime < 10000 && !responseReceived) {
    if (rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      
      if (rf95.recv(buf, &len)) {
        // Asegurar terminación de cadena
        buf[len] = '\0';
        String response = String((char*)buf);
        response.trim();
        
        Serial.print("Respuesta recibida por LoRa (RSSI: ");
        Serial.print(rf95.lastRssi(), DEC);
        Serial.print("): ");
        Serial.println(response);

        // Si la respuesta no está vacía, procesamos la firma y datos GPS
        if (response.length() > 0) {
          // Separar firma y datos GPS
          String signaturePart = "";
          String gpsPart = "";
          
          if (response.indexOf("|GPS:") != -1) {
            // La respuesta contiene datos GPS
            int gpsIndex = response.indexOf("|GPS:");
            signaturePart = response.substring(0, gpsIndex);
            gpsPart = response.substring(gpsIndex + 5); // +5 para saltar "|GPS:"
            
            Serial.println("📡 Respuesta contiene datos GPS");
          } else {
            // Solo firma sin GPS
            signaturePart = response;
            Serial.println("📡 Respuesta solo contiene firma");
          }
          
          // Extraer la firma (quitar "SIGNATURE:" si está presente)
          String signatureHex = signaturePart;
          if (signaturePart.startsWith("SIGNATURE:")) {
            signatureHex = signaturePart.substring(10);
          }
          
          // Verificar la firma
          if (verifySignature(challengeBytes, challengeLength, signatureHex)) {
            Serial.println("✅ VERIFICACIÓN EXITOSA: La firma es válida.");
            
            // Procesar datos GPS si están disponibles
            if (gpsPart.length() > 0) {
              processGPSData(gpsPart);
            }
          } else {
            Serial.println("❌ VERIFICACIÓN FALLIDA: La firma no es válida.");
          }
        }
        responseReceived = true;
      } else {
        Serial.println("Error al recibir mensaje LoRa");
      }
    }
    delay(100); // Pequeña pausa para no saturar el loop
  }
  
  if (!responseReceived) {
    Serial.println("⏰ Timeout: No se recibió respuesta de la Placa A");
  } else {
    // Mostrar resumen de la verificación
    Serial.println("📋 RESUMEN DE VERIFICACIÓN:");
    Serial.println("=============================");
    if (receivedGpsFix) {
      Serial.println("✅ Autenticación exitosa con datos GPS");
      Serial.print("📍 Ubicación del dispositivo: ");
      Serial.print(receivedLatitude, 6);
      Serial.print(", ");
      Serial.println(receivedLongitude, 6);
    } else {
      Serial.println("✅ Autenticación exitosa (sin datos GPS)");
    }
    Serial.println("=============================");
  }
  
  delay(5000); // Espera 5 segundos antes de enviar otro desafío.
}