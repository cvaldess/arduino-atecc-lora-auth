# Sistema de Autenticación ATECC508A con LoRa

Sistema de autenticación criptográfica usando chips ATECC508A y comunicación inalámbrica LoRa (RFM95).

## Descripción

Este proyecto implementa un sistema de autenticación de desafío-respuesta entre dos placas Arduino:

- **Placa A (Probador)**: Recibe desafíos y los firma usando ATECC508A
- **Placa B (Verificador)**: Envía desafíos y verifica las firmas recibidas

La comunicación se realiza mediante módulos RFM95 (LoRa) en la frecuencia 433.775 MHz.

## Hardware Requerido

### Para cada placa:
- **Adafruit Feather M0 LoRa** (con RFM95 integrado)
- **Chip ATECC508A** (con breakout board)
- **Antena LoRa** (915MHz para América o 433MHz para Europa)
- **Cables jumper**
- **Protoboard** (opcional)

## Conexiones

### RFM95 en Feather M0 LoRa:
```
¡El RFM95 ya está integrado! Solo conectar la antena.
```

### ATECC508A (para ambas placas):
| ATECC508A Pin | Feather M0 Pin | Función |
|---------------|----------------|---------|
| VCC           | 3V             | Alimentación |
| GND           | GND            | Tierra |
| SDA           | SDA            | I2C Data |
| SCL           | SCL            | I2C Clock |

### Configuración de frecuencia:
- **Frecuencia exacta**: 433.775 MHz ✅

## Librerías Necesarias

Instalar desde el Administrador de Librerías de Arduino:

1. `SparkFun ATECCX08a Arduino Library`
2. `Crypto`
3. `RadioHead` (para RFM95)

## Configuración

1. **Frecuencia específica**: `433.775 MHz`

2. **Potencia de transmisión**: 5-23 dBm (ajustar con `setTxPower()`)

3. **Clave pública**: Actualizar el array `publicKey[]` en el verificador con la clave real de la Placa A

4. **Configurar Arduino IDE para Feather M0**:
   - Instalar "Adafruit SAMD Boards"
   - Seleccionar "Adafruit Feather M0" como placa
   - Puerto: USB Native

5. **Antena**: Usar antena de 433MHz para mejor rendimiento

## Uso

1. Programar una placa con `probador-atecc-rfm95.ino`
2. Programar la otra placa con `verificador-atecc-rfm95.ino`
3. Abrir el Monitor Serie de ambas placas (115200 baudios)
4. El verificador enviará desafíos automáticamente cada 5 segundos
5. El probador responderá con firmas criptográficas
6. El verificador validará las firmas y mostrará el resultado

## Flujo de Operación

```
PLACA B (Verificador)              PLACA A (Probador)
---------------------              ------------------

1. Genera desafío        LoRa      2. Recibe desafío
   "Hola desde B!"   --------->       "Hola desde B!"

                                   3. Calcula SHA-256
                                      del desafío

                                   4. Firma hash con
                                      ATECC508A

5. Recibe firma          LoRa      6. Envía firma
   (128 chars hex)   <---------       (64 bytes)

7. Verifica firma
   con clave pública

8. Muestra resultado:
   "VERIFICACION EXITOSA" 
   o "VERIFICACION FALLIDA"
```

## Características

- ✅ Comunicación LoRa de largo alcance
- ✅ Autenticación criptográfica segura
- ✅ Hash SHA-256 de los desafíos
- ✅ Firmas ECDSA con ATECC508A
- ✅ Indicador RSSI de calidad de señal
- ✅ Manejo de timeouts y errores
- ✅ Monitor serie detallado

## Troubleshooting

### LoRa no inicializa:
- Verificar conexiones SPI
- Comprobar alimentación 3.3V
- Revisar continuidad en cables

### ATECC508A no responde:
- Verificar conexiones I2C
- Comprobar dirección I2C (generalmente 0x60)
- Asegurar alimentación estable

### Firmas no válidas:
- Verificar que la clave pública en el verificador sea correcta
- Comprobar que ambos chips usen la misma clave privada (slot 0)

## Parámetros Modificables

```cpp
// Frecuencia LoRa
#define RF95_FREQ 433.775

// Potencia de transmisión (5-23 dBm)
rf95.setTxPower(5, false);

// Timeout de respuesta (ms)
unsigned long timeout = 10000;

// Intervalo entre desafíos (ms)
delay(5000);
```

## Licencia

MIT License - Libre uso y modificación

---

**Autor**: Sistema desarrollado para comunicación segura IoT
**Fecha**: $(date)
