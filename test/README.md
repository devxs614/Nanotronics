# Hardware test plan

This directory contains hardware test skeletons for the robot subsystem validation flow.

Recommended order:
1. Blink
2. Digital pins
3. Motor 1..4 and drivetrain
4. Encoder checks
5. PID tuning
6. IMU validation
7. VL53L0X checks
8. TCS34725 classification
9. QTR-8A detection
10. Servo and gripper
11. OLED smoke test
12. Full integration test

# Bitácora de desarrollo y resolución de problemas

## 1. Módulo de actuadores y motores DC (TB6612FNG)

### Problema detectado

Durante las primeras pruebas de movimiento con los controladores TB6612FNG, dos de los cuatro motores (específicamente Motor 2 y Motor 3) no respondían a los comandos PWM enviando señales de dirección correctas, o su comportamiento era intermitente al cambiar el sentido de giro.

### Análisis y solución

1. **Verificación eléctrica**: Utilicé un multímetro para medir el voltaje de salida en las terminales $A01/A02$ y $B01/B02$ del driver, confirmando que la caída de tensión no provenía del regulador de potencia, sino de una señal lógica flotante.
2. **Mapeo de pines y lógica directa**: Revisé la asignación de pines en el archivo `test/motor_test.cpp`, re-mapeando las salidas PWM y las líneas de dirección (`IN1`/`IN2`) hacia pines con soporte de temporizador dedicado en el Arduino Mega 2560.
3. **Validación**: Con un script modular aislado en PlatformIO, verifiqué el comportamiento individual de cada puente H en ambos sentidos a diferentes ciclos de trabajo ($0–255$).

---

## 2. Odometría y diagnóstico de interrupciones en encoders de cuadratura

### Problema detectado

Al implementar la lectura de velocidad e impulsos de los cuatro encoders de cuadratura, los motores M1, M2 y M3 registraban conteos precisos mediante interrupciones por cambio de pin (*Pin Change Interrupts - PCINT*). Sin embargo, el Motor 4 no registraba ningún impulso al girar su eje.

### Análisis de arquitectura del ATmega2560

Al auditar el mapeo físico de los pines asignados al Motor 4 (`D16` y `D17`), identificué que pertenecen al **Puerto H (`PH0` y `PH1`)** del microcontrolador ATmega2560. A diferencia de los puertos B y J, el Puerto H **carece de hardware para vectores PCINT (`PCINTx`)**. Intentar leer estos pines por *polling* dentro de la función `loop()` causaba una pérdida masiva de *ticks* cuando el motor giraba a revoluciones altas.

### Solución implementada

1. **Re-enrutamiento físico**: Reasigné las líneas de lectura del encoder del Motor 4 desde los pines `D16/D17` hacia los pines **`D18` (INT3)** y **`D19` (INT2)**.
2. **Aprovechamiento de interrupciones nativas**: Configuré la lectura mediante `attachInterrupt(digitalPinToInterrupt(18), ISR_M4, RISING)`, utilizando las interrupciones hardware dedicadas del chip en lugar del registro de puerto.
3. **Resultado**: Logré una lectura determinista a alta frecuencia en los cuatro canales de odometría con cero pérdida de pulsos.

---

## 3. Manejo del módulo seguidor de línea (QTR-8A)

### Problema detectado

Al actualizar las librerías a la versión `QTRSensors` v4.x de Pololu, la sintaxis heredada utilizada en las pruebas iniciales arrojaba errores de compilación por constantes obsoletas como `QTR_EMITTERS_ON`.

### Solución implementada

1. **Modernización del script**: Reescribí el módulo de prueba `test/qtr_test.cpp` adaptándolo a la API v4.x.
2. **Configuración analógica**: Definí explícitamente el arreglo de lectura con `qtr.setTypeAnalog()` y el mapeo de los 8 canales analógicos del Arduino Mega (`A0` a `A7`).
3. **Optimización de lectura**: Simplifiqué la función de captura a `qtr.read(sensorValues)`, aislando las lecturas analógicas brutas para su posterior normalización mediante `qtr.calibrate()`.

---

## 4. Conflicto de direcciones en el bus I2C (3x VL53L0X + TCS34725)

### El problema técnico

El robot integra tres sensores de distancia Time-of-Flight (VL53L0X: Frontal, Derecho, Izquierdo) y un sensor de color RGB (TCS34725). Al realizar un escaneo I2C inicial, la terminal solo detectaba un dispositivo en la dirección `0x29`.

Al revisar las hojas de datos de los componentes, descubrí que **tanto el TCS34725 como los tres sensores VL53L0X vienen configurados de fábrica con la misma dirección I2C por defecto (`0x29`)**. Al estar todos energizados simultáneamente en el bus (SDA `D20` / SCL `D21`), las transmisiones colisionaban y el bus quedaba completamente bloqueado.

### Intentos fallidos y diagnóstico profundo

1. **Re-direccionamiento por software simple**: Intenté ejecutar la rutina estándar `lox.begin(NUEVA_DIRECCION)` encendiendo los pines `XSHUT` uno a uno. Sin embargo, los sensores de distancia seguían fallando con el mensaje `[FALLO]`.
2. **Descubrimiento del conflicto con el sensor de color**: Al desconectar físicamente el sensor de color TCS34725 del bus, los tres sensores de distancia VL53L0X lograban cambiar de dirección a `0x30`, `0x31` y `0x32` sin ningún problema.
3. **Causa raíz**:
* La librería `Adafruit_VL53L0X` envía comandos broadcast a la dirección `0x29` durante su inicialización para ordenar el cambio de dirección.
* Como el TCS34725 **no tiene pin `XSHUT**` para apagarlo por software y sus resistencias de *pull-up* integradas mantenían la impedancia del bus baja, el sensor de color respondía y corrompía los paquetes que iban dirigidos a los VL53L0X.



### Control de alimentación dinámica por software

Dado que no era posible apagar el TCS34725 por software ni cambiar su dirección fija de fábrica (`0x29`), diseñé una estrategia de control de energía e inicialización por etapas:

1. **Modificación de hardware**: Desconecté el pin de alimentación (`VIN/VCC`) del sensor TCS34725 de la línea constante de 5V y lo conecté directamente al pin digital **`D28`** del Arduino Mega.
2. **Conexión de líneas XSHUT**: Conecté las líneas de reset de los VL53L0X a pines digitales independientes:
* Frontal $\rightarrow$ **`D24`**
* Derecho $\rightarrow$ **`D22`**
* Izquierdo $\rightarrow$ **`D26`**


3. **Algoritmo de arranque secuencial en `setup()**`:
* **Paso A**: Puse en estado `LOW` el pin `D28` ($0\text{V}$, desenergizando por completo el TCS34725) y puse en `LOW` los pines `D22`, `D24` y `D26` (manteniendo los tres VL53L0X en reset).
* **Paso B**: Puse en `HIGH` el pin `D24`. Inicialicé el sensor en `0x29` y de inmediato le asigné la nueva dirección **`0x30`**.
* **Paso C**: Puse en `HIGH` el pin `D22`. Inicialicé el sensor en `0x29` y le asigné la dirección **`0x31`**.
* **Paso D**: Puse en `HIGH` el pin `D26`. Inicialicé el sensor en `0x29` y le asigné la dirección **`0x32`**.
* **Paso E**: Habiendo movido los tres sensores de distancia a direcciones privadas (`0x30`, `0x31`, `0x32`), la dirección `0x29` quedó 100% libre. En ese momento, conmuté el pin `D28` a `HIGH` ($5\text{V}$) para energizar el TCS34725 y llamé a `tcs.begin(0x29, &Wire)`.



### Resultado

Los 4 sensores I2C conviven de forma estable en el mismo bus de datos, permitiendo lecturas simultáneas y en tiempo real de distancia (en mm) y color (valores RGB) sin ningún tipo de colisión.

---

## 5. Gestión del entorno de desarrollo y estructura de proyecto (PlatformIO)

### Desafío

Probar múltiples periféricos de forma individual sin generar conflictos de redefinición de funciones principales (`setup()` y `loop()`) en C/C++ ni ensuciar la carpeta principal `src/main.cpp`.

### Solución

Aproveché la directiva **`build_src_filter`** en el archivo de configuración `platformio.ini`:

* Creé una carpeta `test/` donde almacené cada script de diagnóstico de forma aislada (`qtr_test.cpp`, `vl53_color_test.cpp`, etc.).
* Utilicé la sintaxis de inclusión y exclusión (`+<../test/script_actual.cpp> -<main.cpp>`) para cambiar de prueba en cuestión de segundos, acelerando drásticamente el proceso de desarrollo y compilación antes de la integración final.