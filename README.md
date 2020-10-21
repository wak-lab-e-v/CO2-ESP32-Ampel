# CO2ESP32AMPEL
Wir verwenden ein DOIT Esp32 DevKit v1 mit einem Sensirion SCD30 Sensormodul und einem BME680 für die Luftdruckmessung. Den Aktuellen Luftdruck benötigt der SCD30 um kalibrierte Werte auszugeben.

Der SCD30 ist über GPIO22 I2C SCL und GPIO 21 I2C SDA verbunden.

## Installation ESP32 für Arduino IDE

* Verzeichnis ~/Arduino/hardware/espressif erstellen und in das Verzeichnis wechseln
* git clone https://github.com/espressif/arduino-esp32.git esp32
* In das Verzeichnis ~/Arduino/hardware/espressif/esp32 wechseln
* git submodule update --init --recursive
* In das Verzeichnis ~/Arduino/hardware/espressif/esp32/tools wechseln
* python get.py oder get.exe ausführen

