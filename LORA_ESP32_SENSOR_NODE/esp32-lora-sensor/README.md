# ESP32 LoRa Sensor Node

An Arduino/ESP32 project for reading sensor data and transmitting via LoRa radio.

## Project Structure

```
esp32-lora-sensor/
├── esp32-lora-sensor.ino   # Main sketch
├── arduino-cli.yaml         # Arduino CLI configuration
└── README.md               # This file
```

## Compile

```bash
arduino-cli compile -b esp32:esp32:esp32 ./esp32-lora-sensor
```

## Upload

```bash
arduino-cli upload -b esp32:esp32:esp32 -p COM3 ./esp32-lora-sensor
```

## Monitor

```bash
arduino-cli monitor -b esp32:esp32:esp32 -p COM3 -c baud=115200
```

## Configuration

Edit `arduino-cli.yaml` to set your COM port and other options.

## LoRa Hardware

The sketch includes pin definitions for common ESP32 LoRa boards (Heltec, TTGO). Modify the pin definitions in the `.ino` file for your specific hardware.