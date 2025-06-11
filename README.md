# SEN0158 / PAJ7025R3 IR Sensor Diagnostic Tool

A comprehensive ESP32 diagnostic tool for the **SEN0158** infrared positioning sensor (based on the **PAJ7025R3** chip). This tool helps verify sensor connectivity, initialization, and real-time data acquisition.

## Features

- **I²C Device Scanner** - Automatically detects connected I²C devices
- **Product ID Verification** - Validates sensor identity (expects 0x7025)
- **Initialization Sequence** - Configures sensor with proper startup commands
- **Real-time Data Polling** - Continuously reads 4-point IR coordinate data at 20Hz
- **Diagnostic Output** - Clear serial output with status indicators
- **Heartbeat LED** - Visual indicator of system operation

## Hardware Configuration

### Default Pin Assignment
```
SDA Pin:     GPIO 21
SCL Pin:     GPIO 22
I²C Speed:   100 kHz
Sensor Addr: 0x58
Baud Rate:   115200
```

### Sensor Specifications
- **Model**: SEN0158 (DFRobot) / PAJ7025R3 (PixArt Imaging)
- **Interface**: I²C
- **Detection Points**: 4 simultaneous IR points
- **Coordinate Range**: 0-1023 (X,Y)
- **Update Rate**: 20Hz (configurable)

## Quick Start

1. **Connect Hardware**:
   - Connect SEN0158 sensor to ESP32 I²C pins (SDA/SCL)
   - Power sensor appropriately
   
2. **Upload Code**:
   - Open `ir_sensor_check.ino` in Arduino IDE
   - Select your ESP32 board
   - Upload to ESP32

3. **Monitor Output**:
   - Open Serial Monitor at 115200 baud
   - Watch for device detection, initialization, and live data

## Expected Output

```
========  SEN0158 / PAJ7025R3  –  ESP32 Diagnostic  ========
I²C ready  SDA=21  SCL=22  @100000 Hz

[SCAN] Detecting I²C devices …
  • 0x58  <– SEN0158

[PID] Reading 0x02/0x03 …
  Product‑ID = 0x7025

[INIT] Sending 6‑byte sequence + extra 0x30‑08 …

Summary: PID OK  |  INIT OK
Polling 0x36 every 50 ms …
    2347 ms  P0=( 512, 384) P1=(   0,   0) P2=(   0,   0) P3=(   0,   0)
    2397 ms  P0=( 485, 392) P1=(   0,   0) P2=(   0,   0) P3=(   0,   0)
```

## Configuration Options

### User Settings (modify as needed)
```cpp
#define SDA_PIN        21      // I²C Data pin
#define SCL_PIN        22      // I²C Clock pin  
#define I2C_FREQ_HZ    100000  // I²C frequency (100kHz recommended)
#define SENSOR_ADDR    0x58    // SEN0158 I²C address
#define SERIAL_BAUD    115200  // Serial monitor baud rate
#define HEARTBEAT_LED  2       // Onboard LED pin (-1 to disable)
#define LOOP_DELAY_MS  50      // Polling interval (20Hz = 50ms)
```

## Troubleshooting

### No I²C Devices Found
- Check wiring connections
- Verify power supply to sensor
- Try different I²C pins
- Check pullup resistors (usually built-in on ESP32)

### Wrong Product ID
- Ensure you have a genuine SEN0158/PAJ7025R3
- Check I²C address (should be 0x58)
- Verify sensor is powered and responding

### Initialization Failure
- Check sensor power stability
- Try lower I²C frequency (50kHz)
- Ensure adequate startup delay

### No Data or Errors
- Verify initialization completed successfully
- Check for I²C bus conflicts
- Ensure proper sensor positioning/calibration

## Technical Details

### Data Format
Each frame contains 4 IR points with 10-bit X,Y coordinates:
- **Frame Size**: 16 bytes from register 0x36
- **Point Encoding**: 3 bytes per point + status bits
- **Coordinate Range**: 0-1023 for both X and Y axes

### Initialization Sequence
The sensor requires a specific 6-command initialization:
```
0x30,0x01  0x30,0x08  0x06,0x90
0x08,0xC0  0x1A,0x40  0x33,0x33
0x30,0x08  (go live)
```

## License

This diagnostic tool is provided as-is for educational and debugging purposes.