# Next Gen communication protocol

## Introduction

## Message structure

### Standard structure / Request

<table style="text-align: center">
<tr style="border-top-style:hidden; border-left-style: hidden; border-right-style: hidden; color: white">
<td></td>
<td style="background-color: indigo; border-style:hidden;" colspan="7">ESCAPED</td>
<td></td>
</tr>
<tr style="background-color: dimgray; color: white">
<td>START</td>
<td colspan="2">CMD</td>
<td colspan="2">DATA LENGTH</td>
<td>DATA</td>
<td colspan="2">CRC</td>
<td>END</td>
</tr>
<td>STX</td>
<td>CMD</td>
<td>SUBCMD</td>
<td>MSB</td>
<td>LSB</td>
<td>DL BYTES</td>
<td>MSB</td>
<td>LSB</td>
<td>ETX</td>
</table>

### ACK response

<table style="text-align: center">
<tr style="border-top-style:hidden; border-left-style: hidden; border-right-style: hidden; color: white">
<td></td>
<td style="background-color: indigo; border-style:hidden;" colspan="9">ESCAPED</td>
<td></td>
</tr>
<tr style="background-color: dimgray; color: white">
<td>START</td>
<td colspan="2">CMD</td>
<td colspan="2">DATA LENGTH</td>
<td colspan="3">DATA</td>
<td colspan="2">CRC</td>
<td>END</td>
</tr>
<td>STX</td>
<td>0x00</td>
<td>0x06</td>
<td>MSB</td>
<td>LSB</td>
<td>CMD</td>
<td>SUBCMD</td>
<td>RESPONSE DATA</td>
<td>MSB</td>
<td>LSB</td>
<td>ETX</td>
</table>

### NACK response

<table style="text-align: center">
<tr style="border-top-style:hidden; border-left-style: hidden; border-right-style: hidden; color: white">
<td></td>
<td style="background-color: indigo; border-style:hidden;" colspan="9">ESCAPED</td>
<td></td>
</tr>
<tr style="background-color: dimgray; color: white">
<td>START</td>
<td colspan="2">CMD</td>
<td colspan="2">DATA LENGTH</td>
<td colspan="3">DATA</td>
<td colspan="2">CRC</td>
<td>END</td>
</tr>
<td>STX</td>
<td>0x00</td>
<td>0x15</td>
<td>0x00</td>
<td>0x03</td>
<td>CMD</td>
<td>SUBCMD</td>
<td>REASON CODE</td>
<td>MSB</td>
<td>LSB</td>
<td>ETX</td>
</table>

> **_Warning_:**
> When NACK reason is NG_NACK_REASON_CRCError or NG_NACK_REASON_MessageToLong CMD/SUBCOMMAND is 0xFFFFu as we can't be sure we received a valid command

## NACK Reason Codes

| Reason Code | Description                             | Type                 |
|:-----------:|:----------------------------------------|:---------------------|
|    0x00     | OK (Reserved for internal firmware use) | Protocol             |
|    0x01     | CRC Error                               | Protocol             |
|    0x02     | Invalid data byte length                | Protocol             |
|    0x03     | Invalid message length                  | Protocol             |
|    0x04     | Message to long                         | Protocol             |
|    0x05     | Invalid/Unknown command                 | Protocol             |
|    0x06     | Invalid device state                    | Protocol/Application |
|    0x07     | Invalid data                            | Application          |
|    0x08     | Command not implemented                 | Application          |
|    0x09     | Test failed                             | Application          |
|    0x0A     | Test Not Finished                       | Application          |
|    0x0B     | NVM Error                               | Application          |
|    0x0C     | Timeout                                 | Application          |
|    0x0D     | Buffer Overflow                         | Application          |
|    0x0E     | Hardware Error                          | Application          |
|    0xFD     | SDK Error                               | Application          |
|    0xFE     | Internal Error                          | Application          |

## Command set

The command should never be used for another function. A standard list of command/subcommand combinations needs to be created

### Command group 0x00__ is reserved for system function

|  SUBCMD  | Description                                                          | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT | P0302_FCT |
|:--------:|:---------------------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|-----------|
|   0x00   | Illegal                                                              |
|   0x01   | [Protocol Enter](#0x0001-protocol-enter)                             |           | &#10004;  |    &#10004;    |                |           |
|   0x02   | [Protocol Exit](#0x0002-protocol-exit)                               |           | &#10004;  |    &#10004;    |                |           |
|   0x03   | [Bootloader Enter](#0x0003-bootloader-enter)                         | &#10004;  | &#10004;  |    &#10004;    |                |           |
|   0x04   | [Protocol Enter with Password](#0x0004-protocol-enter-with-password) |           |           |                |                |           |
| **0x06** | **ACK**                                                              | &#10004;  | &#10004;  |    &#10004;    |                |           |
| **0x15** | **NACK**                                                             | &#10004;  | &#10004;  |    &#10004;    |                |           |
|   0x20   | [Read Serial Number](#0x0020-read-serial-number)                     |           | &#10004;  |                |                |           |
|   0x21   | [Read Firmware Number SAxxxx](#0x0021-read-firmware-number-saxxxx)   |           | &#10004;  |                |                |           |
|   0x22   | [Read Firmware Version](#0x0022-read-firmware-version)               |           | &#10004;  |                |                |           |
|   0x23   | [Read HW Revision](0x0023-read-hw-revision)                          |           | &#10004;  |                |                |           |
|   0x24   | [Write Serial Number](#0x0024-write-serial-number)                   |           |           |    &#10004;    |                |           |
|   0x25   | [Read MAC Address (EUI48)](#0x0025-read-mac-address)                 |           |           |                |                | &#10004;  |
|   0x26   | [Write MAC Address (EUI48)](#0x0026-write-mac-address)               |           |           |                |                | &#10004;  |
|   0x27   | [Reset Timeout](#0x0027-reset-timeout)                               |           |           |                |                |           |
|   0xFF   | Illegal                                                              |

> **_NOTE:_**
> CMD group 0x00__ is reserved for system command

### Command group 0x01__ Timers

| SUBCMD | Description                                    | P0200 CAL | P0200 FTM | P0200 FCM | P0200 MCU2 FCT |
|:------:|:-----------------------------------------------|:---------:|:---------:|:---------:|----------------|
|  0x00  | Illegal                                        |
|  0x01  | [Set System Time](#0x0101-set-system-time)     |           | &#10004;  |           |                |
|  0x02  | [Reset System Time](#0x0102-reset-system-time) |           | &#10004;  |           |                |
|  0x03  | [Increase Rate](#0x0103-increase-rate)         |           | &#10004;  |           |                |
|  0xFF  | Illegal                                        |

### Command group 0x02__ Demount

| SUBCMD | Description                              | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-----------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                  |
|  0x01  | [Change Status](#0x0201-change-status)   |           | &#10004;  |                |                |
|  0x02  | [Simulate State](#0x0202-simulate-state) |           | &#10004;  |                |                |
|  0xFF  | Illegal                                  |

### Command group 0x03__ Soiling

| SUBCMD | Description                                                      | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-----------------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                          |
|  0x01  | [Set State](#0x0301-set-state)                                   |           | &#10004;  |                |                |
|  0x02  | [Set Periodicity](#0x0302-set-periodicity)                       |           | &#10004;  |                |                |
|  0x03  | [Simulate Soil Level](#0x0303-simulate-soil-level)               |           | &#10004;  |                |                |
|  0x04  | [Get Background Measurement](#0x0304-get-background-measurement) |           |           |    &#10004;    |                |
|  0x05  | [Get Active Measurement](#0x0305-get-active-measurement)         |           |           |    &#10004;    |                |
|  0x06  | [End Soiling Test](#0x0306-end-soil-test)                        |           |           |                |                |
|  0xFF  | Illegal                                                          |

### Command group 0x04__ CO

| SUBCMD | Description                                                                  | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-----------------------------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                                      |
|  0x01  | [Set State](0x0401-set-state)                                                |           | &#10004;  |                |                |
|  0x02  | [Set BIST Periodicity](#0x0402-set-bist-periodicity)                         |           | &#10004;  |                |                |
|  0x03  | [Set Measurement Periodicity](#0x0403-set-measurement-periodicity)           |           | &#10004;  |                |                |
|  0x04  | [Set Mute Status](#0x0404-set-mute-status)                                   |           | &#10004;  |                |                |
|  0x05  | [Simulate CO Level](#0x0405-simulate-co-level)                               |           | &#10004;  |                |                |
|  0x06  | [Read CO](#0x0406-read-co)                                                   | &#10004;  |           |                |                |
|  0x07  | [Read CO CF](#0x0407-read-co-cf)                                             | &#10004;  |           |                |                |
|  0x08  | [Write CO CF](#0x0408-write-co-cf)                                           | &#10004;  |           |                |                |
|  0x09  | [Write CO Calibration](#0x0409-write-co-calibration)                         | &#10004;  |           |                |                |
|  0x0A  | [Read CO Calibration](#0x040a-read-co-calibration)                           | &#10004;  |           |                |                |
|  0x0B  | [Run CO BIST](#0x040b-run-co-bist)                                           |           |           |                |                |
|  0x0C  | [Run CO Sensitivity Test](#0x040c-run-co-sensitivity-test)                   |           |           |                |                |
|  0x0D  | [Read CO Sensitivity Test Results](#0x040d-read-co-sensitivity-test-results) |           |           |                |                |
|  0x0E  | [Stop CO Sensitivity Tet](#0x040e-stop-co-sensitivity-test)                  |           |           |                |                |
|  0xFF  | Illegal                                                                      |

### Command group 0x05__ Smoke Detection

| SUBCMD | Description                                                                      | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:---------------------------------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                                          |
|  0x01  | [Set State](#0x0501-set-state)                                                   |           | &#10004;  |                |                |
|  0x02  | [Set Measurement Periodicity](#0x0502-set-measurement-periodicity)               |           | &#10004;  |                |                |
|  0x03  | [Set BIST Periodicity](#0x0503-set-bist-periodicity)                             |           | &#10004;  |                |                |
|  0x04  | [Set Smoke Chamber BIST Periodicity](#0x0504-set-smoke-chamber-bist-periodicity) |           | &#10004;  |                |                |
|  0x05  | [Simulate Smoke Level](#0x0505-simulate-smoke-level)                             |           | &#10004;  |                |                |
|  0x06  | [Set Mute Status](#0x0506-set-mute-state)                                        |           | &#10004;  |                |                |
|  0x07  | [Read Smoke Threshold](0x0507-read-smoke-threshold)                              | &#10004;  |           |                |                |
|  0x08  | [Write Smoke Threshold](0x0508-write-smoke-threshold)                            | &#10004;  |           |                |                |
|  0x09  | [Execute Clean Air Calibration](#0x0509-execute-clean-air-calibration)           |           |           |    &#10004;    |                |
|  0x0A  | [Get Background Measurement](#0x050a-get-background-measurement)                 |           |           |    &#10004;    |                |
|  0x0B  | [Get Active Measurement](#0x050b-get-active-measurement)                         |           |           |    &#10004;    |                |
|  0x0C  | [Run BIST](#0x050c-run-bist)                                                     |           |           |                |                |
|  0x0D  | [Start Sensitivity Test](#0x050d-start-sensitivity-test)                         |           |           |                |                |
|  0x0E  | [Stop Sensitivity Test](#0x050e-stop-sensitivity-test)                           |           |           |                |                |
|  0x0F  | [Read Sensitivity Test Data](#0x050f-read-sensitivity-test-data)                 |           |           |                |                |
|  0xFF  | Illegal                                                                          |

### Command group 0x06__ Heat Detection

| SUBCMD | Description                                                    | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:---------------------------------------------------------------|:---------:|:---------:|:--------------:|:--------------:|
|  0x00  | Illegal                                                        |
|  0x01  | [Set State](#0x0601-set-state)                                 |           | &#10004;  |                |                |
|  0x02  | [Set Detection Periodicity](#0x0602-set-detection-periodicity) |           | &#10004;  |                |                |
|  0x03  | [Set BIST Periodicity](#0x0603-set-bist-periodicity)           |           | &#10004;  |                |                |
|  0x04  | [Set Mute State](#0x0604-set-mute-state)                       |           | &#10004;  |                |                |
|  0x05  | [Simulate Heat Level](#0x0605-simulate-heat-level)             |           | &#10004;  |                |                |
|  0x06  | [Get Active Measurement](#0x0606-get-active-measurement)       |           |           |                |                |
|  0xFF  | Illegal                                                        |

### Command group 0x07__ Battery

| SUBCMD | Description                                                                                            | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT | P0302 FCT |
|:------:|:-------------------------------------------------------------------------------------------------------|:---------:|:---------:|:--------------:|:--------------:|:---------:|
|  0x00  | Illegal                                                                                                |
|  0x01  | [Set State](#0x0701-set-state)                                                                         |           | &#10004;  |                |                |           |
|  0x02  | [Set BIST Periodicity](#0x0702-set-bist-periodicity)                                                   |           | &#10004;  |                |                |           |
|  0x03  | [Simulate Voltage Level](#0x0703-simulate-voltage-level)                                               |           | &#10004;  |                |                |           |
|  0x04  | [Simulate Impedance Level](#0x0704-simulate-impedance-level)                                           |           | &#10004;  |                |                |           |
|  0x05  | [Write Low Battery Threshold](#0x0705-write-low-battery-threshold)                                     | &#10004;  |           |                |                |           |
|  0x06  | [Read Low Battery Threshold](#0x0706-read-low-battery-threshold)                                       | &#10004;  |           |                |                |           |
|  0x07  | [Read Unloaded Voltage](0x0707-read-unloaded-voltage)                                                  |           |           |    &#10004;    |                |           |
|  0x08  | [Voltage Test](#0x0708-voltage-test)                                                                   |           |           |                |                | &#10004;  |
|  0x09  | [Start Impedance Test](#0x0709-start-impedance-test)                                                   |           |           |                |                | &#10004;  |
|  0x0A  | [Get Impedance Test Status and Result](#0x070a-get-impedance-test-status-and-result)                   |           |           |                |                | &#10004;  |
|  0x0B  | [Get Impedance Test Status and Result (RAW ADC)](#0x070b-get-impedance-test-status-and-result-raw-adc) |           |           |                |                | &#10004;  |
|  0x0C  | [Read Unloaded Voltage (RAW ADC)](#0x070c-read-unloaded-voltage-raw-adc)                               |           |           |    &#10004;    |                |           |
|  0x0D  | [Voltage Test (RAW ADC)](#0x070d-voltage-test-raw-adc)                                                 |           |           |                |                | &#10004;  |
|  0x0E  | [Read Impedance (Short Test)](#0x070e-read-impedance-short-test)                                       |           |           |    &#10004;    |                |           |
|  0xFF  | Illegal                                                                                                | 

### Command group 0x08__ Buzzer

| SUBCMD | Description                      | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:---------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                          |
|  0x01  | [Set State](#0x0801-set-state)   |           | &#10004;  |                |                |
|  0x02  | [Start test](#0x0802-start-test) |           | &#10004;  |                |                |
|  0x03  | [Get State](#0x0803-get-state)   |           | &#10004;  |                |                |
|  0xFF  | Illegal                          |

### Command group 0x09__ Humidity

| SUBCMD | Description                                                | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-----------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                    |
|  0x01  | [Set BIST State](#0x0901-set-bist-state)                   |           | &#10004;  |                |                |
|  0x02  | [Simulate Humidity Level](#0x0902-simulate-humidity-level) |           | &#10004;  |                |                |
|  0x03  | [Read Humidity](#0x0903-read-humidity)                     | &#10004;  |           |                |                |
|  0xFF  | Illegal                                                    |

### Command group 0x0A__ Temperature

| SUBCMD | Description                                                              | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-------------------------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                                  |
|  0x01  | [Set BIST State](#0x0a01-set-bist-state)                                 |           | &#10004;  |                |                |
|  0x02  | [Simulate Temperature Value](#0x0a02-simulate-temperature-value)         |           | &#10004;  |                |                |
|  0x03  | [Read Temperature](#0x0a03-read-temperature)                             | &#10004;  |           |                |                |
|  0x04  | [Write Temperature Compensation](#0x0a04-write-temperature-compensation) | &#10004;  |           |                |                |
|  0x05  | [Read Temperature Compensation](#0x0a05-read-temperature-compensation)   | &#10004;  |           |                |                |
|  0xFF  | Illegal                                                                  |

### Command group 0x0B__ LED

| SUBCMD | Description                      | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT | P0302 FCT |
|:------:|:---------------------------------|:---------:|:---------:|:--------------:|----------------|-----------|
|  0x00  | Illegal                          |
|  0x01  | [Get State](#0x0b01-get-state)   |           | &#10004;  |                |                |           |
|  0x02  | [Start Test](#0x0b02-start-test) |           | &#10004;  |                |                |           |
|  0x03  | [Set State](#0x0b03-set-state)   |           |           |    &#10004;    |                | &#10004;  |
|  0xFF  | Illegal                          |

### Command group 0x0C__ Obstacle & Coverage / Laser

| SUBCMD | Description                                                                    | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-------------------------------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                                        |
|  0x01  | [Select Sensor](#0x0c01-select-sensor)                                         |           | &#10004;  |                |                |
|  0x02  | [Simulate Object Distance](#0x0c02-simulate-object-distance)                   |           | &#10004;  |                |                |
|  0x03  | [Set Object Detection Periodicity](#0x0c03-set-object-detection-periodicity)   |           | &#10004;  |                |                |
|  0x04  | [Set Object Detection BIST State](#0x0c04-set-object-detection-bist-state)     |           | &#10004;  |                |                |
|  0x05  | [Set Coverage Detection BIST State](#0x0c05-set-coverage-detection-bist-state) |           | &#10004;  |                |                |
|  0x06  | [Read Reflection Threshold](#0x0c06-read-reflection-threshold)                 | &#10004;  |           |                |                |
|  0x07  | [Write Reflection Threshold](#0x0c07-write-reflection-threshold)               | &#10004;  |           |                |                |
|  0x08  | [Execute Clean Air Calibration](#0x0c08-execute-clean-air-calibration)         |           |           |                |                |
|  0x09  | [Get Laser RAW Distance](#0x0c09-get-laser-raw-distance)                       |           |           |                | &#10004;       |
|  0x0A  | [Get Laser Distance (mm)](#0x0c0a-get-laser-distance-mm)                       |           |           |                | &#10004;       |
|  0x0B  | [Get Neighbour Laser RAW Distance](#0x0c0b-get-neighbour-laser-raw-distance)   |           |           |                |                |
|  0x0C  | [Get Neighbour Laser Distance (mm)](#0x0c0c-get-neighbour-laser-distance-mm)   |           |           |                |                |
|  0x0D  | [Get All Lasers RAW Distances](#0x0c0d-get-all-lasers-raw-distance)            |           |           |                |                |}
|  0x0E  | [Get All Lasers Distances (mm)](#0x0c0e-get-all-laser-distance-mm)             |           |           |                |                |
|  0xFF  | Illegal                                                                        |

### Command group 0x0D__ Assistance Light

| SUBCMD | Description                                | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                    |
|  0x01  | [Set Power State](#0x0d01-set-power-state) |           | &#10004;  |    &#10004;    |                |
|  0xFF  | Illegal                                    |

### Command group 0x0E__ Airing

| SUBCMD | Description                                              | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:---------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                  |
|  0x01  | [Set Airing Periodicity](#0x0e01-set-airing-periodicity) |           | &#10004;  |                |                |
|  0x02  | [Set State](#0x0e02-set-state)                           |           | &#10004;  |                |                |
|  0x03  | [Set Humidity Value](#0x0e03-set-humidity-value)         |           | &#10004;  |                |                |
|  0x04  | [Set Temperature](#0x0e04-set-temperature)               |           | &#10004;  |                |                |
|  0xFF  | Illegal                                                  |

### Command group 0x0F__ EEPROM

| SUBCMD | Description                                          | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-----------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                              |
|  0x01  | [Set BIST Periodicity](#0x0f01-set-bist-periodicity) |           | &#10004;  |                |                |
|  0x02  | [Simulate Corruption](#0x0f02-simulate-coruption)    |           | &#10004;  |                |                |
|  0x03  | [Read All / Download](#0x0f03-download)              | &#10004;  | &#10004;  |                |                |
|  0x04  | [Clear EEPROM](#0x0f04-clear-eeprom)                 |           | &#10004;  |                |                |
|  0x05  | [Write All / Upload](#0x0f05-upload)                 | &#10004;  |           |                |                |
|  0x06  | [Write (8Byte)](#0x0f06-write)                       |           |           |    &#10004;    |                |
|  0x07  | [Read  (8Byte)](#0x0f07-read)                        |           |           |    &#10004;    |                |
|  0xFF  | Illegal                                              |

### Command group 0x10__ Ambient Light

| SUBCMD | Description                                | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                    |
|  0x01  | [Set State](#0x1001-set-state)             |           | &#10004;  |                |                |
|  0x02  | [Get Measurement](#0x1002-get-measurement) |           |           |    &#10004;    |                |
|  0xFF  | Illegal                                    |

### Command group 0x11__ Switch

| SUBCMD | Description                                                          | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:---------------------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                              |
|  0x01  | [Set Read Periodicity](#0x1101-set-read-periodicity)                 |           | &#10004;  |                |                |
|  0x02  | [Get State](#0x1102-get-state)                                       |           |           |    &#10004;    |                |
|  0x03  | [Get Test Periodicity Results](#0x1103-get-test-periodicity-results) |           |           |                |                |
|  0xFF  | Illegal                                                              |

### Command group 0x12__ Configuration

| SUBCMD | Description                                                      | P0200 CAL | P0200 FTM | P0200 TM | P0200 MCU2 FCT |
|:------:|:-----------------------------------------------------------------|:---------:|:---------:|:--------:|----------------|
|  0x00  | Illegal                                                          |
|  0x01  | [Update System Config Flags](#0x1201-update-system-config-flags) |           | &#10004;  |          |                |
|  0x02  | [Update Device Config Flags](#0x1202-update-device-config-flags) |           | &#10004;  |          |                |
|  0xFF  | Illegal                                                          |

### Command group 0x13__ Radio- 868MHz

| SUBCMD | Description                                                                    | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-------------------------------------------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                                                        |
|  0x01  | [Start RF Test](#0x1301-start-rf-test)                                         |           |           |                | &#10004;       |
|  0x02  | [Set Mode](#0x1302-set-mode)                                                   |           |           |                | &#10004;       |
|  0x03  | [Get Mode](#0x1303-get-mode)                                                   |           |           |                | &#10004;       |
|  0x04  | [Set Channel](#0x1304-set-channel)                                             |           |           |                | &#10004;       |
|  0x05  | [Get Channel](#0x1305-get-channel)                                             |           |           |                | &#10004;       |
|  0x06  | [Set CTUNE value](#0x1306-set-ctune-value)                                     |           |           |                | &#10004;       |
|  0x07  | [Get CTUNE value](#0x1307-get-ctune-value)                                     |           |           |                | &#10004;       |
|  0x08  | [Adjust CTUNE value](#0x1308-adjust-ctune-value)                               |           |           |                | &#10004;       |
|  0x09  | [Start Sensitivity Test](#0x1309-start-sensitivity-test)                       |           |           |                | &#10004;       |
|  0x0A  | [End Sensitivity Test (Return the test results)](#0x130a-end-sensitivity-test) |           |           |                | &#10004;       |
|  0x0B  | [Execute Transmit Sweep Test](#0x130b-execute-transmit-sweep-test)             |           |           |                | &#10004;       |
|  0x0C  | [Execute Receive Sweep Test](#0x130c-execute-receive-sweep-test)               |           |           |                | &#10004;       |
|  0x0D  | [Set Tx Power](#0x130d-set-tx-power)                                           |           |           |                | &#10004;       |
|  0x0E  | [Get Tx Power](#0x130e-get-tx-power)                                           |           |           |                | &#10004;       |
|  0x0F  | [Get RF Test Status and Result](#0x130f-get-rf-test-status-and-result)         |           |           |                | &#10004;       |
|  0xFF  | Illegal                                                                        |

### Command group 0x14__ IrDA

| SUBCMD | Description                          | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                              |
|  0x01  | [Perform Test](#0x1401-perform-test) |           | &#10004;  |                |                |
|  0x02  | [Write Data](#0x1402-write-data)     |           |           |                | &#10004;       |
|  0x03  | [Read Data](#0x1403-read-data)       |           |           |                | &#10004;       |
|  0xFF  | Illegal                              |

### Command group 0x15__ FLASH

| SUBCMD | Description                        | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:-----------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                            |
|  0x01  | [Write Flash](#0x1501-write-flash) | &#10004;  |           |                |                |
|  0x02  | [Read Flash](#0x1502-read-flash)   | &#10004;  |           |                |                |
|  0xFF  | Illegal                            |

### Command group 0x16__ NFC

| SUBCMD | Description | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:------------|:---------:|:---------:|:--------------:|:--------------:|
|  0x00  | Illegal     |
|  0xFF  | Illegal     |

### Command group 0x17__ IO

| SUBCMD | Description                      | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT  |    P0302 FCT    |
|:------:|:---------------------------------|:---------:|:---------:|:--------------:|:---------------:|:---------------:|
|  0x00  | Illegal                          |
|  0x01  | [Read Pin](#0x1701-read-pin)     |           |           |                | &#10004; (1B_R) |                 |
|  0x02  | [Set Pin](#0x1702-set-pin)       |           |           |                | &#10004; (1B_T) | &#10004; (1B_T) |
|  0x03  | [Reset Pin](#0x1703-reset-set)   |           |           |                | &#10004; (1B_T) | &#10004; (1B_T) |
|  0x04  | [Read Pins](#0x1704-read-pins)   |           |           |                | &#10004; (6B_R) |                 |
|  0x05  | [Set Pins](#0x1705-set-pins)     |           |           |                | &#10004; (6B_T) |                 |
|  0x06  | [Reset Pins](#0x1706-reset-pins) |           |           |                | &#10004; (6B_T) |                 |
|  0xFF  | Illegal                          |

### Command group 0x18__ X-tal

| SUBCMD | Description                                                                          | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT | P0302 FCT |
|:------:|:-------------------------------------------------------------------------------------|:---------:|:---------:|:--------------:|:--------------:|:---------:|
|  0x00  | Illegal                                                                              |
|  0x01  | [Set 32.768kHz Calibration Mode](#0x1801-set-32768khz-calibration-mode)              |           |           |    &#10004;    |    &#10004;    |           |
|  0x02  | [Get 32.768kHz Calibration Mode](#0x1802-set-32768khz-calibration-mode)              |           |           |    &#10004;    |    &#10004;    |           |
|  0x03  | [Set 32.768kHz Capacitor Bank Settings](#0x1803-set-32768-capacitor-bank-settings)   |           |           |    &#10004;    |    &#10004;    |           |
|  0x04  | [Get 32.768kHz Capacitor Bank Settings](#0x1804-get-32768-capacitor-bank-settings)   |           |           |    &#10004;    |    &#10004;    |           |
|  0x05  | [Set 39MHz Calibration Mode](#0x1805-set-39mhz-calibration-mode)                     |           |           |                |    &#10004;    |           |
|  0x06  | [Get 39MHz Calibration Mode](#0x1806-get-39mhz-calibration-mode)                     |           |           |                |    &#10004;    |           |
|  0x07  | [Set 39MHz Capacitor Bank Settings](#0x1807-set-39mhz-capacitor-bank-settings)       |           |           |                |    &#10004;    |           |
|  0x08  | [Get 39MHz Capacitor Bank Settings](#0x1808-get-39mhz-capacitor-bank-settings)       |           |           |                |    &#10004;    |           |
|  0x09  | [Enable Route X-tal Frequency to GPIO](#0x1809-enable-route-x-tal-frequency-to-gpio) |           |           |                |                | &#10004;  |
|  0x0A  | [Disable Route X-tal Frequency to GPIO](#0x180a-disable-x-tal-frequency-to-gpio)     |           |           |                |                | &#10004;  |
|  0x0B  | [Save 32.768kHz CTUNE value](#0x180b-save-32768khz-ctune-value)                      |           |           |    &#10004;    |    &#10004;    |           |
|  0x0C  | [Save 39MHz CTUNE value](#0x180c-save-39mhz-value)                                   |           |           |                |    &#10004;    |           |
|  0xFF  | Illegal                                                                              |

### Command group 0x19__ WiFi

| SUBCMD | Description                                      | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT  | P0302 FCT |
|:------:|:-------------------------------------------------|:---------:|:---------:|:--------------:|:---------------:|:---------:|
|  0x00  | Illegal                                          |
|  0x01  | [Connect](#0x1901-connect)                       |           |           |                |                 | &#10004;  |
|  0x02  | [Disconnect](#0x1902-disconnect)                 |           |           |                |                 | &#10004;  |
|  0x03  | [Get Status](#0x1903-get-status)                 |           |           |                |                 | &#10004;  |
|  0x04  | [Ping Test](#0x1904-ping-test)                   |           |           |                |                 | &#10004;  |
|  0x05  | [Put In Reset](#0x1905-put-in-reset)             |           |           |                |                 | &#10004;  |
|  0x06  | [Release From Reset](#0x1906-release-from-reset) |           |           |                |                 | &#10004;  |
|  0xFF  | Illegal                                          |

### Command group 0x20__ Cellular

| SUBCMD | Description                                                                | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT | P0302 FCT |
|:------:|:---------------------------------------------------------------------------|:---------:|:---------:|:--------------:|:--------------:|:---------:|
|  0x00  | Illegal                                                                    |
|  0x01  | [Power ON](#0x2001-power-on)                                               |           |           |                |                | &#10004;  |
|  0x02  | [Power OFF](#0x2002-power-off)                                             |           |           |                |                | &#10004;  |
|  0x03  | [Get Hardware Status](#0x2003-get-hardware-status)                         |           |           |                |                | &#10004;  |
|  0x04  | [Get Numerical Identifier](#0x2004-get-numerical-identifier)               |           |           |                |                | &#10004;  |
|  0x05  | [Get Manufacturer Identification](#0x2005-get-manufacturer-identification) |           |           |                |                | &#10004;  |
|  0x06  | [Get Model Identification](#0x2006-get-model-identification)               |           |           |                |                | &#10004;  |
|  0x07  | [Get Revision Identification](#0x2007-get-revision-identification)         |           |           |                |                | &#10004;  |
|  0x08  | [Get Serial Number](#0x2008-get-serial-number)                             |           |           |                |                | &#10004;  |
|  0x09  | [Get IMEI](#0x2009-gete-imei)                                              |           |           |                |                | &#10004;  |
|  0x0A  | [Get SIM Card IMSI](#0x200a-get-sim-card-imsi)                             |           |           |                |                | &#10004;  |
|  0xFF  | Illegal                                                                    |

### Command group 0x21__ Zigbee

| SUBCMD | Description                                                | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT | P0302 FCT |
|:------:|:-----------------------------------------------------------|:---------:|:---------:|:--------------:|:--------------:|:---------:|
|  0x00  | Illegal                                                    |
|  0x01  | [Test Zigbee Port Header](#0x2101-test-zigbee-port-header) |           |           |                |                | &#10004;  |
|  0xFF  | Illegal                                                    |

### Command group 0x22__ SPI

| SUBCMD | Description                           | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT | P0302 FCT |
|:------:|:--------------------------------------|:---------:|:---------:|:--------------:|:--------------:|:---------:|
|  0x00  | Illegal                               |
|  0xFF  | Illegal                               |

### Command group 0x23__ Certificate

| SUBCMD | Description                                                    | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT | P0302 FCT |
|:------:|:---------------------------------------------------------------|:---------:|:---------:|:--------------:|:--------------:|:---------:|
|  0x00  | Illegal                                                        |
|  0x01  | [Upload root certificate](#0x2301-upload-root-certificate)     |           |           |                |                | &#10004;  |
|  0x02  | [Upload device certificate](#0x2302-upload-device-certificate) |           |           |                |                | &#10004;  |
|  0x03  | [Upload private key](#0x2303-upload-private-key)               |           |           |                |                | &#10004;  |
|  0xFF  | Illegal                                                        |


### Command group 0xE0__ Sundry

| SUBCMD | Description                                  | P0200 CAL | P0200 FTM | P0200 MCU1 FCT | P0200 MCU2 FCT |
|:------:|:---------------------------------------------|:---------:|:---------:|:--------------:|----------------|
|  0x00  | Illegal                                      |
|  0x01  | [Read all Sensors](#0xe001-read-all-sensors) | &#10004;  |           |                |                |
|  0xFF  | Illegal                                      |


# Command Set Description

The intensions of this section is to have all the commands structures in one position. These details are copied to the
relevant document (eg Manufacturing PSxxxxx).

## System Commands

### 0x0001 Protocol Enter

The *Protocol Enter* command is used where access to the device is restricted and the access needs to be enabled before 
any other commands are accepted. When access is active any valid command send to the device will reset the timout timer.

The protocol handles the access restriction and no extra checks in the command handlers is needed to check if access is
enabled. However, the implementation if the *Protocol Enter* command handler needs to set the field *isProtocolActive* in
the *nextGenCommsDriverInterface_t* instance to *true*.

In order to enable this functionality the *nextGenCommsTimoutResetHandler_t* in the *nextGenCommsDriverInterface_t*
driver definition needs to be set, this is needed to automatically reset the *Timeout Timer* on reception of a valid command.
The *nextGenCommsTimoutResetHandler_t* handler needs to implement the reset of the timout timer and is responsible to set the
field *isProtocolActive* in the *nextGenCommsDriverInterface_t* instance to *false* when a timeout occurs.

* **Tx Data Type:** NA
* **Rx Data Type:** NA

### 0x0002 Protocol Exit

The *Protocol Exit* command handler disables the access to the device by setting the field *isProtocolActive* in the 
*nextGenCommsDriverInterface_t* instance to *false*.

* **Tx Data Type:** NA
* **Rx Data Type:** NA

### 0x0003 Bootloader Enter

Implementation of this command_handler is device specific.

* **Tx Data Type:** NA
* **Rx Data Type:** NA

### 0x0004 Protocol Enter with Password

In some cases it is a requirement to add an extra level of security so no unauthorized person can access the device.

This implementation is the same as [0x0001 Protocol Enter](#0x0001-protocol-enter) with the addition of an 8-byte password.

* **Tx Data Type:** uint64
* **Rx Data Type:** NA

### 0x0020 Read Serial Number

Read the device serial number

* **Tx Data Type:** NA
* **Rx Data Type:** uint32

### 0x0021 Read Firmware Number SAxxxx

Read the SA number

* **Tx Data Type:** NA
* **Rx Data Type:** uint16

### 0x0022 Read Firmware Version

Read the Firmware Version number

* **Tx Data Type**: NA
* **Rx Data Type**: uint32

> B0: MCU Number
> 
> B1: Version Major
> 
> B2: Version Minor
> 
> B3: Version Revision

### 0x0023 Read HW Revision

Read the Hardware Revision number.

* **Tx Data Type:** NA
* **Rx Data Type:** ***NOT DEFINED***

### 0x0024 Write Serial Number

Writes the serial number to the device. This is only allowed once as serial numbers should never be changes once assigned.

* **Tx Data Type:** NA
* **Rx Data Type:** uint32

### 0x0025 Read MAC address

Reads the device EUI48 MAC address.

* **Tx Data Type:** NA
* **Rx Data Type:** uint8[6]

### 0x0026 Write MAC address

Write the device EUI48 MAC address.

* **Tx Data Type:** uint8[6]
* **Rx Data Type:** NA

### 0x0027 Reset Timeout

Resets the timeout timer to keep the protocol into the active state. This should call the *nextGenCommsTimoutResetHandler_t*
in the *nextGenCommsDriverInterface_t* driver

## Timers

### 0x0101 Set System Time

* **Tx Data Type:** ***NOT DEFINED***
* **Rx Data Type:** NA

### 0x0102 Reset System Time

* **Tx Data Type:** NA
* **Rx Data Type:** NA

### 0x0103 Increase Rate

* **Tx Data Type:** ***NOT DEFINED***
* **Rx Data Type:** NA

## Demount

### 0x0201 Change Status

* **Tx Data Type:** ***NOT DEFINED***
* **Rx Data Type:** NA

### 0x0202 Simulate State

* **Tx Data Type:** ***NOT DEFINED***
* **Rx Data Type:** NA

## Soiling

### 0x0301 Set State

Change the state for soil detection

* **Tx Data Type:** uint8

> 1 = Start Soil Detect.
>
> 2 = Stop Soil Detect
>
> 3 = Set Hardware fault
>
> 4 = Clear Hardware fault

* **Rx Data Type:** NA


### 0x0302 Set Periodicity

* **Tx Data Type:** uint16

> B0-B1 : periodicity in counts

* **Rx Data Type:** byte[4]

> B0-B1 : RAW ADC Counts
> 
> B2 : Soil Level
> 
> B3 : Soil Detection Status

### 0x0303 Simulate Soil Level

### 0x0304 Get Background Measurement

### 0x0305 Get Active Measurement

### 0x0306 End Soiling Test
