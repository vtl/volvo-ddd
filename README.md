# VOLVO Driver Distraction Display :)

Firmware for Arduino Due connected to car's CAN bus (both high-speed and low-speed)
and a small 2.4" LCD display, which fills in an empty space on aftermarket 2DIN fascia.

![pic](/doc/IMG_20161109_241725347.jpg)

See doc/ for more pics!

# Features supported:

## Reads out next sensors:
### ECM - engine (in imperial and metric units)
* Boost
* Coolant temperature
* ATF temperature
* Intake air temperature
* Engine speed (RPM)
* Engine torque
* Battery voltage
* Fuel pump duty cycle
* Fuel rail pressure
* Short-term fuel trim (STFT)
* Long-term fuel trim (LTFT)
* Turbo control valve (TCV) duty
* Throttle plate angle
* Mass Air Flow (MAF) rate
* VVT angles, intake and exhaust
* Ignition timing (BTDC)
* Engine fan duty cycle
* Misfire counter
### TCM - transmission
* Gear selector
* Current gear (calculated via solenoids state in valve body)
* Gear ratio
* SLT, SLS, SLU linear solenoids current
* S1-S5 solenoids state
### DEM - Haldex/AWD
* All 4 wheels speed
* Haldex unit oil pressure and temperature
* Haldex unit solenoid and pump currents
### CCM - climate
* A/C high port pressure
* A/C compressor duty
* Evaporator temperature
* Cabin temperature
* Blower motor duty
## Controls
### Steering wheel buttons (SWC)
* sends Kenwood remote codes
### Touch-screen
* Raise/lower GPS navigation 
* Power on/off original rear seat entertainment displays and select wireless audio channel (via separate controller, see https://github.com/vtl/volvo-rse)
* Reset service remainder indicator (SRI)
* Transmission adaptation
* Reset crash mode

# Hardware used:

* ESP32 
* 2 or 4 MCP2515-based CAN-adapters
* 4D Systems gen4-uLCD-28PTU (+ programming cable) or any other 4D Systems display
* ULN2003A (Darlington array, level converter)
* 2x CYT1076 (3.3v <> 5.v level shifter)

Schematics: https://easyeda.com/v.mayatskih/volvo-ddd

![pic](/doc/esp32.png)
