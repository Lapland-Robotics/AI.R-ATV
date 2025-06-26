# INA219
### Software change is required for the code to work
The Adafruit_INA219 library that is being used, is using a different value resistor in the power and current calculations. For this change, a new class should be added to the library code in order for it to function with the setup that we have.

The current setup in the universal power distribution pcb is the following: 
RShunt = 0.002
Max current = 3.5A
Bus voltage = 32V

In the **Adafruit_INA219.cpp** add the following function:

```
void Adafruit_INA219::setCalibration_32V_3_5A_2mOhms(void) {
  // 1) Calculate calibration register
  //    CurrentLSB = 0.0005 A (i.e. 0.5 mA/bit)
  //    cal = floor(0.04096 / (0.0005 * 0.002)) = 40960
  ina219_calValue = 40960; // 0xA000

  // 2) Set the raw current/power conversion factors:
  //    If each bit = 0.5mA, we do: current_mA = raw / 2
  //    => ina219_currentDivider_mA = 2
  ina219_currentDivider_mA = 2;

  //    Power LSB = 20 * currentLSB = 20 * 0.0005 = 0.01 W = 10 mW
  //    => ina219_powerMultiplier_mW = 10.0f
  ina219_powerMultiplier_mW = 10.0f;

  // 3) Write calibration register
  Adafruit_BusIO_Register calibration_reg(
      i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);

  // 4) Configure:
  //    - Bus Voltage range = 32 V
  //    - Gain = 1 (±40mV range across shunt)
  //    - 12-bit resolution / 532µs conv. for both bus & shunt
  //    - Continuous measurement
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_1_40MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  Adafruit_BusIO_Register config_reg(
      i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
  _success = config_reg.write(config, 2);
}
```

In the **Adafruit_INA219.h** Add a reference to this class function together with the other setCalibration functions:
```
void setCalibration_32V_3_5A_2mOhms(void);
```

If you compile this library, you can now call it in the arduino IDE in the required code to get the correct calculations:
```
ina219.setCalibration_32V_3_5A_2mOhms();
```