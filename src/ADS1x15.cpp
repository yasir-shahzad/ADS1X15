/**************************************************************************/
/*!
    @file     ADS1X15.cpp
    @author   Yasir Shahzad

    @mainpage ADS1X15 ADC Breakout Driver

    @section intro_sec Introduction

    This is a library for the ADS1X15 ADC breakout boards.

    @section author Author

    Written by Yasir Shahzad for Public.

    @section  HISTORY
d
    v1.0  - First release
    v1.1  - Added ADS1115 support - W. Earl
    v2.0  - Refactor - C. Nelson

    @section license License

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

#include "ADS1x15.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1015 class w/appropriate properties
*/
/**************************************************************************/
ADS1015::ADS1015() {
  m_bitShift = 4;
  m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
  m_dataRate = RATE_ADS1015_1600SPS;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1115 class w/appropriate properties
*/
/**************************************************************************/
ADS1115::ADS1115() {
  m_bitShift = 0;
  m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
  m_dataRate = RATE_ADS1115_128SPS;
}

/**************************************************************************/
/*!
    @brief  Sets up the HW (reads coefficients values, etc.)

    @param i2c_addr I2C address of device
    @param wire I2C bus

    @return true if successful, otherwise false
*/
/**************************************************************************/
bool ADS1X15::begin(uint8_t i2c_bus, uint8_t i2c_addr) {
    char device[32];
    if (fd > 0) {
        close(fd);
    }

    address = i2c_addr;
    snprintf(device, sizeof(device), "/dev/i2c-%u", i2c_bus);
    if ((fd = open(device, O_RDWR)) < 0) {
      printf("File descriptor opening error %s\n", strerror(errno));
      return -1;
    }
    else {
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            printf("File descriptor does not exist\n");
            return -1;
        }
        printf("File descriptor opened successfully\n");
    }
    return 1;
}

/**************************************************************************/
/*!
    @brief  Sets the gain and input voltage range

    @param gain gain setting to use
*/
/**************************************************************************/
void ADS1X15::setGain(adsGain_t gain) { m_gain = gain; }

/**************************************************************************/
/*!
    @brief  Gets a gain and input voltage range

    @return the gain setting
*/
/**************************************************************************/
adsGain_t ADS1X15::getGain() { return m_gain; }

/**************************************************************************/
/*!
    @brief  Sets the data rate

    @param rate the data rate to use
*/
/**************************************************************************/
void ADS1X15::setDataRate(uint16_t rate) { m_dataRate = rate; }

/**************************************************************************/
/*!
    @brief  Gets the current data rate

    @return the data rate
*/
/**************************************************************************/
uint16_t ADS1X15::getDataRate() { return m_dataRate; }

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel

    @param channel ADC channel to read

    @return the ADC reading
*/
/**************************************************************************/
int16_t ADS1X15::readADC_SingleEnded(uint8_t channel) {
  if (channel > 3) {
    return 0;
  }

  startADCReading(MUX_BY_CHANNEL[channel], /*continuous=*/false);

  // Wait for the conversion to complete
  while (!conversionComplete())
    ;
 
  // Read the conversion results
  return getLastConversionResults();
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
int16_t ADS1X15::readADC_Differential_0_1() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);

  // Wait for the conversion to complete
  while (!conversionComplete())
    ;

  // Read the conversion results
  return getLastConversionResults();
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.
    @return the ADC reading
*/
/**************************************************************************/
int16_t ADS1X15::readADC_Differential_0_3() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_3, /*continuous=*/false);

  // Wait for the conversion to complete
  while (!conversionComplete())
    ;

  // Read the conversion results
  return getLastConversionResults();
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN1) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.
    @return the ADC reading
*/
/**************************************************************************/
int16_t ADS1X15::readADC_Differential_1_3() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_1_3, /*continuous=*/false);

  // Wait for the conversion to complete
  while (!conversionComplete())
    ;

  // Read the conversion results
  return getLastConversionResults();
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
int16_t ADS1X15::readADC_Differential_2_3() {
  startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/false);

  // Wait for the conversion to complete
  while (!conversionComplete())
    ;

  // Read the conversion results
  return getLastConversionResults();
}

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.

    @param channel ADC channel to use
    @param threshold comparator threshold
*/
/**************************************************************************/
void ADS1X15::startComparator_SingleEnded(uint8_t channel,
                                                   int16_t threshold) {
  // Start with default values
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1X15_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1X15_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1X15_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1X15_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set data rate
  config |= m_dataRate;

  config |= MUX_BY_CHANNEL[channel];

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  writeRegister(ADS1X15_REG_POINTER_HITHRESH, threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(ADS1X15_REG_POINTER_CONFIG, config);
}

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.

    @return the last ADC reading
*/
/**************************************************************************/
int16_t ADS1X15::getLastConversionResults() {
  // Read the conversion results
  uint16_t res = readRegister(ADS1X15_REG_POINTER_CONVERT) >> m_bitShift;

  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}

/**************************************************************************/
/*!
    @brief  Returns true if conversion is complete, false otherwise.

    @param counts the ADC reading in raw counts

    @return the ADC reading in volts
*/
/**************************************************************************/
float ADS1X15::computeVolts(int16_t counts) {
  // see data sheet Table 3
  float fsRange;
  switch (m_gain) {
  case GAIN_TWOTHIRDS:
    fsRange = 6.144f;
    break;
  case GAIN_ONE:
    fsRange = 4.096f;
    break;
  case GAIN_TWO:
    fsRange = 2.048f;
    break;
  case GAIN_FOUR:
    fsRange = 1.024f;
    break;
  case GAIN_EIGHT:
    fsRange = 0.512f;
    break;
  case GAIN_SIXTEEN:
    fsRange = 0.256f;
    break;
  default:
    fsRange = 0.0f;
  }
  return counts * (fsRange / (32768 >> m_bitShift));
}

/**************************************************************************/
/*!
    @brief  Non-blocking start conversion function

    Call getLastConversionResults() once conversionComplete() returns true.
    In continuous mode, getLastConversionResults() will always return the
    latest result.
    ALERT/RDY pin is set to RDY mode, and a 8us pulse is generated every
    time new data is ready.

    @param mux mux field value
    @param continuous continuous if set, otherwise single shot
*/
/**************************************************************************/
void ADS1X15::startADCReading(uint16_t mux, bool continuous) {
  // Start with default values
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |   // Set CQUE to any value other than
                                        // None so we can use it in RDY mode
      ADS1X15_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1X15_REG_CONFIG_CMODE_TRAD;    // Traditional comparator (default val)

  if (continuous) {
    config |= ADS1X15_REG_CONFIG_MODE_CONTIN;   
  } else {
    config |= ADS1X15_REG_CONFIG_MODE_SINGLE;   //set 9th bit
  }

  // Set PGA/voltage range
  config |= m_gain;

  // Set data rate
  config |= m_dataRate;

  // Set channels
  config |= mux;

  // Set 'start single-conversion' bit
  config |= ADS1X15_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(ADS1X15_REG_POINTER_CONFIG, config);

  // Set ALERT/RDY to RDY mode.
  writeRegister(ADS1X15_REG_POINTER_HITHRESH, 0x8000);
  writeRegister(ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);
}

/**************************************************************************/
/*!
    @brief  Returns true if conversion is complete, false otherwise.

    @return True if conversion is complete, false otherwise.
*/
/**************************************************************************/
bool ADS1X15::conversionComplete() {
  return (readRegister(ADS1X15_REG_POINTER_CONFIG) & 0x8000) != 0;
}

/**************************************************************************/
/*!
    @brief  Writes 16-bits to the specified destination register

    @param reg register address to write to
    @param value value to write to register
*/
/**************************************************************************/
int16_t ADS1X15::writeRegister(uint8_t reg, uint16_t value) {
    buffer[0] = reg;
    buffer[1] = ((uint8_t) (value >> 8));
    buffer[2] = ((uint8_t) (value & 0xFF));

    if (write(fd, buffer, 3) != 3)
        return -1;
    return 3;
}

/**************************************************************************/
/*!
    @brief  Read 16-bits from the specified destination register

    @param reg register address to read from

    @return 16 bit register value read
*/
/**************************************************************************/
uint16_t ADS1X15::readRegister(uint8_t reg) {

    if (write(fd, &reg, 1) != 1)
        return 0;

    if (read(fd, buffer, 2) != 2)
        return 0;

    return ((buffer[0] << 8) | buffer[1]);
}

ADS1115::~ADS1115(){
    close(fd);
}
