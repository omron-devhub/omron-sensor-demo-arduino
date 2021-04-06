
/*
 * MIT License
 * Copyright (c) 2019, 2018 - present OMRON Corporation
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/* includes */
#include <Wire.h>
#include <Arduino.h>

/* defines for D6T-01 */
#define D6T_ADDR 0x0A  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02
#define N_ROW 1
#define N_PIXEL 1
#define N_READ ((N_PIXEL + 1) * 2 + 1)
uint8_t rbuf[N_READ];

/* defines for D6F-10 & D6F-PH0505 */
#define D6F_ADDR 0x6C  // D6F-PH I2C client address at 7bit expression

/* defines for B5W-LB */
#if defined(ARDUINO_FEATHER_ESP32)
#define PIN_B5WLB_IN      13
#define PIN_B5WLB_OUT     33
#else
#define PIN_B5WLB_IN      19
#define PIN_B5WLB_OUT     17
#endif

/* defines for 2SMPP */
#define PIN_2SMPP_INPUT   2

/* defines for 2SMPB-02E */
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BARO_2SMPB02E_ADDRESS                (0x56)
/*=========================================================================*/
/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define BARO_2SMPB02E_REGI2C_PRES_TXD2               0xF7
    #define BARO_2SMPB02E_REGI2C_IO_SETUP                0xF5
    #define BARO_2SMPB02E_REGI2C_CTRL_MEAS               0xF4
    #define BARO_2SMPB02E_REGI2C_IIR                     0xF1
    #define BARO_2SMPB02E_REGI2C_CHIP_ID                 0xD1
    #define BARO_2SMPB02E_REGI2C_COEFS                   0xA0
    /* Register values */
    #define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0001MS       ((uint8_t)0x00)
    #define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0125MS       ((uint8_t)0x20)
    #define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0250MS       ((uint8_t)0x40)
    #define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0500MS       ((uint8_t)0x60)
    #define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_1000MS       ((uint8_t)0x80)
    #define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_2000MS       ((uint8_t)0xA0)
    #define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_4000MS       ((uint8_t)0xC0)
    #define BARO_2SMPB02E_VAL_IOSETUP_STANDBY_8000MS       ((uint8_t)0xE0)

    #define BARO_2SMPB02E_VAL_TEMPAVERAGE_01     ((uint8_t)0x20)
    #define BARO_2SMPB02E_VAL_TEMPAVERAGE_02     ((uint8_t)0x40)
    #define BARO_2SMPB02E_VAL_TEMPAVERAGE_04     ((uint8_t)0x60)

    #define BARO_2SMPB02E_VAL_PRESAVERAGE_01    ((uint8_t)0x04)
    #define BARO_2SMPB02E_VAL_PRESAVERAGE_02    ((uint8_t)0x08)
    #define BARO_2SMPB02E_VAL_PRESAVERAGE_04    ((uint8_t)0x0C)
    #define BARO_2SMPB02E_VAL_PRESAVERAGE_08    ((uint8_t)0x10)
    #define BARO_2SMPB02E_VAL_PRESAVERAGE_16    ((uint8_t)0x14)
    #define BARO_2SMPB02E_VAL_PRESAVERAGE_32    ((uint8_t)0x18)

    #define BARO_2SMPB02E_VAL_POWERMODE_SLEEP  ((uint8_t)0x00)
    #define BARO_2SMPB02E_VAL_POWERMODE_FORCED ((uint8_t)0x01)
    #define BARO_2SMPB02E_VAL_POWERMODE_NORMAL ((uint8_t)0x03)

    #define BARO_2SMPB02E_VAL_IIR_OFF     ((uint8_t)0x00)
    #define BARO_2SMPB02E_VAL_IIR_02TIMES ((uint8_t)0x01)
    #define BARO_2SMPB02E_VAL_IIR_04TIMES ((uint8_t)0x02)
    #define BARO_2SMPB02E_VAL_IIR_08TIMES ((uint8_t)0x03)
    #define BARO_2SMPB02E_VAL_IIR_16TIMES ((uint8_t)0x04)
    #define BARO_2SMPB02E_VAL_IIR_32TIMES ((uint8_t)0x05)

    /* Coeff */
    #define BARO_2SMPB02E_COEFF_S_A1   ((double)( 4.3E-04))
    #define BARO_2SMPB02E_COEFF_A_A1   ((double)(-6.3E-03))
    #define BARO_2SMPB02E_COEFF_S_A2   ((double)( 1.2E-10))
    #define BARO_2SMPB02E_COEFF_A_A2   ((double)(-1.9E-11))
    #define BARO_2SMPB02E_COEFF_S_BT1  ((double)( 9.1E-02))
    #define BARO_2SMPB02E_COEFF_A_BT1  ((double)( 1.0E-01))
    #define BARO_2SMPB02E_COEFF_S_BT2  ((double)( 1.2E-06))
    #define BARO_2SMPB02E_COEFF_A_BT2  ((double)( 1.2E-08))
    #define BARO_2SMPB02E_COEFF_S_BP1  ((double)( 1.9E-02))
    #define BARO_2SMPB02E_COEFF_A_BP1  ((double)( 3.3E-02))
    #define BARO_2SMPB02E_COEFF_S_B11  ((double)( 1.4E-07))
    #define BARO_2SMPB02E_COEFF_A_B11  ((double)( 2.1E-07))
    #define BARO_2SMPB02E_COEFF_S_BP2  ((double)( 3.5E-10))
    #define BARO_2SMPB02E_COEFF_A_BP2  ((double)(-6.3E-10))
    #define BARO_2SMPB02E_COEFF_S_B12  ((double)( 7.6E-13))
    #define BARO_2SMPB02E_COEFF_A_B12  ((double)( 2.9E-13))
    #define BARO_2SMPB02E_COEFF_S_B21  ((double)( 1.2E-14))
    #define BARO_2SMPB02E_COEFF_A_B21  ((double)( 2.1E-15))
    #define BARO_2SMPB02E_COEFF_S_BP3  ((double)( 7.9E-17))
    #define BARO_2SMPB02E_COEFF_A_BP3  ((double)( 1.3E-16))

    #define BARO_2SMPB02E_VAL_MEASMODE_HIGHSPEED \
        (BARO_2SMPB02E_VAL_PRESAVERAGE_02 | BARO_2SMPB02E_VAL_TEMPAVERAGE_01)
    #define BARO_2SMPB02E_VAL_MEASMODE_LOWPOWER \
        (BARO_2SMPB02E_VAL_PRESAVERAGE_04 | BARO_2SMPB02E_VAL_TEMPAVERAGE_01)
    #define BARO_2SMPB02E_VAL_MEASMODE_STANDARD \
        (BARO_2SMPB02E_VAL_PRESAVERAGE_08 | BARO_2SMPB02E_VAL_TEMPAVERAGE_01)
    #define BARO_2SMPB02E_VAL_MEASMODE_HIGHACCURACY \
        (BARO_2SMPB02E_VAL_PRESAVERAGE_16 | BARO_2SMPB02E_VAL_TEMPAVERAGE_02)
    #define BARO_2SMPB02E_VAL_MEASMODE_ULTRAHIGH \
        (BARO_2SMPB02E_VAL_PRESAVERAGE_32 | BARO_2SMPB02E_VAL_TEMPAVERAGE_04)
/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct baro_2smpb02e_setting {
        /* Compensation Factor */
        double _A0, _A1, _A2;
        double _B00, _BT1, _BP1;
        double _B11, _BT2, _BP2;
        double _B12, _B21, _BP3;
    } baro_2smpb02e_setting_t;

/*=========================================================================
    Values
    -----------------------------------------------------------------------*/
/* defines */
#define BARO_2SMPB02E_CHIP_ID     0x5C
/* values */
baro_2smpb02e_setting_t baro_2smpb02e_setting;
/* macros */
#define conv8s_s24_be(a, b, c) \
        (int32_t)((((uint32_t)a << 16) & 0x00FF0000) | \
                  (((uint32_t)b << 8) & 0x0000FF00) | \
                   ((uint32_t)c & 0x000000FF))



//Define for Common 
static uint16_t state = 0;
static int counts_idx = 0;

/** D6T function ********/
uint8_t calc_crc(uint8_t data) {
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

/** <!-- D6T_checkPEC {{{ 1--> D6T PEC(Packet Error Check) calculation.
 * calculate the data sequence,
 * from an I2C Read client address (8bit) to thermal data end.
 */
bool D6T_checkPEC(uint8_t buf[], int n) {
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1);  // I2C Read address (8bit)
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret) {
        Serial.print("PEC check failed:");
        Serial.print(crc, HEX);
        Serial.print("(cal) vs ");
        Serial.print(buf[n], HEX);
        Serial.println("(get)");
    }
    return ret;
}


/** D6T function <!-- conv8us_s16_le {{{1 --> convert a 16bit data from the byte stream.*/
int16_t conv8us_s16_le(uint8_t* buf, int n) {
    int ret;
    ret = buf[n];
    ret += buf[n + 1] << 8;
    return (int16_t)ret;   // and convert negative.
}

/**D6F function ************************/
uint8_t conv16_u8_h(int16_t a) {
    return (uint8_t)(a >> 8);
}

uint8_t conv16_u8_l(int16_t a) {
    return (uint8_t)(a & 0xFF);
}

uint16_t conv8us_u16_be(uint8_t* buf) {
    return (uint16_t)(((uint32_t)buf[0] << 8) | (uint32_t)buf[1]);
}

/** D6F <!-- i2c_write_reg16 {{{1 --> I2C write bytes with a 16bit register.*/
bool i2c_write_reg16(uint8_t slave_addr, uint16_t register_addr,
                     uint8_t *write_buff, uint8_t len) {
    Wire.beginTransmission(slave_addr);

    Wire.write(conv16_u8_h(register_addr));
    Wire.write(conv16_u8_l(register_addr));

    if (len != 0) {
        for (uint8_t i = 0; i < len; i++) {
            Wire.write(write_buff[i]);
        }
    }
    Wire.endTransmission();
    return false;
}

/** D6F <!-- i2c_read_reg8 {{{1 --> I2C read bytes with a 8bit register. */
bool i2c_read_reg8(uint8_t slave_addr, uint8_t register_addr,
                   uint8_t *read_buff, uint8_t len) {
    Wire.beginTransmission(slave_addr);
    Wire.write(register_addr);
    Wire.endTransmission();

    Wire.requestFrom(slave_addr, len);

    if (Wire.available() != len) {
        return true;
    }
    for (uint16_t i = 0; i < len; i++) {
        read_buff[i] = Wire.read();
    }
    return false;
}

/** 2SMPB  <!-- i2c_write_reg8 {{{1 --> I2C write function for bytes transfer.
 */
bool i2c_write_reg8(uint8_t slave_addr, uint8_t register_addr,
                    uint8_t *write_buff, uint8_t len) {
    Wire.beginTransmission(slave_addr);

    Wire.write(register_addr);
    if (len != 0) {
        for (uint8_t i = 0; i < len; i++) {
            Wire.write(write_buff[i]);
        }
    }
    Wire.endTransmission();
    return false;
}

/** 2SMPB <!-- i2c_read_reg8 {{{1 --> I2C read function for bytes transfer.
 */
bool i2c_read_reg8_2SMPB(uint8_t slave_addr, uint8_t register_addr,
                   uint8_t *read_buff, uint8_t len) {
    i2c_write_reg8(slave_addr, register_addr, NULL, 0);

    Wire.requestFrom(slave_addr, len);

    if (Wire.available() != len) {
        return true;
    }
    for (uint16_t i = 0; i < len; i++) {
        read_buff[i] = Wire.read();
    }
    return false;
}

/** <!-- baro_2smpb02e_setup {{{1 --> setup for 2SMPB-02E
 * 1. check CHIP_ID to confirm I2C connections.
 * 2. read coefficient values for compensations.
 * 3. sensor setup and start to measurements.
 */
bool baro_2smpb02e_setup(void) {
    bool result;
    uint8_t rbuf[32] = {0};
    uint8_t ex;

    // 1.
    result = i2c_read_reg8(BARO_2SMPB02E_ADDRESS,
                           BARO_2SMPB02E_REGI2C_CHIP_ID, rbuf, 1);

    // 2.
    result = i2c_read_reg8(BARO_2SMPB02E_ADDRESS,
            BARO_2SMPB02E_REGI2C_COEFS, rbuf, 25);


    // pressure parameters
    ex = (rbuf[24] & 0xf0) >> 4;
    baro_2smpb02e_setting._B00 = baro_2smpb02e_conv20q4_dbl(rbuf, ex, 0);
    baro_2smpb02e_setting._BT1 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BT1, BARO_2SMPB02E_COEFF_S_BT1, rbuf, 2);
    baro_2smpb02e_setting._BT2 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BT2, BARO_2SMPB02E_COEFF_S_BT2, rbuf, 4);
    baro_2smpb02e_setting._BP1 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BP1, BARO_2SMPB02E_COEFF_S_BP1, rbuf, 6);
    baro_2smpb02e_setting._B11 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_B11, BARO_2SMPB02E_COEFF_S_B11, rbuf, 8);
    baro_2smpb02e_setting._BP2 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BP2, BARO_2SMPB02E_COEFF_S_BP2, rbuf, 10);
    baro_2smpb02e_setting._B12 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_B12, BARO_2SMPB02E_COEFF_S_B12, rbuf, 12);
    baro_2smpb02e_setting._B21 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_B21, BARO_2SMPB02E_COEFF_S_B21, rbuf, 14);
    baro_2smpb02e_setting._BP3 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BP3, BARO_2SMPB02E_COEFF_S_BP3, rbuf, 16);

    // temperature parameters
    ex = (rbuf[24] & 0x0f);
    baro_2smpb02e_setting._A0 = baro_2smpb02e_conv20q4_dbl(rbuf, ex, 18);
    baro_2smpb02e_setting._A1 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_A1, BARO_2SMPB02E_COEFF_S_A1, rbuf, 20);
    baro_2smpb02e_setting._A2 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_A2, BARO_2SMPB02E_COEFF_S_A2, rbuf, 22);

    // 3. setup a sensor at 125msec sampling and 32-IIR filter.
    rbuf[0] = BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0125MS;
    i2c_write_reg8(BARO_2SMPB02E_ADDRESS, BARO_2SMPB02E_REGI2C_IO_SETUP,
                   rbuf, sizeof(rbuf));

    rbuf[0] = BARO_2SMPB02E_VAL_IIR_32TIMES;
    i2c_write_reg8(BARO_2SMPB02E_ADDRESS, BARO_2SMPB02E_REGI2C_IIR,
                   rbuf, sizeof(rbuf));

    // then, start to measurements.
    result = baro_2smpb02e_trigger_measurement(
            BARO_2SMPB02E_VAL_MEASMODE_ULTRAHIGH);
    return false;
}

/** <!-- baro_2smpb02e_conv16_dbl {{{1 --> convert bytes buffer to double.
 * bytes buffer format is a signed-16bit Big-Endian.
 */
static double baro_2smpb02e_conv16_dbl(double a, double s,
                                       uint8_t* buf, int offset) {
    uint16_t val;
    int16_t ret;

    val = (uint16_t)(
            (uint16_t)(buf[offset] << 8) | (uint16_t)buf[offset + 1]);
    if ((val & 0x8000) != 0) {
        ret = (int16_t)((int32_t)val - 0x10000);
    } else {
        ret = val;
    }
    return a + (double)ret * s / 32767.0;
}

/** <!-- baro_2smpb02e_conv20q4_dbl {{{1 --> convert bytes buffer to double.
 * bytes buffer format is signed 20Q4, from -32768.0 to 32767.9375
 *
 * ### bit field of 20Q4
 * ```
 * |19,18,17,16|15,14,13,12|11,10, 9, 8| 7, 6, 5, 4| 3, 2, 1, 0|
 * | buf[offset]           | buf[offset+1]         | ex        |
 *                                                 A
 *                                                 |
 *                                                 +-- Decimal point
 * ```
 */
static double baro_2smpb02e_conv20q4_dbl(uint8_t* buf,
                                         uint8_t ex, int offset) {
    int32_t ret;
    uint32_t val;

    val = (uint32_t)((buf[offset] << 12) | (buf[offset + 1] << 4) | ex);
    if ((val & 0x80000) != 0) {
        ret = (int32_t)val - 0x100000;
    } else {
        ret = val;
    }
    return (double)ret / 16.0;
}

/** <!-- baro_2smpb02e_trigger_measurement {{{1 --> start the sensor
 */
static bool baro_2smpb02e_trigger_measurement(uint8_t mode) {
    uint8_t wbuf[1] = {
        (uint8_t)(mode | BARO_2SMPB02E_VAL_POWERMODE_NORMAL)};

    i2c_write_reg8(BARO_2SMPB02E_ADDRESS, BARO_2SMPB02E_REGI2C_CTRL_MEAS,
                   wbuf, sizeof(wbuf));
    return false;
}

/** <!-- baro_2smpb02e_read {{{1 --> read the sensor digit and convert to
 * physical values.
 */
int baro_2smpb02e_read(uint32_t* pres, int16_t* temp,
                      uint32_t* dp, uint32_t* dt) {
    bool ret;
    uint8_t rbuf[6] = {0};
    uint32_t rawtemp, rawpres;

    ret = i2c_read_reg8(
            BARO_2SMPB02E_ADDRESS, BARO_2SMPB02E_REGI2C_PRES_TXD2,
            rbuf, sizeof(rbuf));
    if (ret) {
        return 1;
    }

    *dp = rawpres = conv8s_s24_be(rbuf[0], rbuf[1], rbuf[2]);
    *dt = rawtemp = conv8s_s24_be(rbuf[3], rbuf[4], rbuf[5]);
    return baro_2smpb02e_output_compensation(rawtemp, rawpres, pres, temp);
}

/** <!-- baro_2smpb02e_output_compensation {{{1 --> compensate sensors
 * raw output digits to [Pa] and [degC].
 */
bool baro_2smpb02e_output_compensation(uint32_t raw_temp_val,
                                       uint32_t raw_press_val,
                                       uint32_t* pres, int16_t* temp
) {
    double Tr, Po;
    double Dt, Dp;

    Dt = (int32_t)raw_temp_val - 0x800000;
    Dp = (int32_t)raw_press_val - 0x800000;

    // temperature compensation
    baro_2smpb02e_setting_t* c = &baro_2smpb02e_setting;
    Tr = c->_A0 + c->_A1 * Dt + c->_A2 * (Dt * Dt);

    // barometer compensation
    Po = c->_B00 + (c->_BT1 * Tr) + (c->_BP1 * Dp) +
         (c->_B11 * Tr * Dp) + c->_BT2 * (Tr * Tr) +
         (c->_BP2 * (Dp * Dp)) + (c->_B12 * Dp * (Tr * Tr)) +
         (c->_B21 * (Dp * Dp) * Tr) + (c->_BP3 * (Dp * Dp * Dp));

    *temp = (int16_t)(Tr / 2.56);     // x100degC
    *pres = (uint32_t)(Po * 10.0);    // x10Pa
    return false;
}



/** <!-- setup {{{1 -->
 * 1. initialize a Serial port for output.
 * 2. initialize an I2C peripheral.
 */
void setup() {
    Serial.begin(115200);  // Serial baudrate = 115200bps
    Wire.begin();  // i2c master
  
    delay(32);
    // D6F setup: EEPROM Control <= 0x00h
    i2c_write_reg16(D6F_ADDR, 0x0B00, NULL, 0);

    // B5W-LB setup: turn off LED for safe operation.
    pinMode(PIN_B5WLB_IN, OUTPUT);
    pinMode(PIN_B5WLB_OUT, INPUT);
    digitalWrite(PIN_B5WLB_IN, LOW);

    // 2SMPB setup: 
    baro_2smpb02e_setup();
    delay(32);
}

/** <!-- loop - Thermal sensor {{{1 -->
 * 1. read sensor.
 * 2. output results, format is: [degC]
 */
void loop() {
    int i, j;
  /**** B5W-LB *******************************/
  int32_t i_low = 0;
  float f_low = 0;
  int32_t i_high = 0;
  float f_high = 0;
  float f_diff = 0;
  /*****************************************/
  /**** D6T *******************************/
  int16_t itemp = 0;
  /*****************************************/
  /**** D6F *******************************/
  uint8_t send0[] = {0x40, 0x18, 0x06};
  uint8_t send1[] = {0x51, 0x2C};
  uint8_t rbuf[2];
  uint16_t rd_flow = 0;
  float flow_rate = 0;
  float pressure = 0; 
  /*****************************************/
  /**** 2SMPP *******************************/
  int32_t gauge_press; //Pa
  /*****************************************/ 
  /**** 2SMPB *******************************/
  uint32_t pres, dp, dt;
  int16_t temp;
  /*****************************************/
  while (state < 500){ //  (500 x 200us = every 100 ms) (250 x 200us = every 50 ms) 
    switch(state){
      /********** B5W-LB ****************************************************/
      case 1: 
        // get Vout(OFF): output voltage when LED is turn off.
        analogReadResolution(10);
        digitalWrite(PIN_B5WLB_IN, LOW);
        break;
      case 2:
      case 3:
      case 4:
      case 5:     
        break;
      case 6: //after 1000us 
        i_low = analogRead(PIN_B5WLB_OUT);
        f_low = (float)i_low * 3.3 / 1024.0;
        break;
      case 7: //after 200us 
        // get Vout(ON): output voltage when LED is turn on.
        digitalWrite(PIN_B5WLB_IN, HIGH);
        break;      
      case 8:
        break;  
      case 9: //after 400us 
        i_high = analogRead(PIN_B5WLB_OUT);
        #if defined(ARDUINO_FEATHER_ESP32)
        f_high = (float)i_high * 3.6 / 4096.0;   // for Feather ESP32
        #else
        f_high = (float)i_high * 3.3 / 1024.0;   // for Arduino MKR
        #endif
        break;  
      case 10:
        break;      
      case 11: //after 400us   should not be exceed 40% ON time of period.
        // turn off to next period.
        digitalWrite(PIN_B5WLB_IN, LOW);
        f_diff = f_high - f_low;  // cancellation processing for external disturbing light interference.
        f_diff = f_diff < 0.0 ? 0.0: f_diff;
        break;      
      /**********************************************************************/
      /********** D6T ****************************************************/
      case 12: 
        memset(rbuf, 0, N_READ);
        Wire.beginTransmission(D6T_ADDR);  // I2C client address
        Wire.write(D6T_CMD);               // D6T register
        Wire.endTransmission();            // I2C repeated start for read
        Wire.requestFrom(D6T_ADDR, N_READ);
        i = 0;
        while (Wire.available()) {
          rbuf[i++] = Wire.read();
        }
        //if (D6T_checkPEC(rbuf, N_READ - 1)) {
        //  return;
        //}    
        itemp = conv8us_s16_le(rbuf, 2);
        break;  
     /**********************************************************************/
     /********** D6F ****************************************************/
     case 0:
      i2c_write_reg16(D6F_ADDR, 0x00D0, send0, 3);      
      break;
     case 450: //after 90ms
      i2c_write_reg16(D6F_ADDR, 0x00D0, send1, 2);      
      //if (i2c_read_reg8(D6F_ADDR, 0x07, rbuf, 2)) {  // read from [07h]
      //  return;
      //}
      i2c_read_reg8(D6F_ADDR, 0x07, rbuf, 2);
      rd_flow = conv8us_u16_be(rbuf);  
      break;  
    /**********************************************************************/
    /********** 2SMPP ****************************************************/
     case 13:
      analogReadResolution(12);
      gauge_press = analogRead(PIN_2SMPP_INPUT);
      //Op-Amp Gain = 17.90
      //Vref = 1.63V ⇒　1.63V/3.3V*2^12 = 2026bit
      //Voffset(D0) = (-2.50mV*17.90*2^12)/(3.3V*1000) = -55bit
      //Span(A) = (0.84*2^12*17.90)/(3.3V*1000) = 19 bit/kPa 
      gauge_press = (int32_t)(( (double)gauge_press - 2026 + 55 ) / 19 * 1000);
      break;  
    /**************************************************************/
    /********** 2SMPB ****************************************************/
     case 14: 
      baro_2smpb02e_read(&pres, &temp, &dp, &dt);
      break;  
    /**************************************************************/
  default:
      break;
    }
    
    delayMicroseconds(200);
    state ++;
  }
  state = 0;
  uint8_t send_buf[40];
  uint8_t b_xor = 0;
  send_buf[0] = 0x5A;
  send_buf[1] = 0xA5;
  send_buf[2] = 54;
  send_buf[3] = 0x00;
  send_buf[4] = 0x00;
  for(int ii=0; ii<54; ii++){
    send_buf[5+ii] = ii;   
  }

  send_buf[5] = (uint8_t)((uint16_t)((int16_t)(f_diff*1000.0)));
  send_buf[6] = (uint8_t)((uint16_t)((int16_t)(f_diff*1000.0)) >> 8);
  send_buf[7] = (uint8_t)rd_flow;
  send_buf[8] = (uint8_t)(rd_flow >> 8);
  send_buf[11] = (uint8_t)rd_flow;
  send_buf[12] = (uint8_t)(rd_flow >> 8);
  send_buf[15] = (uint8_t)pres;
  send_buf[16] = (uint8_t)(pres >> 8);
  send_buf[17] = (uint8_t)(pres >> 16);
  send_buf[18] = (uint8_t)(pres >> 24);
  send_buf[21] = (uint8_t)((uint32_t)gauge_press);
  send_buf[22] = (uint8_t)(((uint32_t)gauge_press) >> 8);
  send_buf[23] = (uint8_t)(((uint32_t)gauge_press) >> 16);
  send_buf[24] = (uint8_t)(((uint32_t)gauge_press) >> 24);
  send_buf[27] = (uint8_t)((uint16_t)itemp);
  send_buf[28] = (uint8_t)(((uint16_t)itemp) >> 8);
  
  for (int ii = 0; ii < 54; ii++)
  {
    b_xor ^= send_buf[5 + ii];
  }
  send_buf[59] = b_xor;

  Serial.write(send_buf, 60);      
}
