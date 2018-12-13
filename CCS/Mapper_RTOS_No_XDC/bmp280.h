#ifndef BMP280_H_
#define BMP280_H_

#include <ti/drivers/I2C.h>

#define BMP280_OK   0


/* Error codes */
#define BMP280_E_NULL_PTR       0x01
#define BMP280_E_DEV_NOT_FOUND  0x02
#define BMP280_E_INVALID_LEN    0x03
#define BMP280_E_COMM_FAIL      0x04
#define BMP280_E_INVALID_MODE   0x05

/* Chip ID's */
#define BMP280_CHIP_ID1         0x56
#define BMP280_CHIP_ID2         0x57
#define BMP280_CHIP_ID3         0x58

/* I2C Addresses */
#define BMP280_I2C_ADDR_PRIM    0x76
#define BMP280_I2C_ADDR_SEC     0x77
#define BMP280_I2C_INTF         0x01

/* Calibration register addresses */
#define BMP280_DIG_T1_LSB_ADDR  0x88
#define BMP280_DIG_T1_MSB_ADDR  0x89
#define BMP280_DIG_T2_LSB_ADDR  0x8A
#define BMP280_DIG_T2_MSB_ADDR  0x8B
#define BMP280_DIG_T3_LSB_ADDR  0x8C
#define BMP280_DIG_T3_MSB_ADDR  0x8D
#define BMP280_DIG_P1_LSB_ADDR  0x8E
#define BMP280_DIG_P1_MSB_ADDR  0x8F
#define BMP280_DIG_P2_LSB_ADDR  0x90
#define BMP280_DIG_P2_MSB_ADDR  0x91
#define BMP280_DIG_P3_LSB_ADDR  0x92
#define BMP280_DIG_P3_MSB_ADDR  0x93
#define BMP280_DIG_P4_LSB_ADDR  0x94
#define BMP280_DIG_P4_MSB_ADDR  0x95
#define BMP280_DIG_P5_LSB_ADDR  0x96
#define BMP280_DIG_P5_MSB_ADDR  0x97
#define BMP280_DIG_P6_LSB_ADDR  0x98
#define BMP280_DIG_P6_MSB_ADDR  0x99
#define BMP280_DIG_P7_LSB_ADDR  0x9A
#define BMP280_DIG_P7_MSB_ADDR  0x9B
#define BMP280_DIG_P8_LSB_ADDR  0x9C
#define BMP280_DIG_P8_MSB_ADDR  0x9D
#define BMP280_DIG_P9_LSB_ADDR  0x9E
#define BMP280_DIG_P9_MSB_ADDR  0x9F

/* General registers */
#define BMP280_CHIP_ID_ADDR     0xD0
#define BMP280_CHIP_ID_ADDR      0xD0
#define BMP280_SOFT_RESET_ADDR  0xE0
#define BMP280_STATUS_ADDR      0xF3
#define BMP280_CTRL_MEAS_ADDR   0xF4
#define BMP280_CONFIG_ADDR      0xF5
#define BMP280_PRES_MSB_ADDR    0xF7
#define BMP280_PRES_LSB_ADDR    0xF8
#define BMP280_PRES_XLSB_ADDR   0xF9
#define BMP280_TEMP_MSB_ADDR    0xFA
#define BMP280_TEMP_LSB_ADDR    0xFB
#define BMP280_TEMP_XLSB_ADDR   0xFC

/* Power Modes */
#define BMP280_SLEEP_MODE       0x00
#define BMP280_FORCED_MODE      0x01
#define BMP280_NORMAL_MODE      0x03


/* Soft Reset Command */
#define BMP280_SOFT_RESET_CMD   0xB6

/* ODR Options */
#define BMP280_ODR_0_5_MS       0x00
#define BMP280_ODR_62_5_MS      0x01
#define BMP280_ODR_125_MS       0x02
#define BMP280_ODR_250_MS       0x03
#define BMP280_ODR_500_MS       0x04
#define BMP280_ODR_1000_MS      0x05
#define BMP280_ODR_2000_MS      0x06
#define BMP280_ODR_4000_MS      0x07

/* Over-sampling macros */
#define BMP280_OS_NONE          0x00
#define BMP280_OS_1X            0x01
#define BMP280_OS_2X            0x02
#define BMP280_OS_4X            0x03
#define BMP280_OS_8X            0x04
#define BMP280_OS_16X           0x05

/* Filter coefficient macros */
#define BMP280_FILTER_OFF       0x00
#define BMP280_FILTER_COEFF_2   0x01
#define BMP280_FILTER_COEFF_4   0x02
#define BMP280_FILTER_COEFF_8   0x03
#define BMP280_FILTER_COEFF_16  0x04

/*! @name SPI 3-Wire macros */
#define BMP280_SPI3_WIRE_ENABLE     0x01
#define BMP280_SPI3_WIRE_DISABLE    0x00

/* Measurement status */
#define BMP280_MEAS_DONE        0x00
#define BMP280_MEAS_ONGOING     0x01

/* Position and mask macros */
#define BMP280_STATUS_IM_UPDATE_POS     0x00
#define BMP280_STATUS_IM_UPDATE_MASK    0x01
#define BMP280_STATUS_MEAS_POS          0x03
#define BMP280_STATUS_MEAS_MASK         0x08
#define BMP280_OS_TEMP_POS              0x05
#define BMP280_OS_TEMP_MASK             0xE0
#define BMP280_OS_PRES_POS              0x02
#define BMP280_OS_PRES_MASK             0x1C
#define BMP280_POWER_MODE_POS           0x00
#define BMP280_POWER_MODE_MASK          0x03
#define BMP280_STANDBY_DURN_POS         0x05
#define BMP280_STANDBY_DURN_MASK        0xE0
#define BMP280_FILTER_POS               0x02
#define BMP280_FILTER_MASK              0x1C
#define BMP280_SPI3_ENABLE_POS          0x00
#define BMP280_SPI3_ENABLE_MASK         0x01

/* Image update */
#define BMP280_IM_UPDATE_DONE           0x00
#define BMP280_IM_UPDATE_ONGOING        0x01

/* Calibration parameters' relative position */
#define BMP280_DIG_T1_LSB_POS   0
#define BMP280_DIG_T1_MSB_POS   1
#define BMP280_DIG_T2_LSB_POS   2
#define BMP280_DIG_T2_MSB_POS   3
#define BMP280_DIG_T3_LSB_POS   4
#define BMP280_DIG_T3_MSB_POS   5
#define BMP280_DIG_P1_LSB_POS   6
#define BMP280_DIG_P1_MSB_POS   7
#define BMP280_DIG_P2_LSB_POS   8
#define BMP280_DIG_P2_MSB_POS   9
#define BMP280_DIG_P3_LSB_POS   10
#define BMP280_DIG_P3_MSB_POS   11
#define BMP280_DIG_P4_LSB_POS   12
#define BMP280_DIG_P4_MSB_POS   13
#define BMP280_DIG_P5_LSB_POS   14
#define BMP280_DIG_P5_MSB_POS   15
#define BMP280_DIG_P6_LSB_POS   16
#define BMP280_DIG_P6_MSB_POS   17
#define BMP280_DIG_P7_LSB_POS   18
#define BMP280_DIG_P7_MSB_POS   19
#define BMP280_DIG_P8_LSB_POS   20
#define BMP280_DIG_P8_MSB_POS   21
#define BMP280_DIG_P9_LSB_POS   22
#define BMP280_DIG_P9_MSB_POS   23
#define BMP280_CALIB_DATA_SIZE  24

/* Function pointer type definitions */
typedef uint8_t (*bmp280_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, void *data, uint8_t len);

typedef int (*bmp280_delay_fptr_t)(unsigned long period);

/* Bit-slicing macros */
#define BMP280_GET_BITS(bitname, x)                     ((x & bitname##_MASK) >> bitname##_POS)
#define BMP280_SET_BITS(regvar, bitname, val)           ((regvar & ~bitname##_MASK) | ((val << bitname##_POS) & bitname##_MASK))
#define BMP280_SET_BITS_POS_0(reg_data, bitname, data)  ((reg_data & ~(bitname##_MASK)) | (data & bitname##_MASK))
#define BMP280_GET_BITS_POS_0(bitname, reg_data)        (reg_data & (bitname##_MASK))

/* Calibration parameters' structure */
struct bmp280_calib_param {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    int32_t t_fine;
};

/* Sensor configuration structure */
struct __attribute__((__packed__)) bmp280_config {
    uint8_t os_temp;
    uint8_t os_pres;
    uint8_t odr;
    uint8_t filter;
    uint8_t spi3w_en;
};

/* Sensor status structure */
struct bmp280_status {
    uint8_t measuring;
    uint8_t im_update;
};

/* Uncompensated data structure */
struct bmp280_uncomp_data {
    uint32_t uncomp_temp;
    uint32_t uncomp_press;
};

struct __attribute__((__packed__)) bmp280_comp_data {
    uint32_t timestamp;
    uint32_t comp_temp;
    uint32_t comp_press;
};

/* API device structure */
struct bmp280_dev {
    uint8_t chip_id;
    uint8_t dev_id;
    uint8_t intf;
    bmp280_com_fptr_t read;
    bmp280_com_fptr_t write;
    bmp280_delay_fptr_t delay_ms;
    struct bmp280_calib_param calib_param;
    struct bmp280_config conf;
    struct bmp280_uncomp_data uncomp_data;
    struct bmp280_comp_data data;
};


/*!
 * This API reads the data from the given register address of the
 * sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len : No of bytes of data to be read.
 * @param[in] dev : Structure instance of bmp280_dev.
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise.
 */
int8_t bmp280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, const struct bmp280_dev *dev);

/*!
 * @brief This API writes the given data to the register addresses
 * of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be written.
 * @param[in] reg_data : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len : No of bytes of data to write..
 * @param[in] dev : Structure instance of bmp280_dev.
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise.
 */
int8_t bmp280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bmp280_dev *dev);

/*!
 * @brief This API triggers the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bmp280_dev.
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise..
 */
int8_t bmp280_soft_reset(const struct bmp280_dev *dev);

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 *
 *  @param[in,out] dev : Structure instance of bmp280_dev
 *
 *  @return Result of API execution
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bmp280_init(struct bmp280_dev *dev);

/*!
 * @brief This API reads the data from the ctrl_meas register and config
 * register. It gives the currently set temperature and pressure over-sampling
 * configuration, power mode configuration, sleep duration and
 * IIR filter coefficient.
 *
 * @param[out] conf : Current configuration of the bmp280
 * conf.osrs_t, conf.osrs_p = BMP280_OS_NONE, BMP280_OS_1X,
 *     BMP280_OS_2X, BMP280_OS_4X, BMP280_OS_8X, BMP280_OS_16X
 * conf.odr = BMP280_ODR_0_5_MS, BMP280_ODR_62_5_MS, BMP280_ODR_125_MS,
 *     BMP280_ODR_250_MS, BMP280_ODR_500_MS, BMP280_ODR_1000_MS,
 *     BMP280_ODR_2000_MS, BMP280_ODR_4000_MS
 * conf.filter = BMP280_FILTER_OFF, BMP280_FILTER_COEFF_2,
 *     BMP280_FILTER_COEFF_4, BMP280_FILTER_COEFF_8, BMP280_FILTER_COEFF_16
 * conf.spi3w_en = BMP280_SPI3_WIRE_ENABLE, BMP280_SPI3_WIRE_DISABLE
 * @param[in] dev : Structure instance of bmp280_dev
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise.
 */
int8_t bmp280_get_config(struct bmp280_config *conf, struct bmp280_dev *dev);

/*!
 * @brief This API writes the data to the ctrl_meas register and config register.
 * It sets the temperature and pressure over-sampling configuration,
 * power mode configuration, sleep duration and IIR filter coefficient.
 *
 * @param[in] conf : Desired configuration to the bmp280
 * conf.osrs_t, conf.osrs_p = BMP280_OS_NONE, BMP280_OS_1X,
 *     BMP280_OS_2X, BMP280_OS_4X, BMP280_OS_8X, BMP280_OS_16X
 * conf.odr = BMP280_ODR_0_5_MS, BMP280_ODR_62_5_MS, BMP280_ODR_125_MS,
 *     BMP280_ODR_250_MS, BMP280_ODR_500_MS, BMP280_ODR_1000_MS,
 *     BMP280_ODR_2000_MS, BMP280_ODR_4000_MS
 * conf.filter = BMP280_FILTER_OFF, BMP280_FILTER_COEFF_2,
 *     BMP280_FILTER_COEFF_4, BMP280_FILTER_COEFF_8, BMP280_FILTER_COEFF_16
 * conf.spi3w_en = BMP280_SPI3_WIRE_ENABLE, BMP280_SPI3_WIRE_DISABLE
 * @param[in] dev : Structure instance of bmp280_dev
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise.
 */
int8_t bmp280_set_config(const struct bmp280_config *conf, struct bmp280_dev *dev);

/*!
 * @brief This API reads the status register
 *
 * @param[out] status : Status of the sensor
 * status.measuring = BMP280_MEAS_DONE, BMP280_MEAS_ONGOING
 * status.im_update = BMP280_IM_UPDATE_DONE, BMP280_IM_UPDATE_ONGOING
 * @param[in] dev : structure instance of bmp280_dev
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise.
 */
int8_t bmp280_get_status(struct bmp280_status *status, const struct bmp280_dev *dev);

/*!
 * @brief This API reads the power mode.
 *
 * @param[out] mode : BMP280_SLEEP_MODE, BMP280_NORMAL_MODE,
 *     BMP280_FORCED_MODE
 * @param[in] dev : Structure instance of bmp280_dev
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise.
 */
int8_t bmp280_get_power_mode(uint8_t *mode, const struct bmp280_dev *dev);

/*!
 * @brief This API writes the power mode.
 *
 * @param[out] mode : BMP280_SLEEP_MODE, BMP280_NORMAL_MODE,
 *     BMP280_FORCED_MODE
 * @param[in] dev : Structure instance of bmp280_dev
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise.
 */
int8_t bmp280_set_power_mode(uint8_t mode, struct bmp280_dev *dev);

/*!
 * @brief This API reads the temperature and pressure data registers.
 * It gives the raw temperature and pressure data.
 *
 * @param[in] uncomp_data : Structure instance of bmp280_uncomp_data
 * @param[in] dev : Structure instance of bmp280_dev
 *
 * @return Result of API execution
 * @retval Zero for Success, non-zero otherwise.
 */
int8_t bmp280_get_uncomp_data(struct bmp280_uncomp_data *uncomp_data, const struct bmp280_dev *dev);

/*!
 * @brief This API is used to get the compensated temperature from
 * uncompensated temperature. This API uses 32 bit integers.
 *
 * @param[in] uncomp_temp : Raw temperature values from the sensor
 * @param[in] dev : Structure instance of bmp280_dev
 *
 * @return Temperature in degC, resolution is 0.01 DegC. output value of
 * "5123" equals 51.23 degree Celsius.
 *
 */
int32_t bmp280_comp_temp_32bit(uint32_t uncomp_temp, struct bmp280_dev *dev);

/*!
 * @brief This API is used to get the compensated pressure from
 * uncompensated pressure. This API uses 32 bit integers.
 *
 * @param[in] uncomp_pres : Raw pressure values from the sensor
 * @param[in] dev : structure instance of bmp280_dev
 *
 * @return Pressure in Pa as unsigned 32 bit integer
 * output value of "96386" equals 96386 Pa = 963.86 hPa
 */
uint32_t bmp280_comp_pres_32bit(uint32_t uncomp_pres, const struct bmp280_dev *dev);


/*!
 * @brief This API computes the measurement time in milliseconds for the
 * active configuration
 *
 * @param[in] dev : Structure instance of bmp280_dev
 *
 * @return Measurement time for the active configuration in milliseconds
 */
uint8_t bmp280_compute_meas_time(const struct bmp280_dev *dev);

#endif /* BMP280_H_ */
