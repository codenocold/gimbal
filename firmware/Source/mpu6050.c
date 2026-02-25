#include "mpu6050.h"
#include "soc.h"

// MPU6050 I2C SlaveAddr
#define MPU6050_ADDR 0xD0 // 0xD0:AD0 = low 0xD2:AD0 = high

static int32_t write_register(uint8_t reg, uint8_t data);
static int32_t i2c_read(uint8_t SlaveAddr, uint8_t reg, uint8_t *pBuffer, uint8_t NumByteToRead);
static int32_t i2c_write(uint8_t SlaveAddr, uint8_t reg, uint8_t *pBuffer, uint8_t NumByteToWrite);

/**************************************************************************
 * @bref 	PIOS_MPU6050_init
 * @param 	none
 * @retval 	(0)  Succese
 			(<0) Error code
**************************************************************************/
int32_t MPU6050_init(void)
{
    int32_t ret = 0;

    // Wait MPU6050 stable
    soc_delay_ms(50);

    // Chose CLK Source
    ret += write_register(0x6B, 0x03);

    // Sample Rate Divider
    ret += write_register(0x19, 0x00);

    // Set DLPF
    ret += write_register(0x1A, 0x06);

    // (+-500 deg/s)
    ret += write_register(0x1B, 0x08);

    // (+-2 G)
    ret += write_register(0x1C, 0x02);

    // Check Who am I
    uint8_t buf[1];
    i2c_read(MPU6050_ADDR, 0x75, buf, 1);
    if (buf[0] != 0x68) {
        ret += -1;
    }

    // Wait MPU6050 stable
    soc_delay_ms(50);

    return ret;
}

/**************************************************************************
 * @bref 		MPU6050_read_raw_data
 * @param[out] 	acce_x: accel X raw data ptr
 * @param[out] 	acce_y: accel Y raw data ptr
 * @param[out] 	acce_z: accel Z raw data ptr
 * @param[out] 	gyro_x: gyro  X raw data ptr
 * @param[out] 	gyro_y: gyro  Y raw data ptr
 * @param[out] 	gyro_z: gyro  Z raw data ptr
 * @param[out] 	temper: Temperature raw data ptr
 * @retval 		(0)  Succese
 				(-1) Error
**************************************************************************/
int32_t MPU6050_read_raw_data(int16_t *acce_x, int16_t *acce_y, int16_t *acce_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z, int16_t *temper)
{
    uint8_t I2C_Rx_Buffer[14];

    /* Read row Data */
    if (i2c_read(MPU6050_ADDR, 0x3B, I2C_Rx_Buffer, 14) != 0) {
        return (-1);
    }

    // Accel_X Y Z
    *acce_x = (int16_t) ((I2C_Rx_Buffer[0] << 8) | I2C_Rx_Buffer[1]);
    *acce_y = (int16_t) ((I2C_Rx_Buffer[2] << 8) | I2C_Rx_Buffer[3]);
    *acce_z = (int16_t) ((I2C_Rx_Buffer[4] << 8) | I2C_Rx_Buffer[5]);

    // Temperature
    *temper = (int16_t) ((I2C_Rx_Buffer[6] << 8) | I2C_Rx_Buffer[7]);

    // Gyro_X Y Z
    *gyro_x = (int16_t) ((I2C_Rx_Buffer[8] << 8) | I2C_Rx_Buffer[9]);
    *gyro_y = (int16_t) ((I2C_Rx_Buffer[10] << 8) | I2C_Rx_Buffer[11]);
    *gyro_z = (int16_t) ((I2C_Rx_Buffer[12] << 8) | I2C_Rx_Buffer[13]);

    return (0);
}

/**************************************************************************
 * @bref 		write_register
 * @param[in] 	reg: MPU6050 register
 * @param[in] 	data: Write byte
 * @retval 		(0)  Succese
 				(-1) Error
**************************************************************************/
static int32_t write_register(uint8_t reg, uint8_t data)
{
    uint8_t buf[1];

    buf[0] = data;
    i2c_write(MPU6050_ADDR, reg, buf, 1);
    buf[0] = 0;
    i2c_read(MPU6050_ADDR, reg, buf, 1);

    if (buf[0] != data) {
        return (-1);
    } else {
        return (0);
    }
}

//---------------------------------------------------------------------------------------------------------------
// I2C
#define IIC_DELAY_TICK 30
#define SCL_H          HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_SET)
#define SCL_L          HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_RESET)
#define SDA_H          HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_SET)
#define SDA_L          HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_RESET)

static __INLINE void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = IMU_SDA_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_SDA_GPIO_Port, &GPIO_InitStruct);
}

static __INLINE void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = IMU_SDA_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull             = GPIO_PULLUP;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_SDA_GPIO_Port, &GPIO_InitStruct);
}

static __INLINE uint8_t SDA_READ(void)
{
    if (HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin)) {
        return 1;
    } else {
        return 0;
    }
}

/**************************************************************************
 * @bref 		i2c_delay
 * @param[in] 	cnt: couont number
 * @retval 		none
**************************************************************************/
static void i2c_delay(uint32_t cnt)
{
    __IO uint32_t i;
    for (i = 0; i < cnt; i++)
        ;
}

/**************************************************************************
 * @bref 	i2c_start
 * @param 	none
 * @retval 	none
**************************************************************************/
static void i2c_start(void)
{
    SDA_OUT();
    SDA_H;
    SCL_H;
    i2c_delay(IIC_DELAY_TICK);
    SDA_L;
    i2c_delay(IIC_DELAY_TICK);
    SCL_L;
}

/**************************************************************************
 * @bref 	i2c_stop
 * @param 	none
 * @retval 	none
**************************************************************************/
static void i2c_stop(void)
{
    SDA_OUT();
    SCL_L;
    SDA_L;
    i2c_delay(IIC_DELAY_TICK);
    SCL_H;
    SDA_H;
    i2c_delay(IIC_DELAY_TICK);
}

/**************************************************************************
 * @bref 	i2c_ack
 * @param 	none
 * @retval 	none
**************************************************************************/
static void i2c_ack(void)
{
    SDA_OUT();
    SDA_L;
    i2c_delay(IIC_DELAY_TICK);
    SCL_H;
    i2c_delay(IIC_DELAY_TICK);
    SCL_L;
}

/**************************************************************************
 * @bref 	i2c_nack
 * @param 	none
 * @retval 	none
**************************************************************************/
static void i2c_nack(void)
{
    SDA_OUT();
    SDA_H;
    i2c_delay(IIC_DELAY_TICK);
    SCL_H;
    i2c_delay(IIC_DELAY_TICK);
    SCL_L;
}

/**************************************************************************
 * @bref 	i2c_wait_ack
 * @param 	none
 * @retval 	(0)  Get ACK
 			(-1) Wait ack timeout
**************************************************************************/
static int32_t i2c_wait_ack(void)
{
    uint8_t i = 0;

    SDA_H;
    i2c_delay(IIC_DELAY_TICK);
    SDA_IN();
    SCL_H;
    i2c_delay(IIC_DELAY_TICK);
    while (SDA_READ()) {
        if (i > 250) {
            i2c_stop();
            return (-1);
        }
        i++;
    }
    SCL_L;

    return (0);
}

/**************************************************************************
 * @bref 		i2c_wirte_byte
 * @param[in] 	send: Byte tobe send
 * @retval 		none
**************************************************************************/
static void i2c_wirte_byte(uint8_t send)
{
    uint8_t i = 0;

    SDA_OUT();
    for (i = 0; i < 8; i++) {
        if ((send & 0x80) > 0) {
            SDA_H;
        } else {
            SDA_L;
        }
        send <<= 1;
        i2c_delay(IIC_DELAY_TICK);
        SCL_H;
        i2c_delay(IIC_DELAY_TICK);
        SCL_L;
        i2c_delay(IIC_DELAY_TICK);
    }
}

/**************************************************************************
 * @bref 		i2c_read_byte
 * @param[in] 	ack: Wehter need ack
 * @retval 		Received byte
**************************************************************************/
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i   = 0;
    uint8_t rec = 0;

    SDA_IN();
    for (i = 0; i < 8; i++) {
        i2c_delay(IIC_DELAY_TICK);
        SCL_H;
        i2c_delay(IIC_DELAY_TICK);
        rec <<= 1;
        if (SDA_READ()) {
            rec++;
        }
        SCL_L;
    }
    if (ack) {
        i2c_ack();
    } else {
        i2c_nack();
    }

    return rec;
}

/**************************************************************************
 * @bref 		i2c_read
 * @param[in] 	SlaveAddr: I2C device addr
 * @param[in] 	reg: read reg
 * @param[out] 	pBuffer: Read buffer ptr
 * @param[in] 	NumByteToRead: Number byte to read
 * @retval 		(0)	 Succese
 				(-1) Error
**************************************************************************/
static int32_t i2c_read(uint8_t SlaveAddr, uint8_t reg, uint8_t *pBuffer, uint8_t NumByteToRead)
{
    i2c_start();
    i2c_wirte_byte(SlaveAddr);
    if (i2c_wait_ack() != 0) {
        return (-1);
    }
    i2c_wirte_byte(reg);
    if (i2c_wait_ack() != 0) {
        return (-1);
    }
    i2c_start();
    i2c_wirte_byte(SlaveAddr + 1);
    if (i2c_wait_ack() != 0) {
        return (-1);
    }
    while (NumByteToRead--) {
        if (NumByteToRead == 0) {
            *pBuffer = i2c_read_byte(0);
        } else {
            *pBuffer = i2c_read_byte(1);
        }
        pBuffer++;
    }
    i2c_stop();

    return 0;
}

/**************************************************************************
 * @bref 		i2c_write
 * @param[in] 	SlaveAddr: I2C device addr
 * @param[in] 	reg: read reg
 * @param[in] 	pBuffer: write buffer ptr
 * @param[in] 	NumByteToWrite: Number byte to write
 * @retval 		(0)	 Succese
 				(-1) Error
**************************************************************************/
static int32_t i2c_write(uint8_t SlaveAddr, uint8_t reg, uint8_t *pBuffer, uint8_t NumByteToWrite)
{
    i2c_start();
    i2c_wirte_byte(SlaveAddr);
    if (i2c_wait_ack() != 0) {
        return (-1);
    }
    i2c_wirte_byte(reg);
    if (i2c_wait_ack() != 0) {
        return (-1);
    }
    while (NumByteToWrite--) {
        i2c_wirte_byte(*pBuffer);
        if (i2c_wait_ack() != 0) {
            return (-1);
        }
        pBuffer++;
    }
    i2c_stop();

    return 0;
}
