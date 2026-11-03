/*
 * Uses the existing micro T-Kernel I2C driver.
 */

#include "imu_driver.h"
#include "../../device/include/dev_i2c.h"

/* LSM303DLHC accelerometer address */
#define IMU_ACCEL_ADDR       0x19

/* Accelerometer registers */
#define IMU_CTRL_REG1_A      0x20
#define IMU_CTRL_REG4_A      0x23
#define IMU_OUT_X_L_A        0x28

/*
 * CTRL_REG1_A = 0x47
 * 50 Hz output rate, all three axes enabled.
 *
 * CTRL_REG4_A = 0x88
 * Block data update enabled.
 * High-resolution mode, +/-2g measurement range.
 */
#define IMU_CTRL1_CONFIG    0x47
#define IMU_CTRL4_CONFIG    0x88

/* I2C1 is registered by the kernel as "iicb". */
LOCAL ID imu_dd = -1;

/*
 * convert two register bytes to signed acceleration.
 *
 * in high-resolution mode, the 12-bit result is
 * left-aligned in the 16-bit output registers.
 *
 * At +/-2g, one count is approximately 1 mg.
 */
LOCAL H imu_decode_axis(UB low, UB high)
{
    UW raw = ((UW)high << 8) | (UW)low;
    W value = (W)raw;

    if (raw & 0x8000U) {
        value -= 65536;
    }

    return (H)(value / 16);
}

/*
 * initialise the accelerometer.
 *
 * returns E_OK if communication and configuration
 * succeed, otherwise returns an error code.
 */
EXPORT ER imu_driver_init(void)
{
    ID dd;
    ER er;
    UB value = 0;

    if (imu_dd >= E_OK) {
        return E_OK;
    }

    dd = tk_opn_dev((UB*)"iicb", TD_UPDATE);

    if (dd < E_OK) {
        return (ER)dd;
    }

    /* Enable 50 Hz acceleration measurements. */
    er = i2c_write_reg(dd, IMU_ACCEL_ADDR,
                       IMU_CTRL_REG1_A, IMU_CTRL1_CONFIG);
    if (er < E_OK) {
        goto failed;
    }

    /* Enable high-resolution +/-2g mode. */
    er = i2c_write_reg(dd, IMU_ACCEL_ADDR,
                       IMU_CTRL_REG4_A, IMU_CTRL4_CONFIG);
    if (er < E_OK) {
        goto failed;
    }

    /* Verify the accelerometer configuration. */
    er = i2c_read_reg(dd, IMU_ACCEL_ADDR,
                      IMU_CTRL_REG1_A, &value);
    if (er < E_OK) {
        goto failed;
    }

    if (value != IMU_CTRL1_CONFIG) {
        er = E_IO;
        goto failed;
    }

    er = i2c_read_reg(dd, IMU_ACCEL_ADDR,
                      IMU_CTRL_REG4_A, &value);
    if (er < E_OK) {
        goto failed;
    }

    if (value != IMU_CTRL4_CONFIG) {
        er = E_IO;
        goto failed;
    }

    imu_dd = dd;
    return E_OK;

failed:
    (void)tk_cls_dev(dd, 0);
    return er;
}

/*
 * read all three acceleration axes in one transaction.
 *
 * the auto-increment bit allows six consecutive register bytes to be read together.
 */
EXPORT ER imu_driver_read_accel(IMU_ACCEL_SAMPLE *sample)
{
    T_I2C_EXEC exec = {0};
    UB reg = IMU_OUT_X_L_A | 0x80;
    UB data[6] = {0};
    SZ asz = 0;
    ER er;

    if (sample == NULL) {
        return E_PAR;
    }

    if (imu_dd < E_OK) {
        return E_OBJ;
    }

    exec.sadr = IMU_ACCEL_ADDR;
    exec.snd_size = 1;
    exec.snd_data = &reg;
    exec.rcv_size = sizeof(data);
    exec.rcv_data = data;

    er = tk_swri_dev(imu_dd, TDN_I2C_EXEC,
                     &exec, sizeof(exec), &asz);

    if (er < E_OK) {
        return er;
    }

    if (asz != (SZ)sizeof(exec)) {
        return E_IO;
    }

    sample->x_mg = imu_decode_axis(data[0], data[1]);
    sample->y_mg = imu_decode_axis(data[2], data[3]);
    sample->z_mg = imu_decode_axis(data[4], data[5]);

    return E_OK;
}

/* Release the I2C device. */
EXPORT void imu_driver_close(void)
{
    if (imu_dd >= E_OK) {
        (void)tk_cls_dev(imu_dd, 0);
        imu_dd = -1;
    }
}
