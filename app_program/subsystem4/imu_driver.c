/* LSM303DLHC I2C accelerometer driver adapted from Buddy 4's a9e1c5c. */
#include "imu_driver.h"
#include "../../device/include/dev_i2c.h"

#define IMU_ACCEL_ADDR  (0x19U)
#define IMU_CTRL_REG1_A (0x20U)
#define IMU_CTRL_REG4_A (0x23U)
#define IMU_CTRL1_SETUP (0x47U) /* 50 Hz, all three axes. */
#define IMU_CTRL4_SETUP (0x88U) /* Block data update, high-resolution ±2 g. */
#define IMU_STATUS_A    (0x27U)
#define IMU_ZYX_READY   (0x08U)
#define IMU_OUT_X_L_A   (0x28U)
#define IMU_AUTOINCR    (0x80U)
#define IMU_AXIS_BYTES  (6U)
#define IMU_SIGN_BIT    (0x8000U)
#define IMU_WORD_COUNTS (65536)
#define IMU_RAW_SCALE   (16)

static ID g_imu_device = -1;

static ER imu_configure_register(ID device, UW reg, UB expected)
{
    UB actual = 0U;
    ER result = i2c_write_reg(device, IMU_ACCEL_ADDR, reg, expected);
    if (E_OK == result)
    {
        result = i2c_read_reg(device, IMU_ACCEL_ADDR, reg, &actual);
    }
    if ((E_OK == result) && (expected != actual))
    {
        result = E_IO;
    }
    return result;
}

static H imu_decode_axis(UB low, UB high)
{
    /* Widen two register bytes before interpreting the signed sample. */
    UW raw = ((UW)high << 8U) | (UW)low;
    /* The unsigned register word is at most 65535 and fits in W. */
    W value = (W)raw;
    if (0U != (raw & IMU_SIGN_BIT))
    {
        value -= IMU_WORD_COUNTS;
    }
    /* The high-resolution sample is signed 12-bit after scaling. */
    return (H)(value / IMU_RAW_SCALE);
}

EXPORT ER imu_driver_init(void)
{
    if (0 < g_imu_device)
    {
        return E_OK;
    }

    /* The kernel API uses UB strings and reads the device name. */
    ID device = tk_opn_dev((UB *)"iicb", TD_UPDATE);
    if (device < E_OK)
    {
        return device;
    }

    ER result = imu_configure_register(device, IMU_CTRL_REG1_A,
                                       IMU_CTRL1_SETUP);
    if (E_OK == result)
    {
        result = imu_configure_register(device, IMU_CTRL_REG4_A,
                                        IMU_CTRL4_SETUP);
    }
    if (result < E_OK)
    {
        (void)tk_cls_dev(device, 0);
        return result;
    }

    g_imu_device = device;
    return E_OK;
}

EXPORT ER imu_driver_sample_ready(bool *ready)
{
    if (NULL == ready)
    {
        return E_PAR;
    }
    if (g_imu_device <= 0)
    {
        return E_OBJ;
    }

    UB status = 0U;
    const ER result = i2c_read_reg(g_imu_device, IMU_ACCEL_ADDR,
                                  IMU_STATUS_A, &status);
    *ready = (E_OK == result) && (0U != (status & IMU_ZYX_READY));
    return result;
}

EXPORT ER imu_driver_read_accel(s4_accel_sample_t *sample)
{
    if (NULL == sample)
    {
        return E_PAR;
    }
    if (g_imu_device <= 0)
    {
        return E_OBJ;
    }

    UB reg = IMU_OUT_X_L_A | IMU_AUTOINCR;
    UB data[IMU_AXIS_BYTES] = {0};
    T_I2C_EXEC transfer =
    {
        .sadr = IMU_ACCEL_ADDR,
        .snd_size = 1,
        .snd_data = &reg,
        .rcv_size = IMU_AXIS_BYTES,
        .rcv_data = data
    };
    /* The fixed-size transaction descriptor fits the kernel's SZ type. */
    const SZ transfer_size = (SZ)sizeof(transfer);
    SZ written = 0;
    ER result = tk_swri_dev(g_imu_device, TDN_I2C_EXEC, &transfer,
                            transfer_size, &written);
    if (result < E_OK)
    {
        return result;
    }
    if (transfer_size != written)
    {
        return E_IO;
    }

    sample->x_mg = imu_decode_axis(data[0], data[1]);
    sample->y_mg = imu_decode_axis(data[2], data[3]);
    sample->z_mg = imu_decode_axis(data[4], data[5]);
    return E_OK;
}

EXPORT void imu_driver_close(void)
{
    if (0 < g_imu_device)
    {
        (void)tk_cls_dev(g_imu_device, 0);
        g_imu_device = -1;
    }
}
