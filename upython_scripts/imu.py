from machine import Pin, I2C


class MPU6050:
    def __init__(self, scl_id=9, sda_id=8, i2c_addr=0x68):
        self.i2c = I2C(0, scl=Pin(scl_id), sda=Pin(sda_id), freq=400_000)
        self.i2c_addr = i2c_addr
        self.i2c.writeto_mem(
            self.i2c_addr,
            0x6B,  # PWR_MGMT_1 register address
            bytes([0x00]),  # data
        )  # wake up sensor
        # TODO: DHPF configuration
        # Variables
        self.lin_acc_x = 0.0
        self.lin_acc_y = 0.0
        self.lin_acc_z = 0.0
        self.ang_vel_x = 0.0
        self.ang_vel_y = 0.0
        self.ang_vel_z = 0.0

    def read_data(self):
        """
        MPU6050 uses 2 bytes to represent values of an entity.
        Acc_X, Acc_Y, Acc_Z, Temp, Gyro_X, Gyro_Y, Gyro_Z are stored in contiguous registers.
        To read'em all, grab 14 bytes starting at the ACCEL_XOUT_H register: 0x3B.
        """

        def process_raw(data, id, scale):
            """
            Args:
                data: a list contains n words(2 bytes) of sensor data
                id: 0 to n-1
                scale: a constant coefficient scaling raw data.
                       accelerometer: 16384 * 9.80665 per g
                       gyro: 131 per deg/s
                TODO: use radians
            Returns:
                value: human readible value in m/s^2 or deg/s
            """

            if data[id] > 32767:
                value = (data[id] - 65535) / scale
            else:
                value = data[0] / scale

            return value

        words = self.i2c.readfrom_mem(
            self.i2c_addr,
            0x3B,  # ACCEL_XOUT_H register address
            14,  # number of bytes
        )  # retrieve raw sensor data in bytes
        # Preprocess bytes, split 2 bytes as a group
        data = [words[i] << 8 | words[i + 1] for i in range(0, len(words), 2)]
        # Calculate human readibles
        self.lin_acc_x = process_raw(data, 0, 16384 * 9.80665)
        self.lin_acc_y = process_raw(data, 1, 16384 * 9.80665)
        self.lin_acc_z = process_raw(data, 2, 16384 * 9.80665)
        self.ang_vel_x = process_raw(data, 4, 131)
        self.ang_vel_y = process_raw(data, 5, 131)
        self.ang_vel_z = process_raw(data, 6, 131)


if __name__ == "__main__":
    from utime import ticks_ms, sleep_ms

    # SETUP
    try:
        sensor = MPU6050()
        print("IMU Connected!")
    except OSError:
        print("IMU Not Found - Check Wiring!")

    # LOOP
    while True:
        stamp = ticks_ms()
        sensor.read_data()
        # Log
        print(f"[Pico, {stamp}]:")
        print("---")
        print(
            f"acc_x={sensor.lin_acc_x} m/s^2, acc_y={sensor.lin_acc_y} m/s^2, acc_z={sensor.lin_acc_z} m/s^2"
        )
        print(
            f"angv_x={sensor.ang_vel_x} deg/s, angv_y={sensor.ang_vel_y} deg/s, angv_z={sensor.ang_vel_z} deg/s"
        )
        sleep_ms(50)  # 20Hz
