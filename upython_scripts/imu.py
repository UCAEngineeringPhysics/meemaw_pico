from machine import Pin, I2C


class MPU6050:
    def __init__(self, scl_id=9, sda_id=8, i2c_addr=0x68):
        self.iic = I2C(0, scl=Pin(scl_id), sda=Pin(sda_id), freq=400_000)
        self.addr = i2c_addr
        self.iic.writeto_mem(
            self.addr,  # i2c address
            0x6B,  # PWR_MGMT_1 register address
            bytes([0x00]),  # data
        )  # wake up sensor

    def _read_raw_data(self, addr):
        # Read two bytes (high and low) and combine them
        val = self.iic.readfrom_mem(self.addr, addr, 2)
        y = (val[0] << 8) | val[1]
        if y > 32768:
            y = y - 65536
        return y

    def get_values(self):
        # Raw readings for 6 axes
        # AcX, AcY, AcZ, Temp, GyX, GyY, GyZ are contiguous registers
        # For simplicity in this assignment, we read them individually
        # or you can read block 14 bytes for speed (advanced).

        # Accelerometer
        ac_x = self._read_raw_data(0x3B)
        ac_y = self._read_raw_data(0x3D)
        ac_z = self._read_raw_data(0x3F)

        # Gyroscope
        gy_x = self._read_raw_data(0x43)
        gy_y = self._read_raw_data(0x45)
        gy_z = self._read_raw_data(0x47)

        # Scaling (Default ranges: Accel +/- 2g, Gyro +/- 250 deg/s)
        # 16384.0 is the scale factor for 2g
        # 131.0 is the scale factor for 250 deg/s
        return {
            "AcX": ac_x / 16384.0,
            "AcY": ac_y / 16384.0,
            "AcZ": ac_z / 16384.0,
            "GyX": gy_x / 131.0,
            "GyY": gy_y / 131.0,
            "GyZ": gy_z / 131.0,
        }


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
        data = sensor.get_values()

        # Extract values for cleaner code
        ax = data["AcX"]
        ay = data["AcY"]
        az = data["AcZ"]
        gx = data["GyX"]
        gy = data["GyY"]
        gz = data["GyZ"]

        # 3. Format the "State Vector" Message
        # Structure: [Header]: Dist, Enc, Ax, Ay, Az, Gx, Gy, Gz
        msg = f"[Pico, {stamp}]: {ax:.2f}, {ay:.2f}, {az:.2f}, {gx:.2f}, {gy:.2f}, {gz:.2f}"

        print(msg)

        sleep_ms(16)  # Approx 60Hz
