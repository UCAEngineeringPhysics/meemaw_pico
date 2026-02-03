from machine import Pin, PWM


class BaseMotor:
    def __init__(self, phase_id, enable_id, sleep_id) -> None:
        self.phase_pin = Pin(phase_id, Pin.OUT)
        self.enable_pin = PWM(Pin(enable_id))  # PWM
        self.enable_pin.freq(5000)
        self.sleep_pin = Pin(sleep_id, Pin.OUT)
        self.snooze()

    def snooze(self):
        self.sleep_pin.off()

    def awaken(self):
        self.sleep_pin.on()

    def stop(self):
        self.enable_pin.duty_u16(0)

    def forward(self, speed=0.0):  # map 0~65535 to 0~1
        assert 0 <= speed <= 1  # make sure speed in range [0, 1]
        self.phase_pin.on()
        self.enable_pin.duty_u16(int(65535 * speed))

    def backward(self, speed=0.0):  # map 0~65535 to 0~1
        assert 0 <= speed <= 1  # make sure speed in range [0, 1]
        self.phase_pin.off()
        self.enable_pin.duty_u16(int(65535 * speed))


# TEST
if __name__ == "__main__":
    from utime import sleep

    # SETUP
    # m = BaseMotor(phase_id=16, enable_id=17, sleep_id=18)  # left motor
    m = BaseMotor(phase_id=15, enable_id=14, sleep_id=13)  # right motor

    # LOOP
    m.awaken()
    # Forwardly ramp up and down
    for i in range(100):
        m.forward((i + 1) / 100)
        print(f"f, dc: {i}%")
        sleep(4 / 100)  # 4 seconds to ramp up
    for i in reversed(range(100)):
        m.forward((i + 1) / 100)
        print(f"f, dc: {i}%")
        sleep(4 / 100)  # 4 seconds to ramp down
    # Backwardly ramp up and down
    for i in range(100):
        m.backward((i + 1) / 100)
        print(f"b, dc: {i}%")
        sleep(4 / 100)  # 4 seconds to ramp up
    for i in reversed(range(100)):
        m.backward((i + 1) / 100)
        print(f"b, dc: {i}%")
        sleep(4 / 100)  # 4 seconds to ramp down

    # Terminate
    m.stop()
    print("motor stopped.")
    sleep(0.5)  # full stop
    m.snooze()  # disable motor driver
    print("motor driver disabled.")
