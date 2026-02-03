from regulated_wheel import RegulatedWheel


class DiffDriveController:
    def __init__(
        self, left_wheel_ids: list | tuple, right_wheel_ids: list | tuple
    ) -> None:
        # Configs
        self.left_wheel = RegulatedWheel(*left_wheel_ids)
        self.right_wheel = RegulatedWheel(*right_wheel_ids)
        self.snooze()
        # Constants
        self.wheel_sep = 0.223

    def awaken(self):
        self.left_wheel.awaken()
        self.right_wheel.awaken()

    def snooze(self):
        self.left_wheel.snooze()
        self.right_wheel.snooze()

    def get_vels(self):
        self.meas_lin_vel = 0.5 * (
            self.left_wheel.meas_lin_vel + self.right_wheel.meas_lin_vel
        )
        self.meas_ang_vel = (
            self.right_wheel.meas_lin_vel - self.left_wheel.meas_lin_vel
        ) / self.wheel_sep
        return self.meas_lin_vel, self.meas_ang_vel

    def set_vels(self, target_lin_vel, target_ang_vel):
        left_wheel_ref_vel = target_lin_vel - 0.5 * (target_ang_vel * self.wheel_sep)
        right_wheel_ref_vel = target_lin_vel + 0.5 * (target_ang_vel * self.wheel_sep)
        self.left_wheel.set_wheel_velocity(left_wheel_ref_vel)
        self.right_wheel.set_wheel_velocity(right_wheel_ref_vel)


if __name__ == "__main__":
    from utime import sleep

    # SETUP
    targ_vels = (
        (0.3, 0.0),
        (0.3, 1.6),
        (0.0, 1.6),
        (-0.3, 1.6),
        (-0.3, 0.0),
        (-0.3, -1.6),
        (0.0, -1.6),
        (0.3, -1.6),
    )
    ddc = DiffDriveController(
        left_wheel_ids=((16, 17, 18), (19, 20)),
        right_wheel_ids=((15, 14, 13), (12, 11)),
    )
    ddc.awaken()

    # LOOP
    for tv in targ_vels:
        ddc.set_vels(*tv)
        for _ in range(150):
            mlv, mav = ddc.get_vels()
            print(f"Differential drive lin_vel={mlv}, ang_vel={mav}")
            sleep(0.01)

    # Terminate
    ddc.snooze()
    print("motor drivers disabled.")
