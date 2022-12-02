from pneumatics_and_linear_rails.control import RS485Control

if __name__ == "__main__":
    pneumatics=Pneumatics(
        suction_read_port='/dev/ttyUSB0', suction_read_baud=9600,
        threshold_true_vacuum_kpa = -55.0,
        threshold_not_activated_kpa = -1.0)
    while 1:
        print(pneumatics.get_pressure())
        