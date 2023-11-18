# MicroPython
# mail: goctaprog@gmail.com
# MIT license
# import time
# Пожалуйста, прочитайте документацию на QMC5883L!
# Please read the QMC5883L documentation!
import math
import sys
from machine import I2C, Pin

import mmc5603mod
import time
from sensor_pack.bus_service import I2cAdapter


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # Внимание!!!
    # Замените id=1 на id=0, если пользуетесь первым портом I2C !!!
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)
    adapter = I2cAdapter(i2c)  # адаптер для стандартного доступа к шине
    delay_func = time.sleep_us  # микросекунды!!!

    sensor = mmc5603mod.MMC5603(adapter)
    print(f"Sensor id: {sensor.get_id()}")
    print(16 * "_")
    test_passed = sensor.perform_self_test()
    print(f"Самопроверка пройдена: {test_passed}")
    if not test_passed:
        print(f"Самопроверка НЕ пройдена! Прекращаем работу! :-)")
        sys.exit(-1)

    print("Демонстрация измерения температуры!")
    for _ in range(10):
        print(f"Температура от датчика: {sensor.get_temperature()}")
        time.sleep_ms(200)

    print("Внимание! Включите плоттер и вращайте датчик!")

    sensor.set_update_rate(20)
    print(f"Частота обновления данных [Гц]: {sensor.get_update_rate()}")
    print(f"bandwidth: {sensor.band_width}")
    print("Непрерывный режим измерения!")
    wt = sensor.get_conversion_cycle_time()
    print(f"Время преобразования [мкс]: {wt}")
    sensor.start_measure(continuous_mode=True, auto_set_reset=True)
    print(f"Это непрерывный режим измерения: {sensor.is_continuous_meas_mode()}")
    index = 0
    samples_count = 5000
    for mf_comp in sensor:
        delay_func(wt)
        if mf_comp:
            x = math.sqrt(sum(map(lambda val: val ** 2, mf_comp)))  # Величина магнитного поля в условных ед.
            print(f"X: {mf_comp[0]}; Y: {mf_comp[1]}; Z: {mf_comp[2]}; Магнитное поле [усл. ед.]: {x}")
        index += 1
        if index > samples_count:
            break

    sys.exit(0)
    # while True:
    #     if sensor.is_data_ready():
    #         x = sensor.get_meas_result('x')
    #         y = sensor.get_meas_result('y')
    #         z = sensor.get_meas_result('z')
    #         print(x, y, z)
