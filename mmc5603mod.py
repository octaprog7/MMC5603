"""MicroPython module for QMC5883L or HMC5883L Geomagnetic Sensor"""
import micropython

# The 1000 Hz ODR is available by writing 255 into Register ODR and setting hi_power to 1.
import array
# import micropython

# MicroPython
# mail: goctaprog@gmail.com
# MIT license
from sensor_pack import bus_service, geosensmod
from sensor_pack.base_sensor import check_value, Iterator, TemperatureSensor
import time

_meas_time_us = 6_600, 3_500, 2_000, 1_200


@micropython.native
def to_bit_tuple(source: int, bits: range) -> tuple:
    """Возвращает кортеж из битов source. Начиная от бита с индексом 0"""
    # if bytes_count not in range(5):
    #     raise ValueError("bad bytes_count")
    mask = 0x01    # маска
    return tuple(0 != source & (mask << shift) for shift in bits)


def _bytes_to_raw(source: bytes) -> int:
    """Из bytes в значение магнитной индукции (raw)"""
    val = source[0] << 12 | source[1] << 4 | source[2] >> 4
    val -= 524288
    return val


def _get_update_rate_limits(bandwidth: int, use_auto_set_reset: bool) -> tuple[int, int]:
    """Возвращает пределы update_rate в зависимости от bandwidth и use_auto_set_reset параметров"""
    check_value(bandwidth, range(4), f"Invalid bandwidth value: {bandwidth}")
    if 0 == bandwidth:
        if use_auto_set_reset:
            return 1, 75
        else:
            return 1, 150
    if 1 == bandwidth:
        if use_auto_set_reset:
            return 1, 50
        else:
            return 1, 255
    if bandwidth >= 2:
        return 1, 255


def axis_name_to_reg_addr(axis_name: int) -> tuple:
    """Функция-обертка. Преобразует имя оси 0('x'), 1('y'), 2('z')) в адрес соответствующего регистра"""
    return geosensmod.axis_name_to_reg_addr(axis_name, offset=0, multiplier=2), 6 + axis_name


class MMC5603(geosensmod.GeoMagneticSensor, Iterator, TemperatureSensor):
    """MMC5603 Geomagnetic Sensor."""

    def __init__(self, adapter: bus_service.BusAdapter, address: int = 0x30):
        check_value(address, valid_range=(0x30,), error_msg=f"Invalid address value: {address}")
        super().__init__(adapter=adapter, address=address, big_byte_order=False)  # little endian
        #
        self._buf_3 = bytearray((0, 0, 0))  # для хранения
        self._buf_9 = bytearray((0 for _ in range(9)))  # для хранения
        self._res = array.array('i', (0, 0, 0))  # signed int
        # self._bus_ref = self.adapter.bus
        #
        self._cmm = False   # continuous meas mode
        self._update_rate = 10   # 1..255, 1000 Гц
        self._bandwidth = 0     # 0..3
        self._axis_measurement = 'xyz'      # оси, по которым производится измерения!
        # для частоты обновления 1000 Гц. Непонятно только,
        # для чего магнитометру такое большое значение частоты измерений!??
        self._hi_power = False
        # определяет, сколько измерений будет выполнено перед автоматическим(!) выполнением do_set,
        # когда датчик находится в периодическом режиме измерений и включен автоматический do_set/do_reset.
        # От 000 до 111! Датчик будет выполнять один do_set для каждых 1, 25, 75, 100, 250, 500, 1000 и 2000 измерений.
        # Чтобы включить эту функцию, En_prd_set и Auto_SR должны быть установлены в 1,
        # а датчик должен работать в непрерывном режиме. Никакого отношения к программному сбросу состояния датчика
        # процедура do_set/do_reset не имеет!!!
        self._do_set_execute_period = 1     # рекомендованное, из datasheet, значение 1
        # не переключать в False, иначе датчик чувствительные элементы
        # датчика намагнитятся и ваши измерения будут неточны!!!
        self._periodical_set_en = True  # автоматическое выполнение do_set/do_reset во время измерений
        self.setup()

    @property
    def is_auto_set_reset(self) -> bool:
        """возвращает Истина, если автоматическое выполнение do_set/do_reset во время измерений включено"""
        return self._periodical_set_en

    @is_auto_set_reset.setter
    def is_auto_set_reset(self, value: bool):
        self._periodical_set_en = value

    def _read_reg_(self, addr: int) -> int:
        """Чтение одного байта из регистра"""
        check_value(addr, range(0x3A), f"Invalid reg address: {addr}")
        return self._read_reg(addr)[0]

    def _get_bandwidth_and_update_rate(self, update_rate: int) -> tuple[int, int]:
        """Возвращает число для bandwidth и output_data_rate,
        соответствующие заданному update_rate и self.is_auto_set_reset"""
        error_msg = f"Invalid update rate: {update_rate}"
        check_value(update_rate, range(1, 256), error_msg)
        if 255 < update_rate != 1000:
            raise ValueError(error_msg)
        self._hi_power = 1000 == update_rate
        _use_auto_set_reset = self.is_auto_set_reset

        for _bw in range(4):
            lo, hi = _get_update_rate_limits(_bw, _use_auto_set_reset)
            if update_rate in range(lo, 1+hi):
                return _bw, update_rate

        if 1000 == update_rate:
            return 3, 255
        return 0, 0

    def set_update_rate(self, update_rate: int):
        """Устанавливает частоту обновления данных датчиком, Гц, измерений в секунду.
        use_auto_set_reset - если Истина, то используется автоматическое размагничивание сердечников катушек датчика.
        Возвращает ширину полосы пропускания (bandwidth) 0..3"""
        self._bandwidth, self._update_rate = self._get_bandwidth_and_update_rate(update_rate)

    def get_update_rate(self) -> int:
        """возвращает частоту обновления данных датчиком"""
        if self._hi_power:
            return 1000
        return self._update_rate

    def _control_0(
            self,
            cmm_freq_en: [bool, None] = False,  # bit 7. Запись 1 в этот бит запустит расчет периода измерения в соответствии с ODR. Этот бит должен быть установлен до начала измерений в непрерывном режиме. Этот бит автоматически очищается после расчета периода измерения внутренними схемами.
            auto_st_en: [bool, None] = False,  # bit 6. Запись 1 в этот бит активирует функцию автоматического самотестирования. Порог в регистрах 1EH, 1FH, 20H должен быть установлен до того, как этот бит будет установлен в 1. Этот бит очищается после завершения операции.
            auto_sr_en: [bool, None] = False,  # bit 5. Запись 1 в этот бит активирует функцию автоматической установки/сброса. Эта функция применима как к измерениям по запросу, так и к измерениям в непрерывном режиме. Этот бит должен быть установлен в 1, чтобы активировать функцию периодической установки. В приложении рекомендуется установить этот бит в «1».
            do_reset: [bool, None] = False,  # bit 4. Запись 1 в этот бит приведет к тому, что чип выполнит операцию размагничивания, что позволит большому току размагничивания течь через катушки датчика в течение 375 нс. Этот бит автоматически очищается в конце операции размагничивания.
            do_set: [bool, None] = False,  # bit 3. Запись 1 в этот бит приведет к тому, что чип выполнит операцию намагничивания, что позволит большому току намагничивания течь через катушки датчика в течение 375 нс. Этот бит автоматически очищается в конце операции намагничивания.
            tm_t: [bool, None] = False,  # bit 1. Запись 1 в этот бит заставляет чип выполнять измерение температуры. Этот бит самоочищается в конце каждого измерения.
            tm_m: [bool, None] = False,  # bit 0. Запись 1 в этот бит заставляет чип выполнять измерение магнитного поля. Этот бит самоочищается в конце каждого измерения.
    ):
        """Control 0 Register."""
        val = 0     # self._read_reg_(0x1B)
        if cmm_freq_en is not None:
            val &= ~(1 << 7)  # mask
            val |= cmm_freq_en << 7
        if auto_st_en is not None:
            val &= ~(1 << 6)  # mask
            val |= auto_st_en << 6
        if auto_sr_en is not None:
            val &= ~(1 << 5)  # mask
            val |= auto_sr_en << 5
        if do_reset is not None:
            val &= ~(1 << 4)  # mask
            val |= do_reset << 4
        if do_set is not None:
            val &= ~(1 << 3)  # mask
            val |= do_set << 3
        if tm_t is not None:
            val &= ~(1 << 1)  # mask
            val |= tm_t << 1
        if tm_m is not None:
            val &= ~1  # mask
            val |= tm_m
        self._write_reg(0x1B, val, 1)

    def _control_1(
            self,
            sw_reset: [bool, None] = False,  # bit 7. Программный сброс. Запись «1» приведет к перезагрузке устройства, аналогично включению питания. Он очистит все регистры, а также перечитает OTP в рамках процедуры запуска. Время включения составляет 20 мс!
            st_enm: [bool, None] = False,  # bit 6. Функция этого бита аналогична st_enp, но смещение магнитного поля имеет противоположную полярность!
            st_enp: [bool, None] = False,  # bit 5. Запись 1 в этот бит приведет к прохождению постоянного тока через катушку самотестирования датчика. Этот ток вызовет смещение магнитного поля. Эта функция используется для проверки насыщения датчика!
            z_inhibit: [bool, None] = False,  # bit 4. запись «1» отключит этот канал и уменьшит время измерения и общий заряд, затрачиваемый на измерение.
            y_inhibit: [bool, None] = False,  # bit 3. то же, что z_inhibit
            x_inhibit: [bool, None] = False,  # bit 2. то же, что z_inhibit
            bandwidth: int = 0,     # bit 1, 0. Эти биты выбора полосы пропускания регулируют длину прореживающего фильтра. Они контролируют продолжительность каждого измерения.
    ):
        """Control 1 Register."""
        val = 0     # self._read_reg_(0x1C)
        if sw_reset is not None:
            val &= ~(1 << 7)  # mask
            val |= sw_reset << 7
        if st_enm is not None:
            val &= ~(1 << 6)  # mask
            val |= st_enm << 6
        if st_enp is not None:
            val &= ~(1 << 5)  # mask
            val |= st_enp << 5
        if z_inhibit is not None:
            val &= ~(1 << 4)  # mask
            val |= z_inhibit << 4
        if y_inhibit is not None:
            val &= ~(1 << 3)  # mask
            val |= y_inhibit << 3
        if x_inhibit is not None:
            val &= ~(1 << 2)  # mask
            val |= x_inhibit << 2
        if bandwidth is not None:
            val &= ~0b11  # mask
            val |= bandwidth
        self._write_reg(0x1C, val, 1)

    def _control_2(
            self,
            hi_power: [bool, None] = False,  # bit 7. Если этот бит установлен в 1, то ODR будет равна 1000 Гц!
            int_meas_done_en: [bool, None] = False,  # bit 6. Не использовать!!!
            int_mdt_en: [bool, None] = False,  # bit 5. Не использовать!!!
            cmm_en: [bool, None] = False,  # bit 4. Устройство перейдет в непрерывный режим измерений, если для ODR установлено ненулевое значение и в Cmm_freq_en записана 1. Внутренний счетчик начнет считать!
            en_prd_set: [bool, None] = False,  # bit 3. Запись 1 в это место активирует функцию периодического выполнения процедуры set.
            prd_set: int = 0,     # bit 2, 1, 0. Эти биты определяют, сколько измерений будет выполнено перед выполнением процедуры set, когда датчик находится в непрерывном режиме измерений и включена автоматическая set/reset. От 000 до 111!
    ):
        """Control 2 Register."""
        val = 0     # self._read_reg_(0x1D)
        if hi_power is not None:
            val &= ~(1 << 7)  # mask
            val |= hi_power << 7
        if int_meas_done_en is not None:
            val &= ~(1 << 6)  # mask
            val |= int_meas_done_en << 6
        if int_mdt_en is not None:
            val &= ~(1 << 5)  # mask
            val |= int_mdt_en << 5
        if cmm_en is not None:
            val &= ~(1 << 4)  # mask
            val |= cmm_en << 4
        if en_prd_set is not None:
            val &= ~(1 << 3)  # mask
            val |= en_prd_set << 3
        if prd_set is not None:
            val &= ~0b111  # mask
            val |= prd_set
        self._write_reg(0x1D, val, 1)

    @property
    def is_periodical_set(self) -> bool:
        """Возвращает Истина, когда разрешено периодическое размагничивание частей датчика!"""
        return self._periodical_set_en

    @is_periodical_set.setter
    def is_periodical_set(self, value: bool):
        self._periodical_set_en = value

    @property
    def set_execute_period(self) -> int:
        """Возвращает число от 0 до 7 включительно. смотри set_execute_period"""
        return self._do_set_execute_period

    @set_execute_period.setter
    def set_execute_period(self, value: int):
        """Определяет, сколько измерений будет выполнено перед автоматическим(!) выполнением процедуры do_set
        value   count_samples
        0       1
        1       25
        2       75
        3       100
        4       250
        5       500
        6       1000
        7       2000"""
        check_value(value, valid_range=range(8), error_msg=f"Invalid set execute period: {value}")
        self._do_set_execute_period = value

    @property
    def axis_measurement(self) -> str:
        """Оси, по которым датчик производит измерения магнитного поля после вызова start_measure. 'xyz', 'xy', 'yz'"""
        return self._axis_measurement

    @axis_measurement.setter
    def axis_measurement(self, value: str):
        """Устанавливает поле _axis_measurement"""
        def_axis = 'xyz'    # valid range
        _tmp = value.lower()
        s = [item for item in value if item not in def_axis]
        check_value(len(s), range(0, 1), error_msg=f"Invalid axis: {s}")
        self._axis_measurement = value

    def do_set(self):
        """Выполняет операцию Set, что вызывает протекание тока через катушки датчика в течение 375 нс.
        Этот бит автоматически очищается в конце операции! Намагничивает!"""
        self._control_0(do_set=True)

    def do_reset(self):
        """Выполняет операцию сброса, что вызывает протекание тока сброса(!) течь через катушки датчика
        в течение 375 нс. Этот бит автоматически очищается в конце операции сброса! Размагничивает!"""
        self._control_0(do_reset=True)

    def demagnetize(self, on_time_us: int = 500):
        """Размагничивание путем подачи тока через катушку самотестирования в разных направлениях"""
        try:
            self._control_1(st_enp=True, st_enm=False)      # ток потек в одном направлении
            time.sleep_us(on_time_us)
            self._control_1(st_enp=False, st_enm=True)      # ток потек в другом направлении
            time.sleep_us(on_time_us)
        finally:    # выключаю ток!
            self._control_1(st_enp=False, st_enm=False)

    def _read_reg(self, reg_addr: int, bytes_count: int = 1) -> bytes:
        """Считывает значение из регистра по адресу регистра reg_addr"""
        return self.adapter.read_register(self.address, reg_addr, bytes_count)

    def _write_reg(self, reg_addr: int, value: int, bytes_count: int = 1):
        """Записывает в регистр с адресом reg_addr значение value по шине."""
        bo = self._get_byteorder_as_str()[0]
        self.adapter.write_register(self.address, reg_addr, value, bytes_count, bo)

    def perform_self_test(self) -> bool:
        """Самотестирование датчика. Если возвратит Истина, то проверка пройдена УСПЕШНО!
        Алгоритм смотри в документации на стр. 14. 'EXAMPLE OF SELFTEST'"""
        # stored = self._read_reg(0x27, 3)    # read axis selftest set value (0x27, 0x28, 0x29)
        stored = self._buf_3
        adapt = self.adapter
        adapt.read_buf_from_mem(self.address, 0x27, stored)  # 3x(3x8) bit value. читаю в буфер из датчика 3 байта
        for offs, val in enumerate(map(lambda x: 8 * x // 10, stored)):
            stored[offs] = val
        adapt.write_buf_to_mem(self.address, 0x1E, stored)  # записываю из буфера в датчик 3 байта
        self._control_0(auto_st_en=True, tm_m=True)  # произвести самотестирование
        time.sleep_ms(20)   # задержка для выполнения самопроверки
        # Этот бит является индикатором прохождения самотестирования.
        # Он остается False, если после самотестирования, датчик прошел проверку!
        # То есть его измерительные катушки НЕ намагничены!
        stat = self.get_status()
        # значение бита Sat_sensor в регистре состояния
        return 0 == stat[1]

    def get_id(self):
        """Возвращает значение (Chip ID), которое равно 0x10!"""
        return self._read_reg(0x39)[0]

    def _enable_temp_meas(self, enable: bool = True):
        """Включает/выключает измерение температуры"""
        # Этот бит самоочищается в конце каждого измерения!
        self._control_0(tm_t=enable)     # произвести(True) или нет(False) измерение температуры.

    def get_temperature(self, coefficient: float = 0.8) -> [int, float]:
        """Возвращает температуру в градусах Цельсия"""
        self._enable_temp_meas(True)
        #
        raw = self._read_reg(0x09, 1)
        return -75 + coefficient * self.unpack("B", raw)[0]  # unsigned char

    def soft_reset(self):
        # software reset
        self._write_reg(reg_addr=0x1C, value=0b1000_0000)

    def is_continuous_meas_mode(self) -> bool:
        """Возвращает Истина, когда включен режим периодических измерений!"""
        # return 0 != (0x10 & self._get_ctrl(2))
        return self._cmm

    def in_standby_mode(self) -> bool:
        """Возвращает Истина, когда включен режим ожидания(экономичный режим)!"""
        return False

    def get_status(self) -> tuple:
        """Возвращает кортеж битов(номер бита): OTP_read_done(4), Sat_sensor(5), Meas_m_done(6), Meas_t_done(7)"""
        stat = self._read_reg(0x18)[0]
        #       4,                  5,          6,              7
        # OTP_read_done(0), Sat_sensor(1), Meas_m_done(2), Meas_t_done(3)
        return to_bit_tuple(stat, range(4, 8))

    def is_data_ready(self) -> bool:
        """Возвращает флаг Data Ready.
        This bit indicates that a measurement of magnetic field is done and the data is ready to be read."""
        return self.get_status()[2]

    def start_measure(self, continuous_mode: bool = True, auto_set_reset: bool = True):
        """Запускает периодические измерения (continuous_mode is True) или измерение по запросу
        (continuous_mode is False)...
        update_rate: 1..255
        До вызова этого метода нужно вызвать set_update_rate !!!"""
        # self._set_execute_period() !!!
        self.is_auto_set_reset = auto_set_reset
        self._write_reg(0x1A, self._update_rate, 1)     # установил update rate
        # Запустите расчет периода измерения по update_rate (ODR). Этот бит должен быть установлен до(!)
        # начала измерений в непрерывном режиме.
        self._control_0(cmm_freq_en=1)
        time.sleep_ms(10)   # ожидание завершения датчиком расчетов!
        _axis = self._axis_measurement
        #
        self._control_0(auto_sr_en=auto_set_reset)
        self._control_1(bandwidth=self._bandwidth,
                        x_inhibit='x' not in _axis,
                        y_inhibit='y' not in _axis,
                        z_inhibit='z' not in _axis)
        self._control_2(hi_power=self._hi_power,
                        en_prd_set=self._periodical_set_en,
                        cmm_en=continuous_mode,
                        prd_set=self.set_execute_period)
        # сохраняю режим измерений
        self._cmm = continuous_mode

    def read_raw(self, axis_name: int) -> int:
        """16, 18, 20 bits operation mode"""
        addresses = axis_name_to_reg_addr(axis_name)
        bts = bytearray(self._read_reg(reg_addr=addresses[0], bytes_count=2))   # два байта
        bts.append(self._read_reg(reg_addr=addresses[1], bytes_count=1)[0])     # один байт
        # ret
        return _bytes_to_raw(bts)

    def _get_all_meas_result(self) -> tuple:
        # чтение всех данных!
        b = self._buf_3
        res = self._res
        buf9 = self._buf_9
        #
        self.adapter.read_buf_from_mem(self.address, 0, buf9)       # 3x(3x8) bit value
        for axis in range(3):
            addresses = axis_name_to_reg_addr(axis)
            b[0] = buf9[addresses[0]]
            b[1] = buf9[1 + addresses[0]]
            b[2] = buf9[addresses[1]]
            res[axis] = _bytes_to_raw(b)
        #
        return tuple(res)

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время, в микросекундах(!), преобразования датчиком в зависимости от его настроек.
        перед вызовом этого метода должен быть вызван метод set_update_rate !!!
        Если измерения по одной из осей (например х) отсутствуют, то общее время цикла измерения уменьшается на 1/3"""
        _axis = self._axis_measurement
        _bw = self._bandwidth
        return int(0.333 * len(_axis) * _meas_time_us[_bw])

    @property
    def band_width(self) -> int:
        """ Возвращает значение от 0 до 3 включительно. Устанавливается методом set_update_rate.
        Биты выбора полосы пропускания регулируют длину прореживающего фильтра. Из документации"""
        return self._bandwidth

    def setup(self):
        pass

    def __iter__(self):
        return self

    def __next__(self):
        """возвращает результат только в режиме периодических измерений!"""
        if self.is_continuous_meas_mode and self.is_data_ready():
            return self.get_axis(-1)
        return None
