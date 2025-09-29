import time
import serial
import pynmea2
import spidev
import RPi.GPIO as GPIO

from board_config import BOARD
from constants import MODE, REG

VESSEL_ID = "Vessel15774"   # ID судна

class LoRa:
    def __init__(self, verbose=False, gps_port="/dev/serial0", gps_baud=9600):
        self.verbose   = verbose

        # ----- ИНИЦИАЛИЗАЦИЯ SPI/GPIO -----
        BOARD.setup()
        self.nss   = BOARD.NSS
        self.reset = BOARD.RESET
        self.dio0  = BOARD.DIO0
        self.dio1  = getattr(BOARD, "DIO1", None)

        GPIO.setup(self.reset, GPIO.OUT)
        GPIO.setup(self.nss,   GPIO.OUT)
        GPIO.setup(self.dio0,  GPIO.IN)

        # SPI bus0, CE0 (GPIO8) -> NSS
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 500000

        # Reset чипа
        GPIO.output(self.nss, GPIO.HIGH)
        GPIO.output(self.reset, GPIO.LOW);  time.sleep(0.01)
        GPIO.output(self.reset, GPIO.HIGH); time.sleep(0.01)

        # Проверка версии (должна быть 0x12)
        ver = self.read_register(REG["VERSION"])
        if self.verbose:
            print(f"[LoRa] VERSION=0x{ver:02X}")

        # Инициализация радиопараметров (868 MHz, BW125, CR4/5, SF7, Sync 0x12)
        self.set_mode(MODE["SLEEP"])
        self.set_freq(868.0)
        self._set_bw_cr(7, 1)           # BW=125kHz, CR=4/5
        self._set_sf_crc(7, crc_on=True)
        self._set_sync_word(0x12)       # совместимо с RadioHead (RH_RF95)
        self.set_dio_mapping([0]*6)     # DIO0: RxDone в RX / TxDone в TX
        self.set_pa_config(pa_select=1) # ~14 dBm по умолчанию

        # GPS (ленивая инициализация)
        self._gps_port = gps_port
        self._gps_baud = gps_baud
        self._gps      = None
        self._last_fix = None           # (lat, lon, timestamp)

        # Переходим в приём
        self.reset_ptr_rx()
        self.set_mode(MODE["RXCONT"])

    # ---------------- SPI / REG ----------------
    def write_register(self, address, value):
        GPIO.output(self.nss, GPIO.LOW)
        self.spi.xfer2([address | 0x80, value & 0xFF])
        GPIO.output(self.nss, GPIO.HIGH)

    def read_register(self, address):
        GPIO.output(self.nss, GPIO.LOW)
        resp = self.spi.xfer2([address & 0x7F, 0x00])
        GPIO.output(self.nss, GPIO.HIGH)
        return resp[1]

    # ---------------- BASIC CONFIG ----------------
    def set_mode(self, mode):
        # включаем LoRa и ставим режим
        op = 0x80 | (mode & 0x07)
        self.write_register(REG["OP_MODE"], op)

    def set_freq(self, mhz):
        frf = int((mhz * 1_000_000.0) / 61.03515625)
        self.write_register(REG["FR_MSB"], (frf >> 16) & 0xFF)
        self.write_register(REG["FR_MID"], (frf >> 8) & 0xFF)
        self.write_register(REG["FR_LSB"], frf & 0xFF)

    def set_spreading_factor(self, sf):
        cur = self.read_register(REG["MODEM_CONFIG2"])
        self.write_register(REG["MODEM_CONFIG2"], (sf << 4) | (cur & 0x0F))

    def set_bandwidth(self, bw_code):
        cur = self.read_register(REG["MODEM_CONFIG1"])
        self.write_register(REG["MODEM_CONFIG1"], (bw_code << 4) | (cur & 0x0F))

    def set_coding_rate(self, cr_code):
        cur = self.read_register(REG["MODEM_CONFIG1"])
        self.write_register(REG["MODEM_CONFIG1"], (cur & 0xF1) | ((cr_code & 0x07) << 1))

    def set_preamble(self, length):
        self.write_register(REG["PREAMBLE_MSB"], (length >> 8) & 0xFF)
        self.write_register(REG["PREAMBLE_LSB"], length & 0xFF)

    def set_pa_config(self, pa_select=1, output_power=14):
        self.write_register(REG["PA_CONFIG"], ((pa_select & 0x01) << 7) | (output_power & 0x0F))

    def set_payload_length(self, length):
        self.write_register(REG["PAYLOAD_LENGTH"], length & 0xFF)

    def reset_ptr_rx(self):
        # FIFO RX base = 0x00
        self.write_register(REG["FIFO_RX_BASE_ADDR"], 0x00)
        self.write_register(REG["FIFO_ADDR_PTR"], 0x00)

    def get_irq_flags(self):
        return self.read_register(REG["IRQ_FLAGS"])

    def set_dio_mapping(self, mapping):
        # mapping: [d0,d1,d2,d3,d4,d5], каждые 2 бита
        d1 = ((mapping[0] & 3) << 6) | ((mapping[1] & 3) << 4) | ((mapping[2] & 3) << 2) | (mapping[3] & 3)
        d2 = ((mapping[4] & 3) << 6) | ((mapping[5] & 3) << 4)
        self.write_register(REG["DIO_MAPPING_1"], d1)
        self.write_register(REG["DIO_MAPPING_2"], d2)

    # ---------------- FIFO ----------------
    def write_payload(self, data_bytes):
        # Установить FIFO на 0 (TX)
        self.write_register(REG["FIFO_TX_BASE_ADDR"], 0x00)
        self.write_register(REG["FIFO_ADDR_PTR"], 0x00)
        # burst write
        GPIO.output(self.nss, GPIO.LOW)
        self.spi.xfer2([0x80] + list(data_bytes))  # 0x80 -> write FIFO
        GPIO.output(self.nss, GPIO.HIGH)

    # ---------------- Helpers для MODEM ----------------
    def _set_bw_cr(self, bw_code, cr_code):
        cur = self.read_register(REG["MODEM_CONFIG1"])
        val = (bw_code << 4) | (cr_code << 1) | (cur & 0x01)
        self.write_register(REG["MODEM_CONFIG1"], val)

    def _set_sf_crc(self, sf, crc_on=True):
        cur = self.read_register(REG["MODEM_CONFIG2"])
        val = ((sf & 0x0F) << 4) | (0x04 if crc_on else 0x00) | (cur & 0x03)
        self.write_register(REG["MODEM_CONFIG2"], val)

    def _set_sync_word(self, sw):
        self.write_register(0x39, sw & 0xFF)

    # ---------------- IRQ helpers ----------------
    def _tx_done(self):
        # TxDone = бит 3 (0x08)
        return bool(self.get_irq_flags() & 0x08)

    def _rx_done(self):
        # RxDone = бит 6 (0x40)
        return bool(self.get_irq_flags() & 0x40)

    def _clear_all_irq(self):
        self.write_register(REG["IRQ_FLAGS"], 0xFF)

    def _burst_read_fifo(self, n):
        data = []
        for _ in range(n):
            data.append(self.read_register(0x00))
        return bytes(data)

    # ---------------- GPS ----------------
    def _ensure_gps(self):
        if self._gps is None:
            self._gps = serial.Serial(self._gps_port, self._gps_baud, timeout=1)

    def _have_valid_rmc(self, line: str) -> bool:
        return (line.startswith("$GPRMC") or line.startswith("$GNRMC")) and ",A," in line

    def _have_valid_gga(self, msg) -> bool:
        # pynmea2.GGA: fix_quality > 0 => есть фикс
        try:
            return int(getattr(msg, "fix_quality", 0)) > 0
        except Exception:
            return False

    def get_gps_fix(self, timeout=10.0, use_cache=True):
        """
        Возвращает (lat, lon, age_seconds) или None.
        Пытается читать RMC (status 'A') и GGA (fix_quality>0).
        """
        # открыть порт
        try:
            self._ensure_gps()
        except Exception as e:
            if self.verbose:
                print(f"[GPS] open error: {e}")
            return (self._last_fix[0], self._last_fix[1], time.time() - self._last_fix[2]) \
                   if (use_cache and self._last_fix) else None

        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = self._gps.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                if self._have_valid_rmc(line):
                    msg = pynmea2.parse(line)
                    lat, lon = msg.latitude, msg.longitude
                    ts = time.time()
                    self._last_fix = (lat, lon, ts)
                    return (lat, lon, 0.0)

                if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                    msg = pynmea2.parse(line)
                    if self._have_valid_gga(msg):
                        lat, lon = msg.latitude, msg.longitude
                        ts = time.time()
                        self._last_fix = (lat, lon, ts)
                        return (lat, lon, 0.0)
            except Exception:
                # игнорируем мусорные строки/парсинг
                pass

        # не успели — вернём кэш, если разрешено
        if use_cache and self._last_fix:
            lat, lon, ts = self._last_fix
            return (lat, lon, max(0.0, time.time() - ts))
        return None

    @staticmethod
    def _format_coords(lat, lon):
        s_lat = f"{lat:.6f}".replace(".", ",")
        s_lon = f"{lon:.6f}".replace(".", ",")
        return f"<{s_lat}:{s_lon}>"

    def build_payload(self, text, vessel_id=VESSEL_ID):
        fix = self.get_gps_fix(timeout=10, use_cache=True)
        if fix is None:
            coord_part = "<no-fix>"
        else:
            lat, lon, age = fix
            coord = self._format_coords(lat, lon)
            coord_part = coord if age < 600 else f"{coord} (AGE:{int(age)}s)"
        return f"<{vessel_id}> {coord_part} <{text}>"

    # ---------------- TX/RX/ACK ----------------
    def tx_bytes(self, payload: bytes):
        """Запись FIFO -> TX -> ожидание TxDone -> RXCONT."""
        self._clear_all_irq()
        try:
            self.set_payload_length(len(payload))
        except Exception:
            pass

        self.write_payload(list(payload))
        time.sleep(0.02)

        self.set_mode(MODE["TX"])
        t0 = time.time()
        while time.time() - t0 < 2.0:
            if self._tx_done():
                break
            time.sleep(0.005)

        self._clear_all_irq()
        self.reset_ptr_rx()
        self.set_mode(MODE["RXCONT"])

    def rx_poll_once(self):
        """Если пришёл пакет — вернуть bytes, иначе None."""
        if not self._rx_done():
            return None
        try:
            n   = self.read_register(REG["RX_NB_BYTES"])
            cur = self.read_register(REG["FIFO_RX_CURRENT_ADDR"])
            self.write_register(REG["FIFO_ADDR_PTR"], cur)
            data = self._burst_read_fifo(n)
            return data
        finally:
            self._clear_all_irq()
            self.reset_ptr_rx()
            self.set_mode(MODE["RXCONT"])

    def send_and_wait_ack(self, text_or_bytes, timeout=3.0) -> bool:
        """
        Отправить строку/байты и ждать ACK до timeout.
        ACK считается полученным, если в пакете есть 'ACK' (без регистра).
        """
        if isinstance(text_or_bytes, str):
            payload = text_or_bytes.encode("utf-8", "ignore")[:251]
        else:
            payload = bytes(text_or_bytes)[:251]

        self.tx_bytes(payload)

        t0 = time.time()
        while time.time() - t0 < timeout:
            pkt = self.rx_poll_once()
            if not pkt:
                time.sleep(0.02)
                continue
            try:
                text = pkt.decode("utf-8", "ignore")
            except Exception:
                text = ""
            if "ACK" in text.upper():
                if self.verbose:
                    print(f"[ACK] {text}")
                return True
        if self.verbose:
            print("[ACK] timeout")
        return False

    # ---------------- cleanup ----------------
    def close(self):
        try:
            self.spi.close()
        finally:
            try:
                BOARD.teardown()
            except Exception:
                pass
