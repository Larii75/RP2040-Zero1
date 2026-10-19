# RP2040-Zero (MicroPython): палітра + зум термокамери з пульта
# Приймач ELRS (CRSF, 420000 бод) -> GP5 (UART1 RX, лише слухаємо)
# Камера (UART 115200 8N1)         -> GP0 (TX) / GP1 (RX) (UART0)
# Земля спільна. Логіка 3.3 В.

import time
from machine import UART, Pin

# ================== НАЛАШТУВАННЯ ==================
CH_PALETTE = 6      # AUX-канал крутилки палітри (EdgeTX мікшер)
CH_ZOOM    = 7      # AUX-канал крутилки зуму

# Палітри: 15 значень, 0x00..0x0E
# Крутилка в лівому положенні = перша, в правому = остання
PALETTE_COUNT = 15  # усі 15 палітр

# Зум: 4 значення, позиції крутилки: лівий=X1, правий=X8
ZOOM_LEVELS = [0x00, 0x01, 0x02, 0x03]  # X1, X2, X4, X8

BOOT_DELAY_MS  = 5000  # чекати після старту, поки камера прокинеться
STABLE_FRAMES  = 5     # скільки кадрів позиція має триматись
MIN_GAP_MS     = 1000  # мін. пауза між командами палітри
MIN_GAP_ZOOM   = 1500  # мін. пауза між командами зуму (трохи більше)
# ==================================================

crsf = UART(1, baudrate=420000, rx=Pin(5), bits=8, parity=None, stop=1, rxbuf=512)
cam  = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)

SYNC    = (0xC8, 0xEE, 0xEA)
RC_FRAME = 0x16


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def channel(payload, n):
    bit = n * 11
    i   = bit >> 3
    val = payload[i] | (payload[i + 1] << 8)
    if i + 2 < len(payload):
        val |= payload[i + 2] << 16
    return (val >> (bit & 7)) & 0x7FF


buf = bytearray()


def parse_latest():
    global buf
    latest = None
    while len(buf) >= 4:
        if buf[0] not in SYNC or buf[1] < 2 or buf[1] > 62:
            del buf[0]
            continue
        total = buf[1] + 2
        if len(buf) < total:
            break
        if crc8(buf[2:total - 1]) != buf[total - 1]:
            del buf[0]
            continue
        if buf[2] == RC_FRAME and total == 26:
            p = channel(buf[3:25], CH_PALETTE - 1)
            z = channel(buf[3:25], CH_ZOOM - 1)
            latest = (p, z)
        del buf[:total]
    if len(buf) > 128:
        buf = buf[-64:]
    return latest


def palette_pos(v):
    # 172..1811 -> 0..14
    v = max(172, min(1811, v))
    return int((v - 172) / (1811 - 172) * (PALETTE_COUNT - 1) + 0.5)


def zoom_pos(v):
    # 172..1811 -> 0..3
    v = max(172, min(1811, v))
    if v < 582:
        return 0
    if v < 992:
        return 1
    if v < 1402:
        return 2
    return 3


def cam_frame(cls, sub, rw, data):
    body = bytes([0x36, cls, sub, rw]) + data
    return bytes([0xF0, len(data) + 4]) + body + bytes([sum(body) & 0xFF, 0xFF])


def cam_send(pkt):
    cam.write(pkt)
    time.sleep_ms(150)
    resp = cam.read()
    print("TX", pkt.hex(), "RX", resp.hex() if resp else None)


def set_palette(p):
    cam_send(cam_frame(0x78, 0x20, 0x00, bytes([p])))


def set_zoom(z):
    cam_send(cam_frame(0x70, 0x12, 0x00, bytes([ZOOM_LEVELS[z]])))


# --- старт ---
print("Старт. Палітра CH%d, Зум CH%d" % (CH_PALETTE, CH_ZOOM))
time.sleep_ms(1500)
cam_send(cam_frame(0x74, 0x02, 0x01, b"\x00"))  # читання моделі

t0           = time.ticks_ms()
last_palette = time.ticks_add(t0, -MIN_GAP_MS)
last_zoom    = time.ticks_add(t0, -MIN_GAP_ZOOM)

pal_applied  = None
zoom_applied = None

pal_cand  = None;  pal_cnt  = 0
zoom_cand = None;  zoom_cnt = 0

while True:
    data = crsf.read()
    if data:
        buf.extend(data)
        vals = parse_latest()
        if vals is not None:
            pv, zv = vals
            pp = palette_pos(pv)
            zp = zoom_pos(zv)
            now = time.ticks_ms()
            booted = time.ticks_diff(now, t0) > BOOT_DELAY_MS

            # Палітра
            if pp == pal_cand:
                pal_cnt += 1
            else:
                pal_cand = pp; pal_cnt = 1

            if (booted and pal_cnt >= STABLE_FRAMES
                    and pp != pal_applied
                    and time.ticks_diff(now, last_palette) > MIN_GAP_MS):
                print("Палітра %d -> 0x%02X" % (pp, pp))
                set_palette(pp)
                pal_applied = pp
                last_palette = now

            # Зум
            if zp == zoom_cand:
                zoom_cnt += 1
            else:
                zoom_cand = zp; zoom_cnt = 1

            if (booted and zoom_cnt >= STABLE_FRAMES
                    and zp != zoom_applied
                    and time.ticks_diff(now, last_zoom) > MIN_GAP_ZOOM):
                zoom_names = ["X1", "X2", "X4", "X8"]
                print("Зум %s -> 0x%02X" % (zoom_names[zp], ZOOM_LEVELS[zp]))
                set_zoom(zp)
                zoom_applied = zp
                last_zoom = now

    time.sleep_ms(2)