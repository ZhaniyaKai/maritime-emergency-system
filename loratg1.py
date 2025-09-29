import argparse
import sys
import requests
import serial

# --- Telegram ---
BOT_TOKEN = "8415029726:AAERG4XuHxpcHztMvQNoFEiUoMdarTeyGLo"
CHAT_ID   = "-1002614097936"   # ваш канал (бот должен быть админом)

def send_to_telegram(text: str) -> None:
    url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"
    try:
        r = requests.post(url, data={"chat_id": CHAT_ID, "text": text}, timeout=10)
        if r.status_code != 200:
            print(f"[TG] HTTP {r.status_code}: {r.text}")
        else:
            print("[TG] Отправлено.")
    except Exception as e:
        print(f"[TG] Ошибка запроса: {e}")

# --- Serial ---
def open_serial(port: str, baud: int) -> serial.Serial:
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=1)
        print(f"[SER] Открыт порт {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"[SER] Ошибка открытия порта {port}: {e}")
        sys.exit(1)

def process_line(line: str, forward_raw: bool) -> None:
    """
    Если forward_raw=True — шлём любые строки в TG.
    Если False — шлём только полезные строки после префикса 'MSG:'.
    НИЧЕГО НЕ ФИЛЬТРУЕМ ПО GPS: '<no-fix>' допустим.
    """
    if not line:
        return

    print(f"[RX] {line}")

    if forward_raw:
        send_to_telegram(line)
        return

    if line.startswith("MSG:"):
        payload = line[4:].strip()
        if payload:
            send_to_telegram(payload)

def main():
    ap = argparse.ArgumentParser(description="Forward LoRa/Arduino messages to Telegram")
    ap.add_argument("--port", default="COM3", help="Serial port, напр. COM3")
    ap.add_argument("--baud", type=int, default=9600, help="Baudrate, напр. 9600")
    ap.add_argument("--raw", action="store_true", help="Пересылать любые строки, не только 'MSG:'")
    args = ap.parse_args()

    ser = open_serial(args.port, args.baud)
    print("[RUN] Готово. Слушаю Arduino…  (Ctrl+C для выхода)")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            process_line(line, forward_raw=args.raw)
    except KeyboardInterrupt:
        print("\n[EXIT] Выход по Ctrl+C")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
