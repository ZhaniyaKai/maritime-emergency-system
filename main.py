import time
import json
import wave
import queue
import pathlib
import sounddevice as sd
from vosk import Model, KaldiRecognizer

import RPi.GPIO as GPIO

from LoRa import LoRa
from constants import MODE

# -------- Настройки --------
VESSEL_ID = "Vessel15774"

# Пины кнопок (BCM)
BTN_FIRE     = 5
BTN_MEDICAL  = 6
BTN_SHORT    = 13
BTN_VOICE    = 19

# Сообщения
MSG_FIRE    = "Fire alert!"
MSG_MEDICAL = "Medical help needed!"
MSG_SHORT   = "Electrical short circuit!"

# Аудио/распознавание
SAMPLE_RATE   = 16000
VOICE_SECONDS = 10
HERE          = pathlib.Path(__file__).parent.resolve()
MODEL_DIR     = (HERE / "vosk-model-small-en-us-0.15").resolve()
TMP_WAV       = HERE / "voice_tmp.wav"

# Очередь отправок
SEND_Q = queue.Queue()


# -------- Голос: запись и распознавание --------
def record_wav(path, seconds=VOICE_SECONDS, samplerate=SAMPLE_RATE, channels=1, device=None):
    print("[VOICE] Recording...")
    data = sd.rec(int(seconds * samplerate),
                  samplerate=samplerate,
                  channels=channels,
                  dtype='int16',
                  blocking=True,
                  device=device)
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)
        wf.setframerate(samplerate)
        wf.writeframes(data.tobytes())
    print("[VOICE] Done")


def transcribe_wav(path):
    print("[VOICE] Recognizing...")
    model = Model(str(MODEL_DIR))
    rec = KaldiRecognizer(model, SAMPLE_RATE)
    rec.SetWords(True)
    text = ""
    with wave.open(str(path), "rb") as wf:
        while True:
            chunk = wf.readframes(4000)
            if not chunk:
                break
            if rec.AcceptWaveform(chunk):
                j = json.loads(rec.Result())
                text += " " + j.get("text", "")
        j = json.loads(rec.FinalResult())
        text += " " + j.get("text", "")
    text = text.strip()
    print(f"[VOICE] Text: {text or '(empty)'}")
    return text or "voice empty"


# -------- Кнопки --------
def setup_buttons():
    GPIO.setmode(GPIO.BCM)
    for pin in (BTN_FIRE, BTN_MEDICAL, BTN_SHORT, BTN_VOICE):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def cb_factory(msg):
        def _cb(ch):
            time.sleep(0.03)  # антидребезг
            if GPIO.input(ch) == GPIO.LOW:
                SEND_Q.put(msg)
        return _cb

    GPIO.add_event_detect(BTN_FIRE,    GPIO.FALLING, callback=cb_factory(MSG_FIRE),    bouncetime=200)
    GPIO.add_event_detect(BTN_MEDICAL, GPIO.FALLING, callback=cb_factory(MSG_MEDICAL), bouncetime=200)
    GPIO.add_event_detect(BTN_SHORT,   GPIO.FALLING, callback=cb_factory(MSG_SHORT),   bouncetime=200)

    def voice_cb(ch):
        time.sleep(0.03)
        if GPIO.input(ch) == GPIO.LOW:
            try:
                record_wav(TMP_WAV, seconds=VOICE_SECONDS)
                txt = transcribe_wav(TMP_WAV)
                SEND_Q.put("VOICE: " + txt)
            except Exception as e:
                print(f"[VOICE] error: {e}")
                SEND_Q.put("VOICE: error")

    GPIO.add_event_detect(BTN_VOICE, GPIO.FALLING, callback=voice_cb, bouncetime=400)


# -------- Главная логика --------
def main():
    setup_buttons()

    lora = LoRa(verbose=True)
    lora.set_mode(MODE["STDBY"])
    lora.reset_ptr_rx()
    lora.set_mode(MODE["RXCONT"])
    print("[RUN] Ready. Buttons: FIRE, MEDICAL, SHORT, VOICE (10s record).")

    try:
        while True:
            try:
                msg = SEND_Q.get(timeout=0.2)
            except queue.Empty:
                continue

            payload = lora.build_payload(msg, vessel_id=VESSEL_ID)
            print(f"[TX] {payload}")

            ok = lora.send_and_wait_ack(payload, timeout=3.0)
            if ok:
                print("[TX] Delivered (ACK).")
            else:
                print("[TX] Sent, but no ACK.")
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C")
    finally:
        GPIO.cleanup()
        try:
            lora.set_mode(MODE["SLEEP"])
            lora.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
