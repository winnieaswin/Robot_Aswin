#!/usr/bin/env python3
#test update
import time
import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def send_motor(ser, pwm_r_pct, pwm_l_pct, dir_r, dir_l):
    pwm_r_pct = clamp(int(pwm_r_pct), 0, 100)
    pwm_l_pct = clamp(int(pwm_l_pct), 0, 100)
    dir_r = 1 if int(dir_r) else 0
    dir_l = 1 if int(dir_l) else 0

    msg = f"{pwm_r_pct},{pwm_l_pct},{dir_r},{dir_l}\n"
    ser.write(msg.encode("ascii"))
    ser.flush()  # ensure it goes out
    return msg

def main():
    # timeout=0.2 lets reads return; you can remove reads if you only send
    with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
        time.sleep(2.0)  # give ESP32 time to reset on serial open (common)

        print(f"Opened {PORT} @ {BAUD}")
        print("Sending: pwmR,pwmL,dirR,dirL (percent 0..100). Ctrl+C to stop.")

        # Example loop: ramp right motor, keep left fixed
        pwm_l = 40
        dir_r = 1
        dir_l = 0

        while True:
            for pwm_r in range(0, 101, 5):
                sent = send_motor(ser, pwm_r, pwm_l, dir_r, dir_l)
                print("TX:", sent.strip())

                # Optional: read response lines from ESP32 (if it prints something back)
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        print("RX:", line)
                except Exception:
                    pass

                time.sleep(0.1)

            for pwm_r in range(100, -1, -5):
                sent = send_motor(ser, pwm_r, pwm_l, dir_r, dir_l)
                print("TX:", sent.strip())
                time.sleep(0.1)

if __name__ == "__main__":
    main()
