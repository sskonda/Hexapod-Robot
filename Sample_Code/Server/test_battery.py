import os
import time
from adc import ADC

def main():
    adc = ADC()

    try:
        while True:
            voltage = adc.recvADC(2)   # battery channel
            os.system("clear")
            print("Hexapod Battery Monitor\n")
            print(f"Battery voltage: {voltage:.2f} V")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
