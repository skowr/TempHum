import machine
import bme280_float as bme280

i2c = machine.I2C(1, scl=machine.Pin(7), sda=machine.Pin(6), freq=400000)

def scani2c():

    print("Scanning I2C bus...")
    devices = i2c.scan()
    if not devices:
        print("No I2C devices found.")
    else:
        print("Found devices at addresses:", [hex(d) for d in devices])

def readbme280():

    print("Reading BME280 ...")
    bme = bme280.BME280(i2c=i2c)
    t, p, h = bme.read_compensated_data()
    print(f'Temp: {t}, Press: {p}, Humid: {h}')

#scani2c()
readbme280()

