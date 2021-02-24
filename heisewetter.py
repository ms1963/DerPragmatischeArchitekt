# Dies ist ein Demoprogramm, das zwei am I2C-Bus 0 des Pico angeschlossene
# Geräte zeigt: das OLED-Display SSD1306 und den Umweltsensor BME280.
# Feuchtigkeit, Druck, Temperatur werden periodisch gemessen und am OLED
# angezeigt


from machine import Pin, I2C    # Wir brauchen Pin und I2C des Pico
from micropython import const
from ustruct import unpack as unp
from ssd1306 import SSD1306_I2C  # Modul für SSD1306
import utime # Zwei Zeit-Bibliotheken 
import time

# Condition -----------------------
COND_RED = 1     # Schlechtes Klima
COND_GREEN = 2   # Angenehmes Klima
COND_YELLOW = 3  # Mittleres  Klima
#----------------------------------

# Hier ist meine Regel, wann das Wetter gut, mittel, schlecht ist
# Das lässt sich an eigene Bedürfnisse anpassen
def condition(temperature, humidity, pressure):
    niceTemperature = temperature >= ComfortZoneTemp[0] and temperature <= ComfortZoneTemp[1]
    niceHumidity = humidity >= ComfortZoneHumi[0] and humidity <= ComfortZoneHumi[1]
    if niceHumidity and niceTemperature:
        return COND_GREEN
    elif (niceHumidity != niceTemperature):
        return COND_YELLOW
    else:
        return COND_RED

# DEBUG -----------------------------------------------------------------------
debug = False # Toggle: True => Print-Ausgaben am Terminal, False => dann nicht
#------------------------------------------------------------------------------

# I2C --------------------------------------------------------------
sda = Pin(4) # BME280 und SSD1306 sind an GPIO 4 und 5 angeschlossen
scl = Pin(5)
i2c = I2C(0,sda=sda,scl=scl,freq=400000) # I2C-Bus 0
i2c_addr_bme = 0x76  # Ich gehe davon aus, der BME280 liegt an 0x76
#-------------------------------------------------------------------

# LEDs -------------------------------------------------------------
GreenLED  = Pin(21, Pin.OUT) # Grüne LED an GPIO 21
YellowLED = Pin(20, Pin.OUT) # Gelbe LED an GPIO 20
RedLED    = Pin(19, Pin.OUT) # Rote  LED an GPIO 19
#-------------------------------------------------------------------

# BME280 ----------------------------------------------------------------------------------
ComfortZoneTemp = (15,25)  # Meine Komfortzone für Temperatur liegt zwischen 15 und 25 Grad
ComfortZoneHumi = (10,40)  # Meine Komfortzone für Feuchtigkeit liegt zwischen 10 und 40%
#------------------------------------------------------------------------------------------

# Cycle Time ----------------------------------------------
SLEEPTIME = 5 # Zeit zwischen zwei Messungen mit dem BME280
#----------------------------------------------------------



# Bosch Sensortec hat folgende Daten bereitgestellt:
# https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf


def readBMX280():
    # Feuchtigkeits Oversampling
    i2c.writeto_mem(i2c_addr_bme, 0xf2, b'\x03')  # ctrl_hum 00000 011
    #
    # Temperatur Oversampling / Druck / Oversampling / Sensor Modus
    i2c.writeto_mem(i2c_addr_bme, 0xf4, b'\x6F')  # ctrl_meas 011 011 11
    #
    # kurze Wartezeit, um das Setup abzuschließen
    time.sleep(0.1)

    #
    # Sensor ist am I2C-Bus ein Gerät mit Memory Mapping
    #
    reg_base_addr = 0x88  # Memory Map von 0x88 bis 0xff
    block_size = 0x100 - reg_base_addr  # Zahl der zu lesenden Bytes
    r = i2c.readfrom_mem(i2c_addr_bme, reg_base_addr, block_size)  # Einlesen in Buffer

    # Die zwei als unsigned int gelesenen Bytes
    def read_const_u(a):
        return r[a - reg_base_addr] + (r[a - reg_base_addr + 1] << 8)


    # Zwei Bytes werden als signed int gelesen
    def read_const_s(a):
        v = r[a - reg_base_addr] + (r[a - reg_base_addr + 1] << 8)
        if v > 32767:
            v = v - 65536
        return v


    # TEMPERATURMESSUNG

    # Rohdaten für Temperatur
    temp_msb  = r[0xfa - reg_base_addr]
    temp_lsb  = r[0xfb - reg_base_addr]
    temp_xlsb = r[0xfc - reg_base_addr]
    adc_t = (temp_msb << 12) + (temp_lsb << 4) + (temp_xlsb >> 4)

    # Compensation Parameter (Sind Konstanten und werden normal nur einmal gelesen)
    dig_t1 = read_const_u(0x88)
    dig_t2 = read_const_s(0x8a)
    dig_t3 = read_const_s(0x8c)

    # Temperatur berechnen
    # folgt dem Bosch Sensortec Datasheet
    t_fine =  (((adc_t >> 3) - (dig_t1 << 1)) * (dig_t2 >> 11)) + ((((((adc_t >> 4) - dig_t1) * ((adc_t >> 4) - dig_t1)) >> 12) * dig_t3) >> 14)

    t = (t_fine * 5 + 128) >> 8
    _temperature = t / 100 # Temperatur merken


    # Druck

    # Rohdaten akquirieren
    press_msb  = r[0xf7 - reg_base_addr]
    press_lsb  = r[0xf8 - reg_base_addr]
    press_xlsb = r[0xf9 - reg_base_addr]
    adc_p = (press_msb << 12) + (press_lsb << 4) + (press_xlsb >> 4)

    # Compensation Parameter (Sind Konstanten und werden normal nur einmal gelesen)
    prs1 = read_const_u(0x8e)
    prs2 = read_const_s(0x90)
    prs3 = read_const_s(0x92)
    prs4 = read_const_s(0x94)
    prs5 = read_const_s(0x96)
    prs6 = read_const_s(0x98)
    prs7 = read_const_s(0x9a)
    prs8 = read_const_s(0x9c)
    prs9 = read_const_s(0x9e)

    # Druck brechnen
    # Folgt Bosch Sensortec Datasheet
    t1 = t_fine - 128000
    t2 = t1 * t1 * prs6
    t2 = t2 + ((t1 * prs5) << 17)
    t2 = t2 + (prs4 << 35)
    t1 = ((t1 * t1 * prs3) >> 8) + ((t1 * prs2) << 12)
    t1 = ((1 << 47) + t1) * prs1 >> 33
    if t1 == 0:  # Division durch 0
        p = 0
    else:
        p = 1048576 - adc_p
        p = int((((p << 31) - t2) * 3125) / t1)
        t1 = (prs9 * (p >> 13) * (p >> 13)) >> 25
        t2 = (prs8 * p) >> 19
        p = ((p + t1 + t2) >> 8) + (prs7 << 4)
    
    _pressure = p / 25600

    # Feuchtigkeit messen

    # Rohdatenakquise
    hum_msb = r[0xfd - reg_base_addr]
    hum_lsb = r[0xfe - reg_base_addr]
    adc_h = (hum_msb << 8) + hum_lsb

    # Compensation Parameter (Sind Konstanten und werden normal nur einmal gelesen)
    hum1 = r[0xa1 - reg_base_addr]
    hum2 = read_const_s(0xe1)
    hum3 = r[0xe3 - reg_base_addr]
    hum4 = (r[0xe4 - reg_base_addr] << 4) + (r[0xe5 - reg_base_addr] & 0x0f)
    hum5 = (r[0xe6 - reg_base_addr] << 4) + ((r[0xe5 - reg_base_addr] & 0x00f0) >> 4)
    hum6 = r[0xe7 - reg_base_addr]

    # Hier wird die Feuchtigkeit berechnet
    # Folgt Bosch Sensortec Datasheet
    v_x1_u32r = t_fine - 76800
    v_x1_u32r = (((((adc_h << 14) - (hum4 << 20) - (hum5 * v_x1_u32r)) + 16384) >> 15) * (((((((v_x1_u32r * ( hum6)) >> 10) * (((v_x1_u32r * hum3) >> 11) + 32768)) >> 10) + 2097152) * hum2 + 8192) >> 14))
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * hum1) >> 4))

    # Limits prüfen
    if v_x1_u32r < 0:
        v_x1_u32r = 0

    if v_x1_u32r > 0x19000000:
        v_x1_u32r = 0x19000000

    h = v_x1_u32r >> 12

    _humidity = h / 1024

    # Alle Messdaten zurückliefern
    return _temperature, _humidity, _pressure
    


if debug:
    print('Ich habe an folgenden Adressen Komponenten am I2C-Bus entdeckt:')
    devices = i2c.scan()
    if devices:
        for i in devices:
            print(hex(i))

    print()
    utime.sleep_ms(2000)


oled = SSD1306_I2C(128,64,i2c)

if debug:
    print("Starte OLED Ausgabe in wenigen Sekunden")
    utime.sleep_ms(2000)
    print("Let's go")

oled.fill(0)
oled.show()

while True:
    temperature, humidity, pressure = readBMX280()
    if debug:
        print("Temperatur ......" + str(temperature) + " Grad")
        print("Feuchtigkeit ...." + str(humidity) + " %")
        print("Druck ..........." + str(pressure) + " HPa")
   
  
  
    currentState = condition(temperature, humidity, pressure) 
    if currentState == COND_GREEN:
        GreenLED.value(1)
        YellowLED.value(0)
        RedLED.value(0)
        if debug:
            print("Angenehmes Klima")
    elif currentState == COND_YELLOW:
        GreenLED.value(0)
        YellowLED.value(1)
        RedLED.value(0)
        if debug:
            print("Geht so")
    elif currentState == COND_RED:
        GreenLED.value(0)
        YellowLED.value(0)
        RedLED.value(1)
        if debug:
            print("Unangenehmes Klima")        


    if debug:
        print()
        print()

    utime.sleep(1)
    oled.fill(0)
    oled.text("Heise Wetter", 5, 10)
    # Formatierte Ausgabe mit 7 Ziffern bei 2 Nachkommastellen
    oled.text(str('% 7.2f' % temperature) + " Grad", 5,20)
    oled.text(str('% 7.2f' % humidity) + " %",5,30)
    oled.text(str('% 7.2f' % pressure) + " HPa",5,40)
    # Und jetzt enthuellen
    oled.show()

    utime.sleep(SLEEPTIME) # Schlafen bis zur nächsten Messung
