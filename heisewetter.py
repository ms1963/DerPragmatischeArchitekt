# Dies ist ein Demoprogramm, das zwei am I2C-Bus 0 des Pico angeschlossene
# Geräte zeigt: das OLED-Display SSD1306 und den Umweltsensor BME280.
# Feuchtigkeit, Druck, Temperatur werden periodisch gemessen und am OLED
# angezeigt


from machine import Pin, I2C, SoftI2C    # Wir brauchen Pin und I2C des Pico
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
debug = True # Toggle: True => Print-Ausgaben am Terminal, False => dann nicht
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


class BMX280:
    # Im Konstruktor werden primäre Datenmember und Konstanten belegt
    def __init__(self, i2c, i2c_addr_bme):
        # Adresse des BME280 am I2C-Bus
        self.i2c_addr_bme = i2c_addr_bme
        # I2C-Objekt zum Zugriff auf den I2C
        self.i2c = i2c
        # Sensor ist am I2C-Bus ein Gerät mit Memory Mapping
        self.reg_base_addr = 0x88  # Memory Map von 0x88 bis 0xff
        self.block_size = 0x100 - self.reg_base_addr  # Zahl der zu lesenden Bytes
        self.r = self.i2c.readfrom_mem(self. i2c_addr_bme, self.reg_base_addr, self.block_size)  # Einlesen in Buffer
        # Compensation Parameter (Sind Konstanten und werden normal nur einmal gelesen)
        # für Druck:
        self.prs1 = self.read_const_u(0x8e)
        self.prs2 = self.read_const_s(0x90)
        self.prs3 = self.read_const_s(0x92)
        self.prs4 = self.read_const_s(0x94)
        self.prs5 = self.read_const_s(0x96)
        self.prs6 = self.read_const_s(0x98)
        self.prs7 = self.read_const_s(0x9a)
        self.prs8 = self.read_const_s(0x9c)
        self.prs9 = self.read_const_s(0x9e)
        # für Feuchtigkeit:
        self.hum1 = self.r[0xa1 - self.reg_base_addr]
        self.hum2 = self.read_const_s(0xe1)
        self.hum3 = self.r[0xe3 - self.reg_base_addr]
        self.hum4 = (self.r[0xe4 - self.reg_base_addr] << 4) + (self.r[0xe5 - self.reg_base_addr] & 0x0f)
        self.hum5 = (self.r[0xe6 - self.reg_base_addr] << 4) + ((self.r[0xe5 - self.reg_base_addr] & 0x00f0) >> 4)
        self.hum6 = self.r[0xe7 - self.reg_base_addr]
        # für Temperatur:
        self.tmp1 = self.read_const_u(0x88)
        self.tmp2 = self.read_const_s(0x8a)
        self.tmp3 = self.read_const_s(0x8c)    


    def measure(self):
        # Feuchtigkeits Oversampling
        self.i2c.writeto_mem(self.i2c_addr_bme, 0xf2, b'\x03')  # ctrl_hum 00000 011
        
        # Temperatur Oversampling / Druck / Oversampling / Sensor Modus
        self.i2c.writeto_mem(self.i2c_addr_bme, 0xf4, b'\x6F')  # ctrl_meas 011 011 11
        
        # kurze Wartezeit, um das Setup abzuschließen
        time.sleep(0.1)

        # Sensor ist am I2C-Bus ein Gerät mit Memory Mapping
        self.reg_base_addr = 0x88  # Memory Map von 0x88 bis 0xff
        self.block_size = 0x100 - self.reg_base_addr  # Zahl der zu lesenden Bytes
        self.r = self.i2c.readfrom_mem(self. i2c_addr_bme, self.reg_base_addr, self.block_size)  # Einlesen in Buffer

        # Alle Messdaten erfassen:
        self.calcTemperature()
        self.calcHumidity()
        self.calcPressure()

        return self._temperature, self._humidity, self._pressure


    # Die zwei als unsigned int gelesenen Bytes
    def read_const_u(self, a):
        return self.r[a - self.reg_base_addr] + (self.r[a - self.reg_base_addr + 1] << 8)


    # Zwei Bytes werden als signed int gelesen
    def read_const_s(self, a):
        v = self.r[a - self.reg_base_addr] + (self.r[a - self.reg_base_addr + 1] << 8)
        if v > 32767:
            v = v - 65536
        return v

    # Temperaturwert ermitteln
    def calcTemperature(self):
        # TEMPERATURMESSUNG

        # Rohdaten für Temperatur
        temp_msb  = self.r[0xfa - self.reg_base_addr]
        temp_lsb  = self.r[0xfb - self.reg_base_addr]
        temp_xlsb = self.r[0xfc - self.reg_base_addr]
        self.adc_t = (temp_msb << 12) + (temp_lsb << 4) + (temp_xlsb >> 4)

        # Temperatur berechnen
        # folgt dem Bosch Sensortec Datasheet
        self.t_fine =  (((self.adc_t >> 3) - (self.tmp1 << 1)) * (self.tmp2 >> 11)) + ((((((self.adc_t >> 4) - self.tmp1) * ((self.adc_t >> 4) - self.tmp1)) >> 12) * self.tmp3) >> 14)
        t = (self.t_fine * 5 + 128) >> 8
        self._temperature = t / 100 # Temperatur merken
            
    # Druck ermitteln
    def calcPressure(self):
        # Rohdaten akquirieren
        self.press_msb  = self.r[0xf7 - self.reg_base_addr]
        self.press_lsb  = self.r[0xf8 - self.reg_base_addr]
        self.press_xlsb = self.r[0xf9 - self.reg_base_addr]
        self.adc_p = (self.press_msb << 12) + (self.press_lsb << 4) + (self.press_xlsb >> 4)   

        # Druck brechnen
        # Folgt Bosch Sensortec Datasheet
        t1 = self.t_fine - 128000
        t2 = t1 * t1 * self.prs6
        t2 = t2 + ((t1 * self.prs5) << 17)
        t2 = t2 + (self.prs4 << 35)
        t1 = ((t1 * t1 * self.prs3) >> 8) + ((t1 * self.prs2) << 12)
        t1 = ((1 << 47) + t1) * self.prs1 >> 33
        if t1 == 0:  # Division durch 0
            p = 0
        else:
            p = 1048576 - self.adc_p
            p = int((((p << 31) - t2) * 3125) / t1)
            t1 = (self.prs9 * (p >> 13) * (p >> 13)) >> 25
            t2 = (self.prs8 * p) >> 19
            p = ((p + t1 + t2) >> 8) + (self.prs7 << 4)
        
            self._pressure = p / 25600

    # Feuchtigkeit ermitteln
    def calcHumidity(self):
        # Feuchtigkeit messen

        # Rohdatenakquise
        hum_msb = self.r[0xfd - self.reg_base_addr]
        hum_lsb = self.r[0xfe - self.reg_base_addr]
        self.adc_h = (hum_msb << 8) + hum_lsb

        # Hier wird die Feuchtigkeit berechnet
        # Folgt Bosch Sensortec Datasheet
        v_x1_u32r = self.t_fine - 76800
        v_x1_u32r = (((((self.adc_h << 14) - (self.hum4 << 20) - (self.hum5 * v_x1_u32r)) + 16384) >> 15) * (((((((v_x1_u32r * (self.hum6)) >> 10) * (((v_x1_u32r * self.hum3) >> 11) + 32768)) >> 10) + 2097152) * self.hum2 + 8192) >> 14))
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * self.hum1) >> 4))

        # Limits prüfen
        if v_x1_u32r < 0:
            v_x1_u32r = 0

        if v_x1_u32r > 0x19000000:
            v_x1_u32r = 0x19000000

        h = v_x1_u32r >> 12

        self._humidity = h / 1024
        


#######################
## Das Hauptprogramm ##
#######################

if debug:
    print('Ich habe an folgenden Adressen Komponenten am I2C-Bus entdeckt:')
    devices = i2c.scan()
    if devices:
        for i in devices:
            print(hex(i))
            utime.sleep_ms(2000)


oled = SSD1306_I2C(128,64,i2c)

bme =BMX280(i2c = i2c, i2c_addr_bme = i2c_addr_bme)

if debug:
    print("Starte OLED Ausgabe in wenigen Sekunden")
    utime.sleep_ms(2000)
    print("Let's go")

oled.fill(0)
oled.show()

while True:
    temperature, humidity, pressure = bme.measure()
    if debug:
        print("Temperatur ......" + str('% 7.2f' % temperature) + " Grad")
        print("Feuchtigkeit ...." + str('% 7.2f' % humidity) + " %")
        print("Druck ..........." + str('% 7.2f' % pressure) + " HPa")
        
        
        
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
                
    oled.fill(0)
    oled.text("Heise Wetter", 5, 10)
    # Formatierte Ausgabe mit 7 Ziffern bei 2 Nachkommastellen
    oled.text(str('% 7.2f' % temperature) + " Grad", 5,20)
    oled.text(str('% 7.2f' % humidity) + " %",5,30)
    oled.text(str('% 7.2f' % pressure) + " HPa",5,40)
    # Und jetzt enthuellen
    oled.show()

    utime.sleep(SLEEPTIME) # Schlafen bis zur nächsten Messung
