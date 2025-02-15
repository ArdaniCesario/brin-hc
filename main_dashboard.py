#Banana Preservation Box Research
#National Research and Innovation Agency
#20230404

Builder.load_file('main_dashboard.kv')

class SensorNamePlate(RelativeLayout):
    def __init__(self, **kwargs):
        super(SensorNamePlate, self).__init__(**kwargs)

class DigitalIndicator(ButtonBehavior, RelativeLayout):
    def __init__(self, **kwargs):
        super(DigitalIndicator, self).__init__(**kwargs)

# HW
# Camera Setup
camera = picamera.PiCamera()
# ~ camera.color_effects = (128,128) #grayscale
# ~ camera.resolution = (800,600)
shutterspeed = 22000
iso = 200
saturation = 20
camera.shutter_speed = (shutterspeed) # 1ms
camera.iso = (iso)
camera.saturation = (saturation)
# ~ camera.exposure_mode = ('beach')

# Adafruit DHT Setup
dhtDevice = adafruit_dht.DHT22(board.D18, False)

# DFRobot O2 Setup
COLLECT_NUMBER   = 10           # collect number, the collection range is 1-100
IIC_MODE         = 0x01         # default use IIC1
oxygen = DFRobot_Oxygen_IIC(IIC_MODE ,ADDRESS_3)

# ADC MCP3002 Setup for DFRobot CO2
vref = 5 * 1000 #Vref = VDD for the MCP3002
resolution = 2**10 # for 10 bits of resolution
calibration = 800 # in mV, to make up for the precision of the components
# SPI setup
spi_max_speed = 1000000 # 1 MHz (1.2MHz = max for 2V7 ref/supply)
# reason is that the ADC input cap needs time to get charged to the input level.
CE = 0 # Selection of the SPI device
spi = spidev.SpiDev()
spi.open(0,CE) # Open up the communication to the device
spi.max_speed_hz = spi_max_speed

# DFRobot CO2 Setup
DC_gain = 8.5
ppm_range = 600
vdrop = 0.04
vzero = 2300/1000
norm_ppm = 400

# Sparkfun MUX Setup
test = qwiic.QwiicTCA9548A(address = 0x71)
test.disable_all()

# STC31 CO2 Setup
test.enable_channels([1])
transceiver = LinuxI2cTransceiver('/dev/i2c-1')
sht3x = Shtc3I2cDevice(I2cConnection(transceiver), slave_address=0x70)
i2c_sensor = sensirion_i2c_stc.stc3x.device.Stc3xI2cDevice(I2cConnection(transceiver), slave_address=41)
i2c_sensor.set_bianry_gas(Stc31BinaryGas.Co2InAirRange25)
temp_stc, hum_stc = sht3x.measure()
gas_concentration, temperature = i2c_sensor .measure_gas_concentration()

# QWIIC Scale Setup
test.enable_channels([3])
# Create the bus
bus = smbus2.SMBus(1)
scale = PyNAU7802.NAU7802()
if scale.begin(bus):
    pass
# Create the scale and initialize it
scale.setZeroOffset(-98646.0)
scale.setCalibrationFactor(-426839.2857142857)

# Fan Setup
# Pin BCM mode
fan01 = 12
fan02 = 13
GPIO.setup(fan01,GPIO.OUT)
GPIO.setup(fan02,GPIO.OUT)
pwm01 = GPIO.PWM(fan01,21000)
pwm01.start(0)
pwm02 = GPIO.PWM(fan02,21000)
pwm02.start(0)

# LED Setup
led=16
GPIO.setup(led,GPIO.OUT)
#LOW Level Trigger Relay (True=OFF, False=ON)
GPIO.output(led,False)

# Refrigerator Setup
refri=27
GPIO.setup(refri,GPIO.OUT)
GPIO.output(refri,True)

# Logging Setup
now = datetime.now()
log_name = now.strftime("%Y-%m-%d_%H-%M-%S")
title = ['Banana Preservation Box Research']
dtlog = ['Date and Time Start: ', log_name]
camlog = ['Camera Shutterspeed:', shutterspeed, 'Camera ISO: ', iso, 'Camera Saturation: ', saturation]
bufferlog = [' ']
header = ['Date&Time','Temperature (degC)','Humidity (%)','O2 Concentration (ppm)','CO2 Concentration (ppm)','Fan PWM Inlet (%)', 'Fan PWM Outlet (%)', 'LED', 'Refrigerator']
with open('/home/bananarpi/Desktop/Data_Ujicoba/Log/log_'+log_name+'.csv', 'w+', encoding='UTF8', newline='') as log:
    writedata = csv.writer(log)
    #Write the header
    writedata.writerow(title)
    writedata.writerow(dtlog)
    writedata.writerow(camlog)
    writedata.writerow(bufferlog)
    writedata.writerow(header)
# End HW

class Dashboard(FloatLayout):
    def __init__(self, **kwargs):
        super(Dashboard, self).__init__(**kwargs)

        self.m__sensor_o2.m__sensor_title.text = "Oxygen"
        self.m__sensor_o2.m__sensor_eu.text = "%"

        self.m__sensor_co2.m__sensor_title.text = "Carbon Dioxide"
        self.m__sensor_co2.m__sensor_eu.text = "%"
        
        self.m__sensor_n.m__sensor_title.text = "Nitrogen"
        self.m__sensor_n.m__sensor_eu.text = "%"

        self.m__sensor_c2h4.m__sensor_title.text = "Ethylene"
        self.m__sensor_c2h4.m__sensor_eu.text = "ppm"
        
        self.m__sensor_m.m__sensor_title.text = "Mass"
        self.m__sensor_m.m__sensor_eu.text = "Kilogram"
        
        self.m__sensor_temp_0.m__sensor_title.text = "Temp Main"
        self.m__sensor_temp_0.m__sensor_eu.text = "deg C"
        self.m__sensor_humidity_0.m__sensor_title.text = "Hum Main"
        self.m__sensor_humidity_0.m__sensor_eu.text = "%"
        
        self.m__sensor_temp_4.m__sensor_title.text = "Temp Inlet"
        self.m__sensor_temp_4.m__sensor_eu.text = "deg C"
        self.m__sensor_humidity_4.m__sensor_title.text = "Hum Inlet"
        self.m__sensor_humidity_4.m__sensor_eu.text = "%"
        
        self.m__sensor_temp_5.m__sensor_title.text = "Temp Outlet"
        self.m__sensor_temp_5.m__sensor_eu.text = "deg C"
        self.m__sensor_humidity_5.m__sensor_title.text = "Hum Outlet"
        self.m__sensor_humidity_5.m__sensor_eu.text = "%"
        
        self.m__sensor_temp_6.m__sensor_title.text = "Temp Light"
        self.m__sensor_temp_6.m__sensor_eu.text = "deg C"
        self.m__sensor_humidity_6.m__sensor_title.text = "Hum Light"
        self.m__sensor_humidity_6.m__sensor_eu.text = "%"
        
        self.m__sensor_temp_7.m__sensor_title.text = "Temp ATM"
        self.m__sensor_temp_7.m__sensor_eu.text = "deg C"
        self.m__sensor_humidity_7.m__sensor_title.text = "Hum ATM"
        self.m__sensor_humidity_7.m__sensor_eu.text = "%"
        
        self.m__cooling.m__title.text = "Cool"
        self.m__led.m__title.text = "LED"
        self.m__indicator1.m__title.text = "Log"
        self.m__indicator2.m__title.text = "Image"
        self.m__indicator3.m__title.text = "Shot"
        self.m__indicator4.m__title.text = "Start"
        self.m__indicator5.m__title.text = "Stop"
        self.m__exit.m__title.text = "Exit"
        
        self.pwminlet = 0
        self.pwmoutlet = 0

        self.temperature = 0.0
        self.humidity = 0.0
        
        self.ledlogic = False
        self.refriLogic = True
        
        Clock.schedule_once(self._finish_init)
        Clock.schedule_interval(self._update_clock_value, 2)
        
        self.last_data = 61
        self.last_camera = 25

    def _finish_init(self, dt):
        print('Starting main...')

    def _update_clock_value(self, dt):
        curDT = datetime.now()
        self.m__txt_date.text = curDT.strftime("%d %B %Y")
        self.m__txt_time.text = curDT.strftime("%H:%M:%S")
        dt_name = curDT.strftime("%Y-%m-%d_%H-%M-%S")
        dt_triggerh = curDT.hour
        dt_triggerm = curDT.minute
        
        # ~ # Read T and Rh Data
        # ~ try:
            # ~ self.temperature = dhtDevice.temperature
            # ~ self.humidity = dhtDevice.humidity
        # ~ except RuntimeError as error:
            # ~ # Errors happen fairly often, DHT's are hard to read, just keep going
            # ~ pass
        
        # ~ try:
            # ~ self.m__sensor_humidity.m__sensor_value.text = str('{}'.format(self.humidity))
            # ~ self.m__sensor_temp.m__sensor_value.text = str('{}'.format(self.temperature))
        # ~ except RuntimeError as error:
            # ~ pass
        
        #Read O2 Data
        o2_vol = oxygen.get_oxygen_data(COLLECT_NUMBER)
        o2_ppm = o2_vol*10000
        self.m__sensor_o2.m__sensor_value.text = str('{:.1f}'.format(o2_vol))
        
        #Read ADC Voltage for CO2 Data
        channeldata = self.read_mcp3002(0)
        #Voltage = (CHX data * (V-ref [= 5000 mV] * 2 [= 1:2 input divider]) / 1024 [= 10bit resolution]
        voltage = int(round(((channeldata * vref) / resolution),0))+calibration
        
        #Conversion to CO2 from ADC Voltage
        co2_ppm = (ppm_range*((vzero/DC_gain)-((voltage/1000)/DC_gain))/vdrop)+norm_ppm
        co2_vol = co2_ppm/10000
        self.m__sensor_co2.m__sensor_value.text = str('{:.2f}'.format(co2_vol))
                
        # Channel 1 - STC31
        # ~ print("bacaan shtc3= {}, {}".format(temp_stc, hum_stc))
        # ~ print("bacaan stc31= {}, {}".format(gas_concentration, temperature))
        # ~ self.m__sensor_co2.m__sensor_value.text = str('{:.2f}'.format(gas_concentration))
        
        # Channel 0 - AHT20
        test.enable_channels([0])
        sensor0 = AHT20.AHT20()
        self.m__sensor_humidity_0.m__sensor_value.text = str('{:.1f}'.format(sensor0.get_humidity_crc8()))
        self.m__sensor_temp_0.m__sensor_value.text = str('{:.1f}'.format(sensor0.get_temperature_crc8()))
        test.disable_channels([0])
        
        # Channel 4 - AHT20
        test.enable_channels([4])
        sensor4 = AHT20.AHT20()
        self.m__sensor_humidity_4.m__sensor_value.text = str('{:.1f}'.format(sensor4.get_humidity_crc8()))
        self.m__sensor_temp_4.m__sensor_value.text = str('{:.1f}'.format(sensor4.get_temperature_crc8()))
        test.disable_channels([4])
        
        # Channel 5 - AHT20
        test.enable_channels([5])
        sensor5 = AHT20.AHT20()
        self.m__sensor_humidity_5.m__sensor_value.text = str('{:.1f}'.format(sensor5.get_humidity_crc8()))
        self.m__sensor_temp_5.m__sensor_value.text = str('{:.1f}'.format(sensor5.get_temperature_crc8()))
        test.disable_channels([5])
        
        # Channel 6 - AHT20
        test.enable_channels([6])
        sensor6 = AHT20.AHT20()
        self.m__sensor_humidity_6.m__sensor_value.text = str('{:.1f}'.format(sensor6.get_humidity_crc8()))
        self.m__sensor_temp_6.m__sensor_value.text = str('{:.1f}'.format(sensor6.get_temperature_crc8()))
        test.disable_channels([6])
        
        # Channel 7 - AHT20
        test.enable_channels([7])
        sensor7 = AHT20.AHT20()
        self.m__sensor_humidity_7.m__sensor_value.text = str('{:.1f}'.format(sensor7.get_humidity_crc8()))
        self.m__sensor_temp_7.m__sensor_value.text = str('{:.1f}'.format(sensor7.get_temperature_crc8()))
        test.disable_channels([7])
        
        # Channel 3 - Scale
        self.m__sensor_m.m__sensor_value.text = str('{:.2f}'.format(scale.getWeight()))

        #Data Logging
        if self.last_data != dt_triggerm:
            log_data = [curDT, self.temperature, self.humidity, round(o2_ppm,2), round(co2_ppm,2), self.pwminlet, self.pwmoutlet, self.ledlogic, self.refriLogic]
        
            with open('/home/bananarpi/Desktop/Data_Ujicoba/Log/log_'+log_name+'.csv', 'a+', encoding='UTF8', newline='') as log:
                writedata = csv.writer(log)
                #Write the data
                writedata.writerow(log_data)
            
            self.last_data = dt_triggerm
        
        #Camera Snapshot
        if self.last_camera != dt_triggerh:
            
            camera.capture('/home/bananarpi/Desktop/Data_Ujicoba/Image/image_'+dt_name+'.jpg')
            
            self.last_camera = dt_triggerh
        
    def _update_pwm_inlet(self, val):
        pwm01.start(val) #inlet
        self.pwminlet = val
        
    def _update_pwm_outlet(self, val):
        pwm02.start(val) #outlet
        self.pwmoutlet = val
        
    #Read ADC Data
    def read_mcp3002(self, channel):
        if channel == 0:
            cmd = 0b01100000
        else:
            cmd = 0b01110000
        spi_data = spi.xfer2([cmd,0]) # send hi_byte, low_byte; receive hi_byte, low_byte
        adc_data = ((spi_data[0] & 3) << 8) + spi_data[1]
        return adc_data

    def _btn_press1(self):
        if self.refriLogic:
            self.refriLogic = False
        else:
            self.refriLogic = True
        GPIO.output(refri,self.refriLogic)
        
    def _btn_press2(self):
        if self.ledlogic:
            self.ledlogic = False
        else:
            self.ledlogic = True
        GPIO.output(led,self.ledlogic)
        
    def _btn_press3(self):
        path = "/home/bananarpi/Desktop/Data_Ujicoba/Log"
        opener = "open" if sys.platform == "darwin" else "xdg-open"
        subprocess.call([opener, path])
        
    def _btn_press4(self):
        path = "/home/bananarpi/Desktop/Data_Ujicoba/Image"
        opener = "open" if sys.platform == "darwin" else "xdg-open"
        subprocess.call([opener, path])
        
    def _btn_press5(self):
        dt_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        camera.capture('/home/bananarpi/Desktop/Data_Ujicoba/Image/image_'+dt_name+'.jpg')

    # ~ def _btn_press6(self):
    # ~ def _btn_press7(self):
    def _btn_press8(self):
        sys.exit()
