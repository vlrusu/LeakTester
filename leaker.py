import time
import logging
import json
import serial
import os
import bme680


from ctypes import *
import time
import threading
import smbus2
from smbus2 import SMBus

from datetime import datetime

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS


NCHANNELS = 6
SERIALDATALENGTH  = 30


class Leaker:
    def __init__ (self,ser,ilplog,bmelog):

        self.ser = ser
        self.ilplog = ilplog
        self.bmelog = bmelog        

        sensor = bme680.BME680()

        sensor.set_humidity_oversample(bme680.OS_2X)
        sensor.set_pressure_oversample(bme680.OS_4X)
        sensor.set_temperature_oversample(bme680.OS_8X)
        sensor.set_filter(bme680.FILTER_SIZE_3)

        sensor.set_gas_status(bme680.ENABLE_GAS_MEAS)
        sensor.set_gas_heater_temperature(320)
        sensor.set_gas_heater_duration(150)
        sensor.select_gas_heater_profile(0)

        self.sensor=sensor

        # Logfile stuff
        logging.basicConfig(filename='leaker.log',format='%(asctime)s %(message)s',encoding='utf-8',level=logging.DEBUG)
        
        # InfluxDB stuff
        token="SEPu5tOASl-Rha8SzlVQTvVfbJ2zNAewynbviYFBCVsuFxeI2EDRXkHbkgtWCio7d0E5UHcMX0cRBqwwjAtJpw=="
        self.org = "mu2e"
        self.bucket = "LeakTester"
        self.client = InfluxDBClient(url="http://trackerpsu0.dhcp.fnal.gov:8086", token=token, org=self.org)
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)


    def __del__(self):
        # self.client.close()
        self.ilplog.close()
        self.bmelog.close()



    def logbmedata(self):

        if self.sensor.get_sensor_data():
            output = "{0:.2f} {1:.2f} {2:.2f} {3:.2f}".format(self.sensor.data.temperature, self.sensor.data.pressure, self.sensor.data.humidity,self.sensor.data.gas_resistance)
            self.bmelog.write(datetime.now().strftime("%Y:%m:%d-%H:%M:%S "))            
            self.bmelog.write(output)
            self.bmelog.write("\n")
            self.bmelog.flush()

            point = Point("bmedata") \
                .tag("user", "vrusu") \
                .field("pressure", self.sensor.data.pressure) \
                .field("temp", self.sensor.data.temperature) \
                .field("humidity", self.sensor.data.humidity) \
                .field("gasresistance", self.sensor.data.gas_resistance) \
                .time(datetime.utcnow(), WritePrecision.NS)

            try:
                self.write_api.write(self.bucket, self.org, point)
            except:
                logging.error("influxdb failed in bmelog")

            
    def logilpdata(self):

        try:
            line = self.ser.readline().decode('ascii')


            d = line.split()
            if (len(d) != SERIALDATALENGTH):
                logging.error('data from serial not the right length')
                logging.error(len(d))
                logging.error(line)                
                return 0

            ilplist = []
            self.pressure = [0]*NCHANNELS
            self.temps = [0]*NCHANNELS
            for ich in range(NCHANNELS):
                self.pressure[ich] = float(d[5*ich+3])
                self.temps[ich] = float(d[5*ich+4])
                ilplist.append(self.pressure[ich])
                ilplist.append(self.temps[ich])

            self.ilplog.write(datetime.now().strftime("%Y:%m:%d-%H:%M:%S "))            
            self.ilplog.write(" ".join(str(e) for e in ilplist))
            self.ilplog.write("\n")

            self.ilplog.flush()


            point = Point("ilpdata") \
                .tag("user", "vrusu") \
                .field("pressure-0", self.pressure[0]) \
                .field("temp-0", self.temps[0]) \
                .field("pressure-1", self.pressure[1]) \
                .field("temp-1", self.temps[1]) \
                .field("pressure-2", self.pressure[2]) \
                .field("temp-2", self.temps[2]) \
                .field("pressure-3", self.pressure[3]) \
                .field("temp-3", self.temps[3]) \
                .field("pressure-4", self.pressure[4]) \
                .field("temp-4", self.temps[4]) \
                .field("pressure-5", self.pressure[5]) \
                .field("temp-5", self.temps[5]) \
                .time(datetime.utcnow(), WritePrecision.NS)


            try:
                self.write_api.write(self.bucket, self.org, point)
            except:
                logging.error("influxdb failed in ilplog")
        except:
            logging.error("ILP logging failed")


def ilploop():

    while (1):
        leaker.logilpdata()
def bmeloop():

    while (1):
        leaker.logbmedata()
        time.sleep(2)
            

if __name__ == '__main__':
    

    ser = serial.Serial('/dev/ttyACM0', 115200)

    topdir = os.path.dirname(os.path.realpath(__file__))
    # Log files
    logilp = open(os.path.join(topdir,"ilpdata.log"),"w")
    logbme = open(os.path.join(topdir,"bmedata.log"),"w")




    leaker = Leaker(ser,logilp,logbme)

    threads = []
    thrd1 = threading.Thread(target=ilploop, daemon = True, name="ILPLOOP")
    thrd1.start()
    threads.append(thrd1)
    thrd2 = threading.Thread(target=bmeloop, daemon = True, name="BMELOOP")
    thrd2.start()
    threads.append(thrd2)


    for t in threads:
        t.join()


