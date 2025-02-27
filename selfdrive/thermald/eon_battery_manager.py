from selfdrive.thermald.power_monitoring import VBATT_PAUSE_CHARGING
from selfdrive.hardware import HARDWARE,EON
BATT_PERC_MIN = 40
BATT_PERC_MAX = 60

def setEONChargingStatus(car_voltage_mV, batteryPercent) :
    if EON:
        if car_voltage_mV is None or batteryPercent is None :
            HARDWARE.set_battery_charging(True)
            return
        # print( "car_voltage_mV:",car_voltage_mV)
        # print( "batteryPercent:",batteryPercent)
        # print( "VBATT_PAUSE_CHARGING:",VBATT_PAUSE_CHARGING)
        # print("VBATT_PAUSE_CHARGING * 1e3:", VBATT_PAUSE_CHARGING * 1e3)
        if HARDWARE.get_battery_charging() :
            # print("log purpose : HARDWARE.get_battery_charging()  True ")
            # if batteryPercent > BATT_PERC_MAX or car_voltage_mV  < VBATT_PAUSE_CHARGING * 1e3 :
            if batteryPercent > BATT_PERC_MAX : #or car_voltage_mV < VBATT_PAUSE_CHARGING * 1e3:
                # print("log purpose : HARDWARE.set_battery_charging(False)  False ")
                HARDWARE.set_battery_charging(False)
                return
        else :
            # print("log purpose : HARDWARE.get_battery_charging()  False ")
            #if batteryPercent < BATT_PERC_MIN and car_voltage_mV  > VBATT_PAUSE_CHARGING * 1e3:
            if batteryPercent < BATT_PERC_MIN :#and car_voltage_mV > VBATT_PAUSE_CHARGING * 1e3:
                #print("log purpose : HARDWARE.set_battery_charging(True)  True ")
                HARDWARE.set_battery_charging(True)
                return
   

