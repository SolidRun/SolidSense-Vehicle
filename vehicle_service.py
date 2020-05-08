# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
# Name:        Vehicle Service.py
# Purpose:     Script for the INEM Demo
#
# Author:      Laurent Carre/Malte Grosse
#
# Created:     14/04/2019
# Copyright:   (c) Laurent Carre - Sterwen Technology / Solid Run 2019
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import datetime
import time
import sys
import json

from obd import OBDStatus

import obd
import subprocess
import logging
import serial
import threading


loc_log=logging.getLogger("VehicleService")

class VehicleOBDException (Exception) :
    pass

class OBDActiveCMD():

    def __init__(self,cmd_name):
        cmd=obd.commands[cmd_name]
        self._cmd=cmd   # this is an OBD command
        self._active=False
        self._result=None

    @property
    def command(self):
        return self._cmd

    @property
    def active(self):
        return self._active

    def setActive(self):
        self._active=True


def  OBDCmdList(cmds):
    out=cmds[0].name
    for cmd in cmds[1:]:
        out += ","
        out += cmd.name
    return out


class VehicleService(object):

    def __init__(self, logger, interface):


        self._logger = logger
        self.INTERFACE = interface
        self._connected=False
        self.engine_on=False
        self._actual_commands=None
        self._request_commands=None
        self._obd_status="Unknown"
        self._error=""
        self._bound=False
        self._all_cmds=None

        self.values={}
        #
        self.setDefaultCmdList()

    def setDefaultCmdList(self):
        self._default_commands={}
        try:
            fd=open("/data/solidsense/vehicle/std_obd_cmd.json","r")
        except IOError as err:
            self._logger.error("Default OBD command file:"+str(err))
            return
        try:
            all_cmds=json.load(fd)
        except ValueError as err:
            self._logger.error("Error decoding standard OBD command file:"+str(err))
            return


        for c in all_cmds :
            o_cmd=OBDActiveCMD(c['name'])
            self._default_commands[o_cmd.command]=o_cmd


    def setActualCommands(self,cmd_list) :
        self._actual_commands=[]
        for cmd in cmd_list :
           try:
            o_cmd=self._default_commands[cmd]
           except KeyError :
            continue

           o_cmd.setActive()
           self._actual_commands.append(o_cmd.command)

    def setRequestCommands(self,cmd_list):
        self._request_commands=[]
        for cmd in cmd_list :
            try:
                o_cmd=self._default_commands[cmd]
            except KeyError :
                continue
            if o_cmd.active :
                self._request_commands.append(o_cmd.command)

    def actualCmdsNum(self):
        return len(self._request_commands)

    def clear_error(self):
        self._error=""

    def connect(self,MAC):
        '''
        Initiate the Bluetooth connection via rfcomm bind
        and connect to the OBD device
        '''
        self.MAC_ADDRESS=MAC.upper()
        if not self.checkMAC() :
            self.bind()
            if not self.checkMAC() :
                self._error="Cannot bind to adress:"+self.MAC_ADDRESS
                self._logger.error(self._error)
                return False

        self._logger.info('VEHICLE_SERVICE: Starting on + ' + self.INTERFACE + ' ' + self.MAC_ADDRESS +' port:' + self._port)
        return self.connectToOBD()


    def checkMAC(self):
        # check if the MAC address is already bound with rfcomm
        res=subprocess.Popen("rfcomm",stdout=subprocess.PIPE)
        val=res.stdout.readline().decode('utf-8')
        self._logger.debug("rfcomm returned values: "+val)
        if len(val) > 0 :
            v=val.split()
        #print (v)
            if self.MAC_ADDRESS !=v[1] :
               self.release()
               self._bound=False
               return False
            self._port='/dev/'+v[0].rstrip(':')
            self._bound=True
            return True
        else:
            self._bound=False
            return False


    def bind(self):
        self._logger.debug("Binding address:"+self.MAC_ADDRESS)
        res=subprocess.Popen(["rfcomm","bind",self.INTERFACE,self.MAC_ADDRESS],stderr=subprocess.PIPE)

    def bound(self):
        return self._bound

    def release(self):
        self._logger.debug("Releasing all Bluetooth bound devices")
        res=subprocess.Popen(["rfcomm","release","all"])


    def connectToOBD(self):

        self._logger.info('VEHICLE_SERVICE: connecting to port:'+ self._port)
        self._connected=False

            # print ("OBD serial port:",port)
        try:
            self.odb_connection = obd.OBD(portstr=self._port)
        except Exception as err:
            self._error="Cannot connect to OBD:"+str(err)
            self._logger.error(self._error)
            return False
        self._connected=self.odb_connection.is_connected()
        self._obd_status= self.odb_connection.status()
        if self._connected :
            self._all_cmds = self.odb_connection.supported_commands
            self.setActualCommands(self._all_cmds)
            self._request_commands=self._actual_commands

            # self._logger.debug("OBD connection:"+self._obd_status+' protocol ' + self.odb_connection.protocol_name())
            self._error= 'CONNECTED! Protocol '+self.odb_connection.protocol_name()+' #CMDS:'+str(len(self._all_cmds))
            self._logger.info('VEHICLE_SERVICE: '+self._error)
            self.read_elm327()
            self.engine_on=True
            return True
        else :
            self._error='Device (' + self._port + ' ' + self.MAC_ADDRESS + ') status:'+self._obd_status
            self._logger.error(self._error)
            self._logger.info('VEHICLE_SERVICE NOT CONNECTED (ENGINE OFF): '+self._port)
            self.engine_on=False
            #exit(1)
        return False

    def read_elm327(self):
        self._elm_version=self._read_odb(obd.commands.ELM_VERSION)
        self._elm_voltage=self._read_odb(obd.commands.ELM_VOLTAGE)
        self._logger.debug("ELM VERSION:"+self._elm_version+" Voltage:"+str(self._elm_voltage))

    def obd_protocol(self):
        if self._connected :
            return self.odb_connection.protocol_name()
        else:
            return ""

    def disconnect(self):
        if not self._connected : return
        try:
            self.odb_connection.disconnect()
        except:
            pass
        self._connected=False
        self._actual_commands = None
        self.odb_connection=None

    def retryConnect(self):
        if self._connected :
            self.disconnect()
        if self.connectToOBD() :
            self.checkEngine()
        else:
            return False

    def checkEngine(self):
        '''
        Check if the engine is running while connected
        '''
        if self._connected :

            tmp_speed = self._read_odb(obd.commands.SPEED)

            if tmp_speed is None:
            # maybe we have lost connection
                self._obd_status=self.odb_connection.status()
                self._status =  self._obd_status
                self.engine_on = False
                self._logger.debug('VEHICLE_SERVICE: Engine not running - Status:'+self._obd_status)
                self.disconnect()
                return False
            else:
                self.engine_on = True
                return True
        else:
            return False

    def _read_odb(self,cmd):
        # actual read to odb
        try:
            res = self.odb_connection.query(cmd).value
        except serial.SerialException as err :
            self._error='VEHICLE SERVICE: OBD Communication error:'+str(err)
            self._logger.error(self._error)
            self._status = 'OBD Disconnected'
            self.engine_on = False
            self.disconnect()
            raise VehicleOBDException(self._error)
        return res

    def read_data(self):

        self._logger.debug('VEHICLE_SERVICE: reading OBD data')
        if not self._connected :
            self.retryConnect()
        if not self._connected:
            self._status = 'OBD Disconnected'
            self.engine_on = False
            self._logger.debug('VEHICLE_SERVICE: No OBD connection, engine off?')
            return False
        if not self.checkEngine() :
            return False
        #
        # if we are here let's go for the reading
        #
        self.nbc_read=1
        for cmd in self._request_commands:

            res=self._read_odb(cmd)  # exception is raised if problems lies below

            if res is not None:
                self.nbc_read +=1
                self.values[cmd]=CMD_Value(cmd,res)
            else:
                self._logger.debug('VEHICLE SERVICE: ERROR ON OBD COMMAND:'+cmd.name)

        # out['dtc'] = self.odb_connection.query(obd.commands.GET_DTC, force=True).value
        self._status = self._obd_status+'- data read'
        self.engine_on= True
        self._logger.debug('VEHICLE_SERVICE: Data received #commands:'+str(self.nbc_read))
        return True


    def status(self):
        return (self._connected,self.engine_on)

    def last_error(self):
        return self._error

    def obd_satus(self):
        return self._obd_status

    def printValues(self):
        for cmd, val in self.values.items() :
            if val._genericType == 0:
                print(cmd,"=",val._magnitude,val._unit)
            else:
                print(cmd,'=',val._magnitude)

    def get_values(self):
        return self.values.values()

    def getAllCmdsList(self):
        if self._all_cmds != None:
            return OBDCmdList(self._all_cmds)
        else:
            return OBDCmdList(obd.commands[1])

    def getActualCmdsList(self):
        if self._request_commands != None:
            return OBDCmdList(self._request_commands)
        else:
            return ""


    def dumpAllcommands(self,file,mode) :
        cmds=obd.commands.modes[mode]
        all_cmd=[]
        for cmd in cmds:
            cd={}
            cd['name']=cmd.name
            cd['desc']=cmd.desc
            cd['pid'] =cmd.pid
            all_cmd.append(cd)

        fp=open(file,"w")
        json.dump(all_cmd,fp,indent=1)
        fp.close()

    def dumpDefaultCMD(self):
        for cdm in self._default_commands.values():
            print(cdm.command)

    def storeValues(self,fd):
        out={}
        out["timestamp"]=datetime.datetime.now().isoformat(' ')
        out["engine_on"]=self.engine_on
        values=[]
        for cmd,val in self.values.items():
            if val._genericType == 0:
                res=(cmd.name,val._magnitude,val._unit)
            else:
                res=(cmd.name,str(val._magnitude))
            values.append(res)
        out["values"]=values
        buf=json.dumps(out)
        fd.write(buf)
        fd.write('\n')
        fd.flush()


class CMD_Value:

    def __init__(self,cmd,result):
        self._cmd= cmd.name
        self._type=type(result)
        if 'Quantity' in str(type(result)):
            self._genericType=0
            self._magnitude=result.m
            self._unit=str(result.u)
            loc_log.debug("Qty cmd="+cmd.name+" value="+str(result))
        else:
            self._genericType=1
            self._magnitude=str(result)
            self._unit=None
            loc_log.debug("Not qty!"+cmd.name+" v:"+str(result)+" Type:"+str(type(result)))



def main():

    # logging.basicConfig(level=logging.DEBUG,format="%(asctime)s [%(levelname)s]:%(message)s",stream=sys.stdout)
    loghandler=logging.StreamHandler()
    logformat= logging.Formatter("%(asctime)s | [%(levelname)s] %(message)s")
    loghandler.setFormatter(logformat)
    log_vs=logging.getLogger("VehicleService")
    log_vs.addHandler(loghandler)
    log_vs.setLevel(logging.DEBUG)
    '''
    set logging in obd
    subsystem
    '''
    log_obd=logging.getLogger('obd')
    log_obd.addHandler(loghandler)
    log_obd.setLevel(logging.ERROR)

    print("Creating the service")
    vs=VehicleService(log_vs,'0')
    if len(sys.argv) < 2 :
        print("Missing MAC address or cmd")
        return

    if sys.argv[1] == "dump" :
        vs.dumpAllcommands("./obd_cmd.json",1)
        return
    elif sys.argv[1] == "test" :
        vs.dumpDefaultCMD()
        return
    if len(sys.argv) > 2 and sys.argv[2]  == "store" :
        fd=open('/data/solidsense/obd_result.log','w')
        store=True
    else:
        store=False

    print ("Connecting to",sys.argv[1])
    if not vs.connect(sys.argv[1]) :
        log_vs.critical("Cannot connect to DB => exit")
        exit(0)

    try:
        while True:
            vs.read_data()
            if store :
                vs.storeValues(fd)
            vs.printValues()
            time.sleep(10.)
    except Exception as e :
        print("OBD read stopped by:",e)
        vs.disconnect()
    if store :
        fd.close()
    '''
    vs.fromMAC(sys.argv[1])
    try:
        tty=serial.Serial.open("/dev/rfcomm0",
            baudrate=38400,
            bytesize=8,
            stopbits=1,
            )
    except serial.SerialException as e:
        print (e)
        return
    try:
        tty.write(b'\r')
        tty.flush()
    except serial.SerialException as e:
        print (e)
        return
    try:
        r=tty.read(1024)
        print("response=",r)
    except serial.SerialException as e:
        print (e)
        return
    tty.close()
    '''
if __name__ == "__main__":
    main()
