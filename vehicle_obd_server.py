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
import os
import inspect

cmd_subfolder = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0], "../common")))
sys.path.insert(0, cmd_subfolder)


import subprocess
import logging
import serial
import threading
import queue
from concurrent import futures

from solidsense_parameters import *
from solidsense_led import *
from vehicle_service import *
from simulator import *

import grpc
from OBD_Service_pb2  import *
import OBD_Service_pb2_grpc

loc_log=logging.getLogger("VehicleService")

state_UNKNOWN=0
state_ACTIVE=1      # trying to connect
state_CONNECTED=2
state_ENGINE_ON=3


def buildResponse(vs):
    r= OBD_Result()
    r.connected=True
    r.engine_on=vs.engine_on
    r.obd_time=datetime.datetime.now().isoformat(' ')
    r.error=vs.last_error()
    if r.engine_on :
        for c in vs.get_values():
            v=r.values.add()
            v.cmd=c._cmd
            v.type=c._genericType
            if v.type == 0 :
                v.f=c._magnitude
                v.unit=c._unit
            else:
                v.s=c._magnitude
    return r


class OBD_Supervisor(threading.Thread):

    def __init__(self,vs,mac,queue):
        self._mac=mac
        self._vs=vs
        self._queue=queue
        threading.Thread.__init__(self)
        self._state=state_UNKNOWN
        self._running_read = False
        self._connect_lock=threading.Semaphore()
        self._connect_retry = SolidSenseParameters.active_set().get('connect_retry')
        self._on_period=10.
        self._off_period=30.
        lednum= SolidSenseParameters.active_set().get('led')
        if lednum != None:
            self.led=SolidSenseLed.ledref(lednum)
        else:
            self.led=None

    def run(self):
        self._state=state_ACTIVE
        self._connect_lock.acquire()
        # release all rfcomm bound connections
        self._vs.release()
        nb_retries = 0
        while True:
            c_stat,e_stat=self._vs.status()
            if c_stat :
                if self._running_read :
                    if self.led != None : self.led.green_only(255)
                    try:
                        self._vs.read_data()
                    except VehicleOBDException :
                        pass
                    r= buildResponse(self._vs)
                    if self.led != None : self.led.off()
                    try:
                        self._queue.put(r,timeout=30.)
                    except queue.Full :
                        loc_log.error("Vehicle service OBD queue full")
                        # stop reading
                        self._running_read = False
                        # return
                    time.sleep(self._on_period) # adjustable
                else:
                    time.sleep(10.)
            else:
                if nb_retries >= self._connect_retry :
                    loc_log.error("Cannot connect ot OBD after "+str(nb_retries)+" attempt")
                    if self.led != None : self.led.off()
                    return
                if self.led != None : self.led.red_only(255)
                self._vs.connect(self._mac)
                self._connect_lock.release()
                nb_retries += 1
                if self._vs.bound() :
                    c_stat,e_stat=self._vs.status()
                    if not c_stat:
                        # wait 10 sec and retry
                        if self.led != None : self.led.off()
                        time.sleep(self._off_period)
                    else:
                        nb_retries=0
                else:
                    # no bind possible on MAC nothing else to do
                    return


    def start_read(self,param):
        self._running_read=True
        if param != None :
            if param.get('on_period') != None :
                self._on_period =  param.get('on_period')
            if param.get('off_period') != None :
                self._on_period =  param.get('off_period')

    def stop_read(self):
        self._running_read=False

    def wait(self):
        self._connect_lock.acquire()



class OBD_Servicer(OBD_Service_pb2_grpc.OBD_ServiceServicer):

    def __init__(self,vehicle):
        self._state=state_UNKNOWN
        self._supervisor=None
        self._vs=vehicle
        self._stop_flag=False
        self._MAC=SolidSenseParameters.getParam("MAC")


    def Status(self,request,context):
        loc_log.debug('Vehicle service gRPC - Status request')
        res=OBD_status()
        res.connected, res.engine_on = self._vs.status()
        res.autoconnect = SolidSenseParameters.getParam('autoconnect')
        if res.autoconnect :
            res.MAC=self._MAC
        if res.connected :
            res.protocol=self._vs.obd_protocol()
            if self._supervisor != None and  self._supervisor.is_alive() :
                if self._supervisor._running_read:
                    res.state="run"
                else:
                    res.state="idle"
            else:
                res.state="inactive"
            if request.request == "vehicle_cmds" :
                res.commands=self._vs.getAllCmdsList()
            elif request.request == "actual_cmds"  :
                res.commands=self._vs.getActualCmdsList()
        return res

    def Connect(self,request,context):
        loc_log.debug("Vehicle Service gRPC - CONNECT request")
        if self._supervisor != None :
            # first check if the Thread is running if yes, this is an eoor, if not
            if self._supervisor.is_alive() :
                loc_log.error("Vehicle Service gRPC - Already connected")
                res=OBD_Result()
                res.connected, res.engine_on = self._vs.status()
                res.error="Attempt to connect while connected"
                return res
        self.start_connection(request.MAC)
        res=OBD_Result()
        res.connected, res.engine_on = self._vs.status()
        res.error=self._vs.last_error()
        if res.error == "" :
            self._MAC=request.MAC
        res.obd_time=datetime.datetime.now().isoformat(' ')
        return res


    def Read(self,request,context):
        loc_log.debug("Vehicle service gRPC - READ request")
        if self._supervisor == None or not self._supervisor.is_alive() :
            # We are not ready no active connection to OBD
            if SolidSenseParameters.getParam('autoconnect') and self._MAC != None :
                loc_log.info("Performing auto connection on:"+self._MAC)
                self.start_connection(self._MAC)
            else:
                res=OBD_Result()
                res.error="Attempt to read an un-activated OBD connection"
                loc_log.error(res.error)
                res.connected=False
                res.engine_on=False
                res.obd_time=datetime.datetime.now().isoformat(' ')
                yield res
                return
        try:
            rules=json.loads(request.rules)
        except ValueError as e:
            loc_log.error("Vehicle service - Read request wrong rules:"+str(e))
            rules=None
        if request.request == OBD_cmd.RequestResult.Specific :
            # specifc set of commands is to be used
            if request.commands != None and type(request.commands) == str:
                commands=request.commands.split(',')
                if len(commands) > 0 :
                    self._vs.setRequestCommands(commands)
                    loc_log.info("Vehicle service number of actual commands:"+str(self._vs.actualCmdsNum()))

        self._supervisor.start_read(rules)
        self._stop_flag=False
        while True:
            if self._stop_flag :
                # empty the queue
                # to be done
                loc_log.debug("Stop vehicle read flag detected")
                self._supervisor.stop_read()
                while not self._queue.empty() :
                    resp=self.__queue.get()
                    yield resp
                    self._queue.task_done()
                loc_log.debug("Response queue is now empty")
                if self._stop_flag:
                    return
            else:
                resp=self._queue.get()
                loc_log.debug("Vehicle publish with:"+resp.error)
                yield resp
                self._queue.task_done()


    def Stop(self,request,context):
        loc_log.debug("Vehicle service gRPC - STOP READ request")
        self._stop_flag=True
        res=OBD_Result()
        res.connected, res.engine_on = self._vs.status()
        res.error=self._vs.last_error()
        res.obd_time=datetime.datetime.now().isoformat(' ')
        return res

    def start_connection(self,MAC):
        self._queue=queue.Queue(10)
        self._supervisor=OBD_Supervisor(self._vs,MAC,self._queue)
        self._supervisor.start()
        self._supervisor.wait()

class Vehicle_GRPC_Service():

    def __init__(self,vehicle):
        self._server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        OBD_Service_pb2_grpc.add_OBD_ServiceServicer_to_server(OBD_Servicer(vehicle),self._server)
        self._endEvent=threading.Event()
        l_address = SolidSenseParameters.active_set().get('address')
        port=SolidSenseParameters.active_set().get('port')
        address=l_address+':'+str(port)
        self._server.add_insecure_port(address)
        self._endEvent=threading.Event()
        loc_log.info("Vehicle service - GRPC server ready on:"+address)

    def start(self):
        loc_log.info("Vehicle Service - starting gRPC server")
        self._server.start()

    def stop(self):
        loc_log.info("Vehicle service - stopping gRPC server")
        self._finalEnd=self._server.stop(1.0)
        self._endEvent.set()

    def waitEnd(self):
        self._endEvent.wait()
        # print("End event received")
        self._finalEnd.wait()
        # print("Final event received")

default_parameters= {
            "trace": "debug",
            "address": "0.0.0.0",
            "port" : "20232",
            "led": 1,
            "connect_retry": 5,
            "interface": "hci0",
            "obd_trace": "info",
            "autoconnect": True,
            "MAC": "00:01:02:03:04:05"
            }

def main():

    # logging.basicConfig(level=logging.DEBUG,format="%(asctime)s [%(levelname)s]:%(message)s",stream=sys.stdout)
    loghandler=logging.StreamHandler()
    logformat= logging.Formatter("%(asctime)s | [%(levelname)s] %(message)s")
    loghandler.setFormatter(logformat)
    log_vs=logging.getLogger("VehicleService")
    log_vs.addHandler(loghandler)
    param=SolidSenseParameters('vehicle',default_parameters)
    log_vs.setLevel(param.getLogLevel())
    simul=False
    if len(sys.argv) > 1 :
        if sys.argv[1] == "simul" :
            if len(sys.argv) > 2 :
                simul=True
                file=sys.argv[2]

    '''
    set logging in obd
    subsystem
    '''
    led=SolidSenseLed.ledref(
            param.getParam('led'))
    if led != None:
        led.off()
    log_obd=logging.getLogger('obd')
    log_obd.addHandler(loghandler)
    obd_trace_level=param.getLogLevel('obd_trace')
    print("OBD trace level:",obd_trace_level)
    log_obd.setLevel(obd_trace_level)

    print("Creating the service")
    if simul:
        vs=VehicleSimulator(log_vs,file)
    else:
        interface=param.get('interface','hci0')[3:4]
        vs=VehicleService(log_vs,interface)

    grpc_server=Vehicle_GRPC_Service(vs)
    grpc_server.start()

    try:
        grpc_server.waitEnd()
    except KeyboardInterrupt :
        led.off()
        exit(0)

if __name__ == "__main__":
    main()
