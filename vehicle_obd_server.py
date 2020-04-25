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
from vehicle_service import *

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
                    try:
                        self._vs.read_data()
                    except VehicleOBDException :
                        pass
                    r= buildResponse(self._vs)
                    try:
                        self._queue.put(r,timeout=30.)
                    except queue.Full :
                        loc_log.error("Vehicle service OBD queue full")
                        return

                    time.sleep(10.) # adjustable
                else:
                    time.sleep(10.)
            else:
                if nb_retries >= self._connect_retry :
                    loc_log.error("Cannot connect ot OBD after "+str(nb_retries)+" attempt")
                    return
                self._vs.connect(self._mac)
                self._connect_lock.release()
                nb_retries += 1
                if self._vs.bound() :
                    c_stat,e_stat=self._vs.status()
                    if not c_stat:
                        # wait 10 sec and retry
                        time.sleep(10.)
                    else:
                        nb_retries=0
                else:
                    # no bind possible on MAC nothing else to do
                    return


    def start_read(self,param):
        self._running_read=True

    def stop_read(self):
        self._running_read=False

    def wait(self):
        self._connect_lock.acquire()



class OBD_Servicer(OBD_Service_pb2_grpc.OBD_ServiceServicer):

    def __init__(self,vehicle):
        self._state=state_UNKNOWN
        self._supervisor=None
        self._vs=vehicle

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
        self._queue=queue.Queue(10)
        self._supervisor=OBD_Supervisor(self._vs,request.MAC,self._queue)
        self._supervisor.start()
        self._supervisor.wait()
        res=OBD_Result()
        res.connected, res.engine_on = self._vs.status()
        res.error=self._vs.last_error()
        res.obd_time=datetime.datetime.now().isoformat(' ')
        return res


    def Read(self,request,context):
        loc_log.debug("Vehicle service gRPC - READ request")
        if self._supervisor == None or not self._supervisor.is_alive() :
            res=OBD_Result()
            res.error="Attempt to read an un-activated OBD connection"
            loc_log.error(res.error)
            res.connected=False
            res.engine_on=False
            res.obd_time=datetime.datetime.now().isoformat(' ')
            yield res
            return

        self._supervisor.start_read(request.rules)
        self._stop_flag=False
        while True:
            if self._stop_flag :
                # empty the queue
                # to be done
                loc_log.debug("Stop vehicle read flag detected")
                while not self._queue.empty() :
                    resp=self.__queue.get()
                    yield resp
                    self._queue.task_done()
                loc_log.debug("Response queue is now empty")
                return
            else:
                resp=self._queue.get()
                loc_log.debug("Vehicle publish with:"+resp.error)
                yield resp
                self._queue.task_done()


    def Stop(self,request,context):
        self._stop_flag=True


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
            "connect_retry": 5,
            "interface": "hci0",
            "obd_trace": "info"
            }
def main():

    # logging.basicConfig(level=logging.DEBUG,format="%(asctime)s [%(levelname)s]:%(message)s",stream=sys.stdout)
    loghandler=logging.StreamHandler()
    logformat= logging.Formatter("%(asctime)s | [%(levelname)s] %(message)s")
    loghandler.setFormatter(logformat)
    log_vs=logging.getLogger("VehicleService")
    log_vs.addHandler(loghandler)
    param=SolidSenseParameters('vehicle',default_parameters,log_vs)
    log_vs.setLevel(param.getLogLevel())
    '''
    set logging in obd
    subsystem
    '''
    log_obd=logging.getLogger('obd')
    log_obd.addHandler(loghandler)
    obd_trace_level=param.getLogLevel('obd_trace')
    print("OBD trace level:",obd_trace_level)
    log_obd.setLevel(obd_trace_level)

    print("Creating the service")
    interface=param('interface','hci0')[3:4]
    vs=VehicleService(log_vs,interface)
    grpc_server=Vehicle_GRPC_Service(vs)
    grpc_server.start()

    try:
        grpc_server.waitEnd()
    except KeyboardInterrupt :
        exit(0)
    '''
    if len(sys.argv) < 2 :
        print("Missing MAC address")
        return

    print ("Connecting to",sys.argv[1])
    if not vs.connect(sys.argv[1]) :
        log_vs.critical("Cannot connect to DB => exit")
        exit(0)

    try:
        while True:
            vs.read_data()
            # vs.printValues()
            time.sleep(20.)
    except Exception as e :
        print("OBD read stopped by:",e)
        vs.disconnect()
    '''
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
