#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      Laurent Carré
#
# Created:     06/05/2020
# Copyright:   (c) Laurent Carré Sterwen Technologies 2020
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import json
import obd

from vehicle_service import *


class VehicleSimulator():

    def __init__(self,logger,file):

        self._logger=logger
        self._file=file
        self._connected=False
        self._engine_on=False
        self._reqcmd=None

    def connect(self,MAC):

        try:
            self.fd=open(self._file,"r")
        except IOError as err:
            self._logger.error(str(err))
            raise
        self._connected=True
        self.engine_on=True

    def read_data(self):
        self._logger.debug("OBD Simulator - read data")
        try:
            buf=self.fd.readline()
        except IOError as err:
            self._logger.error(str(err))
            self._connected=False
            self._engine_on=False
            self.fd.close()
            return
        if len(buf) == 0 :
            self._connected=False
            self._engine_on=False
            self.fd.close()
            return
        self.values={}
        res=json.loads(buf)
        values=res['values']
        for cmd_val in values :
            if len(cmd_val) != 3 : continue
            self.values[cmd_val[0]] = SIM_Values(cmd_val)

    def status(self):
        return (self._connected,self._engine_on)

    def obd_protocol(self):
        return "OBD Simulator"

    def last_error(self):
        return ""

    def release(self):
        return

    def bound(self):
        return self._connected

    def get_values(self):
        return self.values.values()

    def getAllCmdsList(self):
        return OBDCmdList(obd.commands[1])

    def setRequestCommands(self,cmds):
        self._reqcmd=cmds

    def getActualCmds(self):
        if self._reqcmd == None : return ""
        out=self._reqcmd[0]
        for cmd in self._reqcmd[1:] :
            out += ','
            out += cmd
        return out

    def actualCmdsNum(self):
        if self._reqcmd == None : return 0
        return len(self._reqcmd)

class SIM_Values():

    def __init__(self,cmd_val):
        self._cmd=cmd_val[0]
        self._genericType =0
        self._magnitude=cmd_val[1]
        self._unit=cmd_val[2]



def main():
    pass

if __name__ == '__main__':
    main()
