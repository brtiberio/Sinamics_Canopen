#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2018 Bruno Tibério
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import canopen
import sys
import logging

class SINAMICS:

    network = None
    logger = None
    _connected = False
    # If EDS file is present, this is not necessary, all codes can be gotten
    # from object dicionary.
    objectIndex = {# control unit objects, independent of drive:
                   'Device Type': 0x1000,
                   'Error Register': 0x1001,
                   'Error History': 0x1003,
                   'COB-ID SYNC Message': 0x1005,
                   'Device Name': 0x1008,
                   'Software Version': 0x100A,
                   'Guard Time': 0x100C,
                   'Life Time Factor': 0x100D,
                   'Store Parameters': 0x1010,
                   'Restore Default Parameters': 0x1011,
                   'COB-ID Emergency Object': 0x1014,
                   # 'Consumer Heartbeat Time': 0x1016,
                   'Producer Heartbeat Time': 0x1017,
                   'Identity Object': 0x1018,
                   # 'Verify configuration': 0x1020,
                   'Module List': 0x1027,
                   'Error Behavior': 0x1029,
                   'Server SDO Default Parameter': 0x1200,
                   'Server SDO CU DO Parameter': 0x1201,
                   # other communication objects
                   'Server SDO Drive DO 1 Parameter': 0x1202,
                   'Server SDO Drive DO 2 Parameter': 0x1203,
                   'Server SDO Drive DO 3 Parameter': 0x1204,
                   'Server SDO Drive DO 4 Parameter': 0x1205,
                   'Server SDO Drive DO 5 Parameter': 0x1206,
                   'Server SDO Drive DO 6 Parameter': 0x1207,
                   'Server SDO Drive DO 7 Parameter': 0x1208,
                   'Server SDO Drive DO 8 Parameter': 0x1209,
                   # Drive dependent objects
                   'Receive PDO 1 Parameter': 0x1400,
                   'Receive PDO 2 Parameter': 0x1401,
                   'Receive PDO 3 Parameter': 0x1402,
                   'Receive PDO 4 Parameter': 0x1403,
                   'Receive PDO 5 Parameter': 0x1404,
                   'Receive PDO 6 Parameter': 0x1405,
                   'Receive PDO 7 Parameter': 0x1406,
                   'Receive PDO 8 Parameter': 0x1407,
                   'Receive PDO 1 Mapping': 0x1600,
                   'Receive PDO 2 Mapping': 0x1601,
                   'Receive PDO 3 Mapping': 0x1602,
                   'Receive PDO 4 Mapping': 0x1603,
                   'Receive PDO 5 Mapping': 0x1604,
                   'Receive PDO 6 Mapping': 0x1605,
                   'Receive PDO 7 Mapping': 0x1606,
                   'Receive PDO 8 Mapping': 0x1607,
                   'Transmit PDO 1 Parameter': 0x1800,
                   'Transmit PDO 2 Parameter': 0x1801,
                   'Transmit PDO 3 Parameter': 0x1802,
                   'Transmit PDO 4 Parameter': 0x1803,
                   'Transmit PDO 5 Parameter': 0x1804,
                   'Transmit PDO 6 Parameter': 0x1805,
                   'Transmit PDO 7 Parameter': 0x1806,
                   'Transmit PDO 8 Parameter': 0x1807,
                   'Transmit PDO 1 Mapping': 0x1A00,
                   'Transmit PDO 2 Mapping': 0x1A01,
                   'Transmit PDO 3 Mapping': 0x1A02,
                   'Transmit PDO 4 Mapping': 0x1A03,
                   'Transmit PDO 5 Mapping': 0x1A04,
                   'Transmit PDO 6 Mapping': 0x1A05,
                   'Transmit PDO 7 Mapping': 0x1A06,
                   'Transmit PDO 8 Mapping': 0x1A07,
                #    'Node ID': 0x2000,
                #    'CAN Bitrate': 0x2001,
                #    'RS232 Baudrate': 0x2002,
                #    'Version Numbers': 0x2003,
                #    'Serial Number': 0x2004,
                #    'RS232 Frame Timeout': 0x2005,
                #    'Miscellaneous Configuration': 0x2008,
                #    'Internal Dip Switch State': 0x2009,
                #    'Custom persistent memory': 0x200C,
                #    'Internal DataRecorder Control': 0x2010,
                #    'Internal DataRecorder Configuration': 0x2011,
                #    'Internal DataRecorder Sampling Period': 0x2012,
                #    'Internal DataRecorder Number of Preceding Samples': 0x2013,
                #    'Internal DataRecorder Number of Sampling Variables': 0x2014,
                #    'DataRecorder Index of Variables': 0x2015,
                #    'Internal DataRecorder SubIndex of Variables': 0x2016,
                #    'Internal DataRecorder Status': 0x2017,
                #    'Internal DataRecorder Max Number of Samples': 0x2018,
                #    'Internal DataRecorder Number of Recorded Samples': 0x2019,
                #    'Internal DataRecorder Vector Start Offset': 0x201A,
                #    'Encoder Counter': 0x2020,
                #    'Encoder Counter at Index Pulse': 0x2021,
                #    'Hallsensor Pattern': 0x2022,
                #    'Internal Object Demand Rotor Angle': 0x2023,
                #    'Internal System State': 0x2024,
                #    'Internal Object Reserved': 0x2025,
                #    'Internal Object ProcessMemory': 0x2026,
                #    'Current Actual Value Averaged': 0x2027,
                #    'Velocity Actual Value Averaged': 0x2028,
                #    'Internal Object Actual Rotor Angle': 0x2029,
                #    'Internal Object NTC Temperatur Sensor Value': 0x202A,
                #    'Internal Object Motor Phase Current U': 0x202B,
                #    'Internal Object Motor Phase Current V': 0x202C,
                #    'Internal Object Measured Angle Difference': 0x202D,
                #    'Trajectory Profile Time': 0x202E,
                #    'CurrentMode Setting Value': 0x2030,
                #    'PositionMode Setting Value': 0x2062,
                #    'VelocityMode Setting Value': 0x206B,
                #    'Configuration Digital Inputs': 0x2070,
                #    'Digital Input Funtionalities': 0x2071,
                #    'Position Marker': 0x2074,
                #    'Digital Output Funtionalities': 0x2078,
                #    'Configuration Digital Outputs': 0x2079,
                #    'Analog Inputs': 0x207C,
                #    'Current Threshold for Homing Mode': 0x2080,
                #    'Home Position': 0x2081,
                #    'Following Error Actual Value': 0x20F4,
                #    'Sensor Configuration': 0x2210,
                #    'Digital Position Input': 0x2300,
                #    'Internal Object Download Area': 0x2FFF,
                   # manufactor specific objects
                   'Current limit': 0x2280,
                   'Technology controller enable': 0x2898,
                   'Technology controller filter time constant': 0x28D9,
                   'Technology controller differentiation time constant': 0x28E2,
                   'Technology controller proportional gain': 0x28E8,
                   'Technology controller maximum limiting': 0x28F3,
                   'Technology controller minimum limiting': 0x28F4,
                   'Output frequency': 0x2042,
                   'Speed setpoint smoothed': 0x2014,
                   'Output frequency smoothed': 0x2018,
                   'Output voltage smoothed': 0x2019,
                   'DC link voltage smoothed': 0x201A,
                   'Absolute actual current smoothed': 0x201B,
                   'Actual torque smoothed': 0x201F,
                   'Actual active power smoothed': 0x2020,
                   'Motor temperature': 0x2023,
                   'Power unit temperatures': 0x2025,
                   'Energy display': 0x2027,
                   'Command Data Set CDS effective': 0x2032,
                   'Statusword 2': 0x2035,
                   'Controlword 1': 0x2036,
                   'CU digital input status': 0x22D2,
                   'CU digital output status': 0x22EB,
                   'CU analog inputs voltage/current': 0x22F0,
                   'CU analog outputs voltage/current': 0x2306,
                   'Fault number': 0x23B3,
                   'Actual pulse frequency': 0x2709,
                   'Alarm number': 0x283E,
                   'Technology controller setpoint after ramp': 0x28D4,
                   'Technology controller actual value after filter': 0x28DA,
                   'Technology controller output signal': 0x28F6,
                   'Technology controller ramp-up time': 0x28D1,
                   'Technology controller ramp-down time': 0x28D2,
                   'Technology controller integral action time': 0x28ED,
                   # DS402 profile
                   # device control
                   'Abort connection option code': 0x6007,
                   'ControlWord': 0x6040,
                   'StatusWord': 0x6041,
                   'Stop option code': 0x605D,
                   'Modes of Operation': 0x6060,
                   'Modes of Operation Display': 0x6061,
                   'Supported drive modes': 0x6502,
                   'Drive manufacturer': 0x6504,
                   'Single device type': 0x67FF,
                   # Factor group
                   'Velocity encoder': 0x6094,
                   # profile velocity mode
                   # TODO
                   'TargetVelocity': 0x60FF,
                   # profile torque mode
                   # TODO
                   # velocity mode
                   'vl target velocity': 0x6042,
                   'vl velocity demand': 0x6043,
                   'vl velocity actual value:': 0x6044,
                   'vl velocity limits': 0x6046,
                   'vl velocity acceleration': 0x6048}
    # CANopen defined error codes and Maxon codes also
    # AbortCode: Description
    errorIndex = {0x00000000:  'Error code: no error',
                  # 0x050x xxxx
                  0x05030000:  'Error code: toogle bit not alternated',
                  0x05040000:  'Error code: SDO protocol timeout',
                  0x05040001:  'Error code: Client/server command specifier not valid or unknown',
                  0x05040002:  'Error code: invalide block size',
                  0x05040003:  'Error code: invalide sequence number',
                  0x05040004:  'Error code: CRC error',
                  0x05040005:  'Error code: out of memory',
                  # 0x060x xxxx
                  0x06010000:  'Error code: Unsupported access to an object',
                  0x06010001:  'Error code: Attempt to read a write-only object',
                  0x06010002:  'Error code: Attempt to write a read-only object',
                  0x06020000:  'Error code: object does not exist',
                  0x06040041:  'Error code: object can not be mapped to the PDO',
                  0x06040042:  'Error code: the number and length of the objects to be mapped would exceed PDO length',
                  0x06040043:  'Error code: general parameter incompatibility',
                  0x06040047:  'Error code: general internal incompatibility in the device',
                  0x06060000:  'Error code: access failed due to an hardware error',
                  0x06070010:  'Error code: data type does not match, length of service parameter does not match',
                  0x06070012:  'Error code: data type does not match, length of service parameter too high',
                  0x06070013:  'Error code: data type does not match, length of service parameter too low',
                  0x06090011:  'Error code: subindex does not exist',
                  0x06090030:  'Error code: value range of parameter exeeded',
                  0x06090031:  'Error code: value of parameter written is too high',
                  0x06090032:  'Error code: value of parameter written is too low',
                  0x06090036:  'Error code: maximum value is less than minimum value',
                  0x060A0023:  'Error code: resource not available: SDO connection',
                  # 0x0800 xxxx
                  0x08000000:  'Error code: General error',
                  0x08000020:  'Error code: Data cannot be transferred or stored to the application',
                  0x08000021:  'Error code: Data cannot be transferred or stored to the application because of local control',
                  0x08000022:  'Error code: Wrong Device State. Data can not be transfered',
                  0x08000023:  'Error code: Object dictionary dynamic generation failed or no object dictionary present',
                  # Maxon defined error codes
                  0x0f00ffc0:  'Error code: wrong NMT state',
                  0x0f00ffbf:  'Error code: rs232 command illegeal',
                  0x0f00ffbe:  'Error code: password incorrect',
                  0x0f00ffbc:  'Error code: device not in service mode',
                  0x0f00ffB9:  'Error code: error in Node-ID'
                  }
    # dictionary describing opMode
    opModes = {0: 'No mode assigned', 3: 'Profile Velocity Mode', -1: 'Manufacturer-specific OP1',
               -2: 'Manufacturer-specific OP2', -3: 'Manufacturer-specific OP3', -4: 'Manufacturer-specific OP4',
               -5: 'Manufacturer-specific OP5', -6: 'Manufacturer-specific OP6', -7: 'Manufacturer-specific OP7',
               -15: 'Manufacturer-specific OP8', -18: 'Manufacturer-specific OP9', -20: 'Manufacturer-specific OP11',
               -22: 'Manufacturer-specific OP12', 4: 'Profile Torque Mode'}
    # dictionary describing current state of drive
    state = {0: 'start', 1: 'not ready to switch on', 2: 'switch on disable',
             3: 'ready to switch on', 4: 'switched on', 5: 'refresh',
             6: 'measure init', 7: 'operation enable', 8: 'quick stop active',
             9: 'fault reaction active (disabled)', 10: 'fault reaction active (enable)', 11: 'fault',
             -1: 'Unknown'}

    def __init__(self, _network=None, debug=False):
        # check if network is passed over or create a new one
        if not _network:
            self.network = canopen.Network()
        else:
            self.network = _network

        self.logger = logging.getLogger('SINAMICS')
        if debug:
            self.logger.setLevel(logging.DEBUG)
        else:
            self.logger.setLevel(logging.INFO)

    def begin(self, nodeID, _channel='can0', _bustype='socketcan', objectDictionary=None):
        '''Initialize SINAMICS device

        Configure and setup SINAMICS device.

        Args:
            nodeID:    Node ID of the device.
            channel (optional):   Port used for communication. Default can0
            bustype (optional):   Port type used. Default socketcan.
            objectDictionary (optional):   Name of EDS file, if any available.
        Return:
            bool: A boolean if all went ok.
        '''
        try:
            self.node = self.network.add_node(
                nodeID, object_dictionary=objectDictionary)
            self.network.connect(channel=_channel, bustype=_bustype)
            self._connected = True
        except Exception as e:
            self.logger.info('[{0}:{1}] Exception caught:{2}\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name,
                str(e)))
            self._connected = False
        finally:
            return self._connected

    def disconnect(self):
        self.network.disconnect
        return

    #--------------------------------------------------------------
    # Basic set of functions
    #--------------------------------------------------------------
    def readObject(self, index, subindex):
        '''Reads an object

         Request a read from dictionary object referenced by index and subindex.

         Args:
             index:     reference of dictionary object index
             subindex:  reference of dictionary object subindex
         Returns:
             bytes:  message returned by SINAMICS or empty if unsucessfull
        '''
        if self._connected:
            try:
                return self.node.sdo.upload(index, subindex)
            except Exception as e:
                self.logger.info('[{0}:{1}] Exception caught:{2}\n'.format(
                    self.__class__.__name__,
                    sys._getframe().f_code.co_name,
                    str(e)))
                return None
        else:
            self.logger.info('[{0}:{1}] Error: {0} is not connected\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return None

    def writeObject(self, index, subindex, data):
        '''Write an object

         Request a write to dictionary object referenced by index and subindex.

         Args:
             index:     reference of dictionary object index
             subindex:  reference of dictionary object subindex
             data:      data to be stored
         Returns:
             bool:      boolean if all went ok or not
        '''
        if self._connected:
            try:
                self.node.sdo.download(index, subindex, data)
                return True
            except canopen.SdoAbortedError as e:
                text = "Code 0x{:08X}".format(e.code)
                if e.code in self.errorIndex:
                    text = text + ", " + self.errorIndex[e.code]
                self.logger.info('[{0}:{1}] SdoAbortedError: '.format(
                    self.__class__.__name__,
                    sys._getframe().f_code.co_name) + text)
                return False
            except canopen.SdoCommunicationError:
                self.logger.info('[{0}:{1}] SdoAbortedError: Timeout or unexpected response'.format(
                    self.__class__.__name__,
                    sys._getframe().f_code.co_name))
                return False
        else:
            self.logger.info('[{0}:{1}] Error: {0} is not connected\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return False

    #------------------------------------------------------------------------------
    # High level functions
    #------------------------------------------------------------------------------
    def readStatusWord(self):
        index = self.objectIndex['StatusWord']
        subindex = 0
        statusword = self.readObject(index, subindex)
        # failded to request?
        if not statusword:
            logging.info("[EPOS:{0}] Error trying to read EPOS statusword".format(
            sys._getframe().f_code.co_name))
            return statusword, False

        # return statusword as an int type
        statusword = int.from_bytes(statusword, 'little')
        return statusword, True


    def writeControlWord(self, controlword):
        '''Send controlword to device

        Args:
            controlword: word to be sent.

        Returns:
            bool: a boolean if all went ok.
        '''
        # sending new controlword
        self.logger.debug('[{0}:{1}] Sending controlword Hex={2:#06X} Bin={2:#018b}'.format(
            self.__class__.__name__,
            sys._getframe().f_code.co_name,
            controlword))
        controlword = controlword.to_bytes(2, 'little')
        return self.writeObject(0x6040, 0, controlword)

    def readControlWord(self):
        index = self.objectIndex['ControlWord']
        subindex = 0
        controlword = self.readObject(index,subindex)
        # failded to request?
        if not controlword:
            logging.info("[EPOS:{0}] Error trying to read EPOS controlword".format(
            sys._getframe().f_code.co_name))
            return controlword, False

        # return controlword as an int type
        controlword = int.from_bytes(controlword, 'little')
        return controlword, True


    def readExtControlWord(self):
        try:
            controlword = self.node.sdo['Controlword 1'].raw
            return controlword, True

        except canopen.SdoAbortedError as e:
            text = "Code 0x{:08X}".format(e.code)
            if e.code in self.errorIndex:
                text = text + ", " + self.errorIndex[e.code]
            self.logger.info('[{0}:{1}] SdoAbortedError: '.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name) + text)
            return None, False
        except canopen.SdoCommunicationError:
            self.logger.info('[{0}:{1}] SdoAbortedError: Timeout or unexpected response'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return None, False

    def writeExtControlWord(self, extControlWord):
        '''Send extControlword to device

        Args:
            extControlword: word to be sent.

        Returns:
            bool: a boolean if all went ok.
        '''
        # sending new extControlWord
        self.logger.debug('[{0}:{1}] Sending controlword Hex={2:#06X} Bin={2:#018b}'.format(
            self.__class__.__name__,
            sys._getframe().f_code.co_name,
            extControlWord))
        extControlWord = extControlWord.to_bytes(2, 'little')
        return self.writeObject(0x2037, 0x5, extControlWord)

    def changeState(self, newState):
        '''Change SINAMICS state

        Change SINAMICS state using controlWord object

        To change SINAMICS state, a write to controlWord object is made.
        The bit change in controlWord is made as shown in the following table:

        +-------------------+--------------------------------+
        | State             | LowByte of Controlword [binary]|
        +===================+================================+
        | shutdown          | 0xxx x110                      |
        +-------------------+--------------------------------+
        | switch on         | 0xxx x111                      |
        +-------------------+--------------------------------+
        | disable voltage   | 0xxx xx0x                      |
        +-------------------+--------------------------------+
        | quick stop        | 0xxx x01x                      |
        +-------------------+--------------------------------+
        | disable operation | 0xxx 0111                      |
        +-------------------+--------------------------------+
        | enable operation  | 0xxx 1111                      |
        +-------------------+--------------------------------+
        | fault reset       | 1xxx xxxx                      |
        +-------------------+--------------------------------+

        see section 8.1.3 of firmware for more information

        Args:
            newState: string with state witch user want to switch.

        Returns:
            bool: boolean if all went ok and no error was received.
        '''
        stateOrder = ['shutdown', 'switch on', 'disable voltage', 'quick stop',
                      'disable operation', 'enable operation', 'fault reset']

        if not (newState in stateOrder):
            logging.info('[{0}:{1}] Unkown state: {2}'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name,
                newState))
            return False
        else:
            controlword, Ok = self.readControlWord()
            if not Ok:
                logging.info('[{0}:{1}] Failed to retreive controlword'.format(
                    self.__class__.__name__,
                    sys._getframe().f_code.co_name))
                return False
            # shutdown  0xxx x110
            if newState == 'shutdown':
                # clear bits
                mask = not ( 1<<7 | 1<<0 )
                controlword = controlword & mask
                # set bits
                mask  = ( 1<< 2 | 1<<1 )
                controlword = controlword | mask
                return self.writeControlWord(controlword)
            # switch on 0xxx x111
            if newState == 'switch on':
                # clear bits
                mask = not ( 1<<7 )
                controlword = controlword & mask
                # set bits
                mask  = ( 1<< 2 | 1<<1 | 1<<0 )
                controlword = controlword | mask
                return self.writeControlWord(controlword)
            # disable voltage 0xxx xx0x
            if newState == 'switch on':
                # clear bits
                mask = not ( 1<<7 | 1 << 1 )
                controlword = controlword & mask
                return self.writeControlWord(controlword)
            # quick stop 0xxx x01x
            if newState == 'quick stop':
                # clear bits
                mask = not ( 1<<7 | 1 << 2)
                controlword = controlword & mask
                # set bits
                mask  = ( 1<<1 )
                controlword = controlword | mask
                return self.writeControlWord(controlword)
            # disable operation 0xxx 0111
            if newState == 'disable operation':
                # clear bits
                mask = not ( 1<<7 | 1 << 3)
                controlword = controlword & mask
                # set bits
                mask  = ( 1<<2 | 1<< 1 | 1<<0 )
                controlword = controlword | mask
                return self.writeControlWord(controlword)
            # enable operation x01x 0111
            if newState == 'enable operation':
                # clear bits
                mask = not ( 1<<6 | 1<<3 )
                controlword = controlword & mask
                # set bits
                mask  = ( 1<< 5 | 1 << 2 | 1 << 1 | 1 << 0 )
                controlword = controlword | mask
                return self.writeControlWord(controlword)
            # fault reset 1xxx xxxx
            if newState == 'fault reset':
                # set bits
                mask  = ( 1<<7 )
                controlword = controlword | mask
                return self.writeControlWord(controlword)


    def checkState(self):
        '''Check current state of SINAMICS

        Ask the StatusWord of SINAMICS and parse it to return the current state of SINAMICS.

        +----------------------------------+-----+---------------------+
        | State                            | ID  | Statusword [binary] |
        +==================================+=====+=====================+
        | Start                            | 0   | x0xx xxx0  x000 0000|
        +----------------------------------+-----+---------------------+
        | Not Ready to Switch On           | 1   | xxxx xxxx  x0xx 0000|
        +----------------------------------+-----+---------------------+
        | Switch on disabled               | 2   | xxxx xxxx  x1xx 0000|
        +----------------------------------+-----+---------------------+
        | ready to switch on               | 3   | xxxx xxxx  x01x 0001|
        +----------------------------------+-----+---------------------+
        | switched on                      | 4   | xxxx xxxx  x01x 0011|
        +----------------------------------+-----+---------------------+
        | refresh                          | 5   | x1xx xxx1  x010 0011|
        +----------------------------------+-----+---------------------+
        | measure init                     | 6   | x1xx xxx1  x011 0011|
        +----------------------------------+-----+---------------------+
        | operation enable                 | 7   | xxxx xxxx  x01x 0111|
        +----------------------------------+-----+---------------------+
        | quick stop active                | 8   | xxxx xxxx  x00x 0111|
        +----------------------------------+-----+---------------------+
        | fault reaction active (disabled) | 9   | x0xx xxx1  x000 1111|
        +----------------------------------+-----+---------------------+
        | fault reaction active (enabled)  | 10  | x0xx xxx1  x001 1111|
        +----------------------------------+-----+---------------------+
        | Fault                            | 11  | xxxx xxxx  x0xx 1000|
        +----------------------------------+-----+---------------------+


        Returns:
            int: numeric identification of the state or -1 in case of fail.
        '''
        statusword, ok = self.readStatusWord()
        if not ok:
            self.logger.info('[{0}:{1}] Failed to request StatusWord\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
        else:

            # state 'start' (0)
            # statusWord == x0xx xxx0  x000 0000
            bitmask = 0b0100000101111111
            if(bitmask & statusword == 0):
                ID = 0
                return ID

        # state 'not ready to switch on' (1)
        # statusWord == xxxx xxxx  x0xx 0000
            bitmask = 0b0000000001001111
            if (bitmask & statusword == 0):
                ID = 1
                return ID

            # state 'switch on disabled' (2)
            # statusWord == xxxx xxxx  x1xx 0000
            bitmask = 0b0000000001001111
            if(bitmask & statusword == 64):
                ID = 2
                return ID

            # state 'ready to switch on' (3)
            # statusWord == xxxx xxxx  x01x 0001
            bitmask = 0b0000000001101111
            if(bitmask & statusword == 33):
                ID = 3
                return ID

            # state 'switched on' (4)
            # statusWord == x0xx xxx1  x010 0011
            bitmask = 0b0000000101111111
            if(bitmask & statusword == 291):
                ID = 4
                return ID

            # state 'refresh' (5)
            # statusWord == x1xx xxx1  x010 0011
            bitmask = 0b0100000101111111
            if(bitmask & statusword == 16675):
                ID = 5
                return ID

            # state 'measure init' (6)
            # statusWord == x1xx xxx1  x011 0011
            bitmask = 0b0100000101111111
            if(bitmask & statusword == 16691):
                ID = 6
                return ID
            # state 'operation enable' (7)
            # statusWord == x0xx xxx1  x011 0111
            bitmask = 0b0100000101111111
            if(bitmask & statusword == 311):
                ID = 7
                return ID

            # state 'Quick Stop Active' (8)
            # statusWord == x0xx xxx1  x001 0111
            bitmask = 0b0100000101111111
            if(bitmask & statusword == 279):
                ID = 8
                return ID

            # state 'fault reaction active (disabled)' (9)
            # statusWord == x0xx xxx1  x000 1111
            bitmask = 0b0100000101111111
            if(bitmask & statusword == 271):
                ID = 9
                return ID

            # state 'fault reaction active (enabled)' (10)
            # statusWord == x0xx xxx1  x001 1111
            bitmask = 0b0100000101111111
            if(bitmask & statusword == 287):
                ID = 10
                return ID

            # state 'fault' (11)
            # statusWord == x0xx xxx1  x000 1000
            bitmask = 0b0100000101111111
            if(bitmask & statusword == 264):
                ID = 11
                return ID

        # in case of unknown state or fail
        print('[{0}:{1}] Error: Unknown state\nStatusword is Bin={2:#018b}'.format(
            self.__class__.__name__,
            sys._getframe().f_code.co_name,
            int.from_bytes(statusword, 'little'))
        )
        return -1

    def printSINAMICSState (self):
        ID = self.checkState()
        if ID is -1:
            print('[{0}:{1}] Error: Unknown state\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
        else:
            print('[{0}:{1}] Current state [ID]:{2} [{3}]\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name,
                self.state[ID],
                ID))
        return



    def printStatusWord(self):
        """ Print meaning of status word.

        See https://w5.siemens.com/web/cz/cz/corporate/portal/home/produkty_a_sluzby/IBT/mereni_a_regulace/frekvencni_menice/Documents/014_Parameter_Manual_CU230P_V441_en.pdf
        page 30 for meaning of each bit value.
        """
        statusword, Ok = self.readStatusWord()
        if not Ok:
            print('[{0}:{1}] Failed to retreive statusword\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return
        else:
            print("[{0}:{1}] The statusword is Hex={2:#06X} Bin={2:#018b}\n".format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name,
                statusword))
            print('Bit 15: Alarm drive converter overload (1=No, 0=Yes):          {0}'.format((statusword & (1 << 15))>>15))
            print('Bit 14: Motor rotates forwards:                                {0}'.format((statusword & (1 << 14))>>14))
            print('Bit 13: Reserved:                                              {0}'.format((statusword & (1 << 13))>>13))
            print('Bit 12: velocity equal to zero:                                {0}'.format((statusword & (1 << 12))>>12))
            print('Bit 11: I, M, P limit reached:                                 {0}'.format((statusword & (1 << 11))>>11))
            print('Bit 10: Target reached:                                        {0}'.format((statusword & (1 << 10))>>10))
            print('Bit 09: Control Request:                                       {0}'.format((statusword & (1 << 9 ))>>9))
            print('Bit 08: Deviation, setpoint/actual speed(1=No, 0=Yes):         {0}'.format((statusword & (1 << 8 ))>>8))
            print('Bit 07: Alarm:                                                 {0}'.format((statusword & (1 << 7 ))>>7))
            print('Bit 06: Switch on disable:                                     {0}'.format((statusword & (1 << 6 ))>>6))
            print('Bit 05: No Quick stop (OFF3):                                  {0}'.format((statusword & (1 << 5 ))>>5))
            print('Bit 04: No Coast down active (OFF2):                           {0}'.format((statusword & (1 << 4 ))>>4))
            print('Bit 03: Fault:                                                 {0}'.format((statusword & (1 << 3 ))>>3))
            print('Bit 02: Operation enable:                                      {0}'.format((statusword & (1 << 2 ))>>2))
            print('Bit 01: Ready:                                                 {0}'.format((statusword & (1 << 1 ))>>1))
            print('Bit 00: Ready to switch on:                                    {0}'.format(statusword & 1))
        return

    # def printAllStatusWords(self):
    #     '''Print all statuswords

    #     Request all statuswords and print the meaning of them
    #     '''
    #     print('------------------------------------------------------------------')
    #     self.printStatusWord()
    #     print('------------------------------------------------------------------')
    #     index = 0x2002
    #     subindex = [0x7, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A]
    #     statusword = []
    #     for subId in subindex:
    #         statusword.append(self.readObject(index, subId))
    #     # show ext statusword 0
    #     if not statusword[0]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 0))
    #     else:
    #         statusword[0] = int.from_bytes(statusword[0], 'little')
    #         cmd_origin = ['Terminals','HMI','ModBus','CANopen']
    #         origin = (statusword & (1 << 14))>>14 + (statusword & (1 << 13))>>13
    #         print("[{0}:{1}] The Extended statusword 0 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[0]))
    #         print('Bit 15: Operation before ramp [0 Foward | 1 Reverse]:          {0}'.format((statusword[0] & (1 << 15))>>15))
    #         print('Bit 14: bit 13 and 14 describes  origin of command:            {0}'.format((statusword[0] & (1 << 14))>>14))
    #         print('Bit 13: bit 13 and 14 describes  origin of command:            {0}'.format((statusword[0] & (1 << 13))>>13))
    #         print('Bit 12: Fast Stop:                                             {0}'.format((statusword[0] & (1 << 12))>>12))
    #         print('Bit 11: Current or torque limit active:                        {0}'.format((statusword[0] & (1 << 11))>>11))
    #         print('Bit 10: Deceleration in progress:                              {0}'.format((statusword[0] & (1 << 10))>>10))
    #         print('Bit 09: Accelereation in progress:                             {0}'.format((statusword[0] & (1 << 9 ))>>9))
    #         print('Bit 08: Overbraking:                                           {0}'.format((statusword[0] & (1 << 8 ))>>8))
    #         print('Bit 07: Motor Thermal:                                         {0}'.format((statusword[0] & (1 << 7 ))>>7))
    #         print('Bit 06: [0 Steady sate | 1 Transient state]:                   {0}'.format((statusword[0] & (1 << 6 ))>>6))
    #         print('Bit 05: DC injection breaking:                                 {0}'.format((statusword[0] & (1 << 5 ))>>5))
    #         print('Bit 04: Power supply present ([0 no Run | 1 Run]):             {0}'.format((statusword[0] & (1 << 4 ))>>4))
    #         print('Bit 03: Reserved:                                              {0}'.format((statusword[0] & (1 << 3 ))>>3))
    #         print('Bit 02: see description                                        {0}'.format((statusword[0] & (1 << 2 ))>>2))
    #         print('Bit 01: Parameter consistency checked:                         {0}'.format((statusword[0] & (1 << 1 ))>>1))
    #         print('Bit 00: Access to the EEPROM non-volatile memory in progress:  {0}'.format(statusword[0] & 1))
    #         print('------------------------------------------------------------------')
    #         print('Additional information')
    #         print('------------------------------------------------------------------')
    #         print('Origin of command: {0}'.format(cmd_origin[origin]))
    #         print('------------------------------------------------------------------')
    #         print('Bit 02: if 0 The drive is not in fault state or a fault is present')
    #         print('if 1 Drive is in fault state but the fault is no longer present')

    #     print('------------------------------------------------------------------')
    #     # show ext statusword 1
    #     if not statusword[1]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 1))
    #     else:
    #         statusword[1] = int.from_bytes(statusword[1], 'little')
    #         print("[{0}:{1}] The Extended statusword 1 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[1]))
    #         print('Bit 15: The "traverse control" function is active:             {0}'.format((statusword[1] & (1 << 15))>>15))
    #         print('Bit 14: Drive thermal state threshold reached:                 {0}'.format((statusword[1] & (1 << 14))>>14))
    #         print('Bit 13: Second frequency threshold reached:                    {0}'.format((statusword[1] & (1 << 13))>>13))
    #         print('Bit 12: 4-20 mA alarm on analog input AI2:                     {0}'.format((statusword[1] & (1 << 12))>>12))
    #         print('Bit 11: PID regulator feedback alarm:                          {0}'.format((statusword[1] & (1 << 11))>>11))
    #         print('Bit 10: PID regulator error alarm:                             {0}'.format((statusword[1] & (1 << 10))>>10))
    #         print('Bit 09: Brake contactor command active:                        {0}'.format((statusword[1] & (1 << 9 ))>>9))
    #         print('Bit 08: Motor 1 thermal state threshold reached:               {0}'.format((statusword[1] & (1 << 8 ))>>8))
    #         print('Bit 07: Frequency reference reached:                           {0}'.format((statusword[1] & (1 << 7 ))>>7))
    #         print('Bit 06: Current threshold reached:                             {0}'.format((statusword[1] & (1 << 6 ))>>6))
    #         print('Bit 05: High speed reached:                                    {0}'.format((statusword[1] & (1 << 5 ))>>5))
    #         print('Bit 04: Freq. Threashold reached:                              {0}'.format((statusword[1] & (1 << 4 ))>>4))
    #         print('Bit 03: The output contactor is controlled:                    {0}'.format((statusword[1] & (1 << 3 ))>>3))
    #         print('Bit 02: [0 Drive locked | 1 Unlocked]:                         {0}'.format((statusword[1] & (1 << 2 ))>>2))
    #         print('Bit 01: Drive is on Fault state:                               {0}'.format((statusword[1] & (1 << 1 ))>>1))
    #         print('Bit 00: Reserved:                                              {0}'.format(statusword[1] & 1))

    #     print('------------------------------------------------------------------')
    #     # show ext statusword 2
    #     if not statusword[2]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 2))
    #     else:
    #         statusword[2] = int.from_bytes(statusword[2], 'little')
    #         print("[{0}:{1}] The Extended statusword 2 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[2]))
    #         print('Bit 15: Reserved:                                              {0}'.format((statusword[2] & (1 << 15))>>15))
    #         print('Bit 14: Reserved:                                              {0}'.format((statusword[2] & (1 << 14))>>14))
    #         print('Bit 13: Reserved:                                              {0}'.format((statusword[2] & (1 << 13))>>13))
    #         print('Bit 12: Reserved:                                              {0}'.format((statusword[2] & (1 << 12))>>12))
    #         print('Bit 11: Reserved:                                              {0}'.format((statusword[2] & (1 << 11))>>11))
    #         print('Bit 10: Reserved:                                              {0}'.format((statusword[2] & (1 << 10))>>10))
    #         print('Bit 09: Reserved:                                              {0}'.format((statusword[2] & (1 << 9 ))>>9))
    #         print('Bit 08: Reserved:                                              {0}'.format((statusword[2] & (1 << 8 ))>>8))
    #         print('Bit 07: Reserved:                                              {0}'.format((statusword[2] & (1 << 7 ))>>7))
    #         print('Bit 06: Reserved:                                              {0}'.format((statusword[2] & (1 << 6 ))>>6))
    #         print('Bit 05: Reserved:                                              {0}'.format((statusword[2] & (1 << 5 ))>>5))
    #         print('Bit 04: Reserved:                                              {0}'.format((statusword[2] & (1 << 4 ))>>4))
    #         print('Bit 03: Reserved:                                              {0}'.format((statusword[2] & (1 << 3 ))>>3))
    #         print('Bit 02: Reserved:                                              {0}'.format((statusword[2] & (1 << 2 ))>>2))
    #         print('Bit 01: Reserved:                                              {0}'.format((statusword[2] & (1 << 1 ))>>1))
    #         print('Bit 00: Reserved:                                              {0}'.format(statusword[2] & 1))

    #     print('------------------------------------------------------------------')
    #     # show ext statusword 3
    #     if not statusword[3]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 3))
    #     else:
    #         statusword[3] = int.from_bytes(statusword[3], 'little')
    #         print("[{0}:{1}] The Extended statusword 3 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[3]))
    #         print('Bit 15: Output torque [0 positive | 1 negative]:               {0}'.format((statusword[3] & (1 << 15))>>15))
    #         print('Bit 14: Reserved:                                              {0}'.format((statusword[3] & (1 << 14))>>14))
    #         print('Bit 13: Reserved:                                              {0}'.format((statusword[3] & (1 << 13))>>13))
    #         print('Bit 12: Reserved:                                              {0}'.format((statusword[3] & (1 << 12))>>12))
    #         print('Bit 11: Reserved:                                              {0}'.format((statusword[3] & (1 << 11))>>11))
    #         print('Bit 10: Reserved:                                              {0}'.format((statusword[3] & (1 << 10))>>10))
    #         print('Bit 09: Stop on low speed time limit function:                 {0}'.format((statusword[3] & (1 << 9 ))>>9))
    #         print('Bit 08: 24 VDC external power supply present:                  {0}'.format((statusword[3] & (1 << 8 ))>>8))
    #         print('Bit 07: Reserved:                                              {0}'.format((statusword[3] & (1 << 7 ))>>7))
    #         print('Bit 06: Motor 3 thermal state threshold reached:               {0}'.format((statusword[3] & (1 << 6 ))>>6))
    #         print('Bit 05: Motor 2 thermal state threshold reached:               {0}'.format((statusword[3] & (1 << 5 ))>>5))
    #         print('Bit 04: Reserved:                                              {0}'.format((statusword[3] & (1 << 4 ))>>4))
    #         print('Bit 03: [0 Current limit 1 active | 1 Current limit 2 active ]:{0}'.format((statusword[3] & (1 << 3 ))>>3))
    #         print('Bit 02: [0 Ramp set 1 | 1 Ramp set 2 ]:                        {0}'.format((statusword[3] & (1 << 2 ))>>2))
    #         print('Bit 01: [0 Com. act. channel 1 | 1 Com. act. channel 2 ]:      {0}'.format((statusword[3] & (1 << 1 ))>>1))
    #         print('Bit 00: [0 Ref. act. channel 1(b) | 1 Ref. act. channel 2 ]:   {0}'.format(statusword[3] & 1))
    #     print('------------------------------------------------------------------')
    #     # show ext statusword 4
    #     if not statusword[4]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 4))
    #     else:
    #         statusword[4] = int.from_bytes(statusword[4], 'little')
    #         print("[{0}:{1}] The Extended statusword 4 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[4]))
    #         print('Bit 15: Fast stop in progress:                                 {0}'.format((statusword[4] & (1 << 15))>>15))
    #         print('Bit 14: Deceleration in progress:                              {0}'.format((statusword[4] & (1 << 14))>>14))
    #         print('Bit 13: Acceleration in progress:                              {0}'.format((statusword[4] & (1 << 13))>>13))
    #         print('Bit 12: Current limiting in progress:                          {0}'.format((statusword[4] & (1 << 12))>>12))
    #         print('Bit 11: DC injection braking:                                  {0}'.format((statusword[4] & (1 << 11))>>11))
    #         print('Bit 10: The motor is "fluxed":                                 {0}'.format((statusword[4] & (1 << 10))>>10))
    #         print('Bit 09: Motor "fluxing" in progress:                           {0}'.format((statusword[4] & (1 << 9 ))>>9))
    #         print('Bit 08: Power section [0 present | 1 absent]                   {0}'.format((statusword[4] & (1 << 8 ))>>8))
    #         print('Bit 07: Reserved:                                              {0}'.format((statusword[4] & (1 << 7 ))>>7))
    #         print('Bit 06: Parameter set 3 is active:                             {0}'.format((statusword[4] & (1 << 6 ))>>6))
    #         print('Bit 05: Parameter set 2 is active:                             {0}'.format((statusword[4] & (1 << 5 ))>>5))
    #         print('Bit 04: Parameter set 1 is active:                             {0}'.format((statusword[4] & (1 << 4 ))>>4))
    #         print('Bit 03: Reserved:                                              {0}'.format((statusword[4] & (1 << 3 ))>>3))
    #         print('Bit 02: Configuration 2 is active:                             {0}'.format((statusword[4] & (1 << 2 ))>>2))
    #         print('Bit 01: Configuration 1 is active:                             {0}'.format((statusword[4] & (1 << 1 ))>>1))
    #         print('Bit 00: Configuration 0 is active:                             {0}'.format(statusword[4] & 1))
    #     print('------------------------------------------------------------------')
    #     # show ext statusword 5
    #     if not statusword[5]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 5))
    #     else:
    #         statusword[5] = int.from_bytes(statusword[5], 'little')
    #         print("[{0}:{1}] The Extended statusword 5 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[5]))
    #         print('Bit 15: Reserved:                                              {0}'.format((statusword[5] & (1 << 15))>>15))
    #         print('Bit 14: limit stops are reached:                               {0}'.format((statusword[5] & (1 << 14))>>14))
    #         print('Bit 13: Current present in the motor:                          {0}'.format((statusword[5] & (1 << 13))>>13))
    #         print('Bit 12: Reserved:                                              {0}'.format((statusword[5] & (1 << 12))>>12))
    #         print('Bit 11: Reserved:                                              {0}'.format((statusword[5] & (1 << 11))>>11))
    #         print('Bit 10: Reserved:                                              {0}'.format((statusword[5] & (1 << 10))>>10))
    #         print('Bit 09: The line contactor is active:                          {0}'.format((statusword[5] & (1 << 9 ))>>9))
    #         print('Bit 08: Reserved:                                              {0}'.format((statusword[5] & (1 << 8 ))>>8))
    #         print('Bit 07: Controlled output cut in progress:                     {0}'.format((statusword[5] & (1 << 7 ))>>7))
    #         print('Bit 06: The drive cannot follow the config. deceleration ramp: {0}'.format((statusword[5] & (1 << 6 ))>>6))
    #         print('Bit 05: Controlled stop in progress:                           {0}'.format((statusword[5] & (1 << 5 ))>>5))
    #         print('Bit 04: Auto-tuning in progress:                               {0}'.format((statusword[5] & (1 << 4 ))>>4))
    #         print('Bit 03: Automatic restart attempts in progress:                {0}'.format((statusword[5] & (1 << 3 ))>>3))
    #         print('Bit 02: The "Power removal" function is active                 {0}'.format((statusword[5] & (1 << 2 ))>>2))
    #         print('Bit 01: Drive braking:                                         {0}'.format((statusword[5] & (1 << 1 ))>>1))
    #         print('Bit 00: Drive DC bus loading:                                  {0}'.format(statusword[5] & 1))
    #     print('------------------------------------------------------------------')
    #     # show ext statusword 6
    #     if not statusword[6]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 6))
    #     else:
    #         statusword[6] = int.from_bytes(statusword[6], 'little')
    #         print("[{0}:{1}] The Extended statusword 6 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[6]))
    #         print('Bit 15: Current or torque limit alarm after time-out:          {0}'.format((statusword[6] & (1 << 15))>>15))
    #         print('Bit 14: Brake contact alarm in the brake control sequence:     {0}'.format((statusword[6] & (1 << 14))>>14))
    #         print('Bit 13: Speed alarm in the brake control sequence:             {0}'.format((statusword[6] & (1 << 13))>>13))
    #         print('Bit 12: Reserved:                                              {0}'.format((statusword[6] & (1 << 12))>>12))
    #         print('Bit 11: Drive overheat alarm:                                  {0}'.format((statusword[6] & (1 << 11))>>11))
    #         print('Bit 10: Slipping alarm:                                        {0}'.format((statusword[6] & (1 << 10))>>10))
    #         print('Bit 09: Line supply loss detection:                            {0}'.format((statusword[6] & (1 << 9 ))>>9))
    #         print('Bit 08: Undervoltage alarm:                                    {0}'.format((statusword[6] & (1 << 8 ))>>8))
    #         print('Bit 07: External Fault:                                        {0}'.format((statusword[6] & (1 << 7 ))>>7))
    #         print('Bit 06: Reserved:                                              {0}'.format((statusword[6] & (1 << 6 ))>>6))
    #         print('Bit 05: LI6 PTC probe alarm:                                   {0}'.format((statusword[6] & (1 << 5 ))>>5))
    #         print('Bit 04: PTC Probe alarm 2:                                     {0}'.format((statusword[6] & (1 << 4 ))>>4))
    #         print('Bit 03: PTC Probe alarm 1:                                     {0}'.format((statusword[6] & (1 << 3 ))>>3))
    #         print('Bit 02: Alarm group 3:                                         {0}'.format((statusword[6] & (1 << 2 ))>>2))
    #         print('Bit 01: Alarm group 2:                                         {0}'.format((statusword[6] & (1 << 1 ))>>1))
    #         print('Bit 00: Alarm group 1:                                         {0}'.format(statusword[6] & 1))
    #     print('------------------------------------------------------------------')
    #     # show ext statusword 7
    #     if not statusword[7]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 7))
    #     else:
    #         statusword[7] = int.from_bytes(statusword[7], 'little')
    #         print("[{0}:{1}] The Extended statusword 7 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[7]))
    #         print('Bit 15: Reserved:                                              {0}'.format((statusword[7] & (1 << 15))>>15))
    #         print('Bit 14: Reserved:                                              {0}'.format((statusword[7] & (1 << 14))>>14))
    #         print('Bit 13: DC bus precharging contactor controlled:               {0}'.format((statusword[7] & (1 << 13))>>13))
    #         print('Bit 12: 4-20 mA alarm on analog input AI4:                     {0}'.format((statusword[7] & (1 << 12))>>12))
    #         print('Bit 11: 4-20 mA alarm on analog input AI3:                     {0}'.format((statusword[7] & (1 << 11))>>11))
    #         print('Bit 10: Alarm sent by the "Controller Inside" card:            {0}'.format((statusword[7] & (1 << 10))>>10))
    #         print('Bit 09: Braking resistor overload alarm:                       {0}'.format((statusword[7] & (1 << 9 ))>>9))
    #         print('Bit 08: IGBT thermal state alarm:                              {0}'.format((statusword[7] & (1 << 8 ))>>8))
    #         print('Bit 07: Torque regulation alarm:                               {0}'.format((statusword[7] & (1 << 7 ))>>7))
    #         print('Bit 06: Master-slave synchronization:                          {0}'.format((statusword[7] & (1 << 6 ))>>6))
    #         print('Bit 05: Spool end:                                             {0}'.format((statusword[7] & (1 << 5 ))>>5))
    #         print('Bit 04: Reference channel 1B is active:                        {0}'.format((statusword[7] & (1 << 4 ))>>4))
    #         print('Bit 03: Command channel 2 is active:                           {0}'.format((statusword[7] & (1 << 3 ))>>3))
    #         print('Bit 02: Command channel 1 is active:                           {0}'.format((statusword[7] & (1 << 2 ))>>2))
    #         print('Bit 01:  Reference channel 2 is active                         {0}'.format((statusword[7] & (1 << 1 ))>>1))
    #         print('Bit 00:  Reference channel 1 or 1b is active                   {0}'.format(statusword[7] & 1))
    #     print('------------------------------------------------------------------')
    #     # show ext statusword 8
    #     if not statusword[8]:
    #         print('[{0}:{1}] Failed to retreive Extended statusword {2}'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name, 8))
    #     else:
    #         statusword[8] = int.from_bytes(statusword[8], 'little')
    #         print("[{0}:{1}] The Extended statusword 8 is Hex={2:#06X} Bin={2:#018b}\n".format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name,
    #             statusword[8]))
    #         print('Bit 15: Driver ready:                                          {0}'.format((statusword[8] & (1 << 15))>>15))
    #         print('Bit 14: Reserved:                                              {0}'.format((statusword[8] & (1 << 14))>>14))
    #         print('Bit 13: Reserved:                                              {0}'.format((statusword[8] & (1 << 13))>>13))
    #         print('Bit 12: Reserved:                                              {0}'.format((statusword[8] & (1 << 12))>>12))
    #         print('Bit 11: Reserved:                                              {0}'.format((statusword[8] & (1 << 11))>>11))
    #         print('Bit 10: Reserved:                                              {0}'.format((statusword[8] & (1 << 10))>>10))
    #         print('Bit 09: Reserved:                                              {0}'.format((statusword[8] & (1 << 9 ))>>9))
    #         print('Bit 08: Reserved:                                              {0}'.format((statusword[8] & (1 << 8 ))>>8))
    #         print('Bit 07: Reserved:                                              {0}'.format((statusword[8] & (1 << 7 ))>>7))
    #         print('Bit 06: Reserved:                                              {0}'.format((statusword[8] & (1 << 6 ))>>6))
    #         print('Bit 05: Reserved:                                              {0}'.format((statusword[8] & (1 << 5 ))>>5))
    #         print('Bit 04: Reserved:                                              {0}'.format((statusword[8] & (1 << 4 ))>>4))
    #         print('Bit 03: Reserved:                                              {0}'.format((statusword[8] & (1 << 3 ))>>3))
    #         print('Bit 02: Reserved:                                              {0}'.format((statusword[8] & (1 << 2 ))>>2))
    #         print('Bit 01: Reserved:                                              {0}'.format((statusword[8] & (1 << 1 ))>>1))
    #         print('Bit 00: Reserved:                                              {0}'.format(statusword[8] & 1))
    #     print('------------------------------------------------------------------')
    #     return

    def printControlWord(self, controlword=None):
        '''Print the meaning of controlword

        Check the meaning of current controlword of device or check the meaning of your own controlword.
        Usefull to check your own controlword before actually sending it to device.

        Args:
            controlword (optional): If None, request the controlword of device.

        '''
        if not controlword:
            controlword, Ok = self.readControlWord()
            if not Ok:
                print('[{0}:{1}] Failed to retreive controlword\n'.format(
                    self.__class__.__name__,
                    sys._getframe().f_code.co_name))
                return
        print("[{0}:{1}] The controlword is Hex={2:#06X} Bin={2:#018b}\n".format(
            self.__class__.__name__,
            sys._getframe().f_code.co_name,
            controlword))
        print('Bit 15: CDS bit 0:                                      {0}'.format((controlword & (1 << 15 ))>>15))
        print('Bit 14: Motorized potentiometer lower:                  {0}'.format((controlword & (1 << 14 ))>>14))
        print('Bit 13: Motorized potentiometer raise:                  {0}'.format((controlword & (1 << 13 ))>>13))
        print('Bit 12: Can be assigned to command:                     {0}'.format((controlword & (1 << 12 ))>>12))
        print('Bit 11: Direction of rotation [0 Foward | 1 Reverse]:   {0}'.format((controlword & (1 << 11 ))>>11))
        print('Bit 10: Master ctrl by PLC:                             {0}'.format((controlword & (1 << 10 ))>>10))
        print('Bit 09: Jog bit 1:                                      {0}'.format((controlword & (1 << 9 ))>>9))
        print('Bit 08: Jog bit 0:                                      {0}'.format((controlword & (1 << 8 ))>>8))
        print('Bit 07: Fault reset:                                    {0}'.format((controlword & (1 << 7 ))>>7))
        print('Bit 06: Speed setpoint enable:                          {0}'.format((controlword & (1 << 6 ))>>6))
        print('Bit 05: Continue ramp-function generator:               {0}'.format((controlword & (1 << 5 ))>>5))
        print('Bit 04: Ramp-function generator enable:                 {0}'.format((controlword & (1 << 4 ))>>4))
        print('Bit 03: Enable operation:                               {0}'.format((controlword & (1 << 3 ))>>3))
        print('Bit 02: OC / OFF3 Do not activate quick stop            {0}'.format((controlword & (1 << 2 ))>>2))
        print('Bit 01: OC / OFF2 Do not activate coast:                {0}'.format((controlword & (1 << 1 ))>>1))
        print('Bit 00: ON/OFF1:                                        {0}'.format(controlword & 1))
        return

    # will be using dictionary object for now
    # def readMotorConfig(self):
    #     '''
    #     TODO
    #     '''
    #     # menu of motor config settings in display
    #     # create an empty dictionary
    #     motorConfig = []
    #     # convert from HP to kW
    #     HPtokW = 75
    #     # control type enum
    #     controlType = ('[SVC V]: Open-loop voltage flux vector control',
    #                    '[SVC I]: Open-loop current flux vector control',
    #                    '[FVC]: Closed-loop current flux vector control for motor with incremental encoder type sensor',
    #                    '[V/F 2pts]: Simple V/F profile without slip compensation',
    #                    '[V/F 5pts]: 5-segment V/F profile',
    #                    '[Sync. mot.]: For synchronous permanent magnet motors with sinusoidal electromotive force (EMF) only')
    #     try:
    #         # request standard motor frequency
    #         motorFreq = self.readObject(0x2000, 0x10)
    #         if motorFreq is None:
    #             self.logger.info('[{0}:{1}] Failed to request standar motor frequency'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name))
    #             return None, False
    #         if motorFreq == 1:
    #             motorConfig.append({'MotorFreq':'60Hz NEMA'})
    #         else:
    #             motorConfig.append({'MotorFreq':'50Hz IEC'})
    #         # request rated motor power
    #         aux = self.readObject(0x2042, 0xE)
    #         if aux is None:
    #             self.logger.info('[{0}:{1}] Failed to request rated motor power'.format(
    #             self.__class__.__name__,
    #             sys._getframe().f_code.co_name))
    #             return None, False
    #         aux = int.from_bytes(aux, 'little')
    #         # scale value is dependent on the motorFreq value
    #         # if motorFreq is 1, unit is  0.1 HP
    #         # if motorFreq is 0 and if power > 75 kW: Unit = 1 kW
    #         # If motorFreq is 0 and if power ≤ 75 kW: Unit = 0.01 kW
    #         if motorFreq == 1:
    #             motorConfig.append({'Rated motor power [HP]': aux * 0.1})
    #         else:
    #             motorConfig.append({'Rated motor power [HP]': (aux * 0.01) / HPtokW})
    #         # request nominal motor voltage
    #         aux = self.readObject(0x2042, 0x2)
    #         if aux is None:
    #             self.logger.info('[{0}:{1}] Failed to request rated motor voltage'.format(
    #                 self.__class__.__name__,
    #                 sys._getframe().f_code.co_name))
    #             return None, False
    #         aux = int.from_bytes(aux, 'little')
    #         motorConfig.append({'Rated motor voltage [V]': aux })
    #         # request nominal motor frequency
    #         aux = self.readObject(0x2042, 0x3)
    #         if aux is None:
    #             self.logger.info('[{0}:{1}] Failed to request rated motor frequency'.format(
    #                 self.__class__.__name__,
    #                 sys._getframe().f_code.co_name))
    #             return None, False
    #         aux = int.from_bytes(aux, 'little')
    #         motorConfig.append({'Rated motor frequency [Hz]': aux * 0.1 })
    #         # request nominal motor current
    #         aux = self.readObject(0x2042, 0x4)
    #         if aux is None:
    #             self.logger.info('[{0}:{1}] Failed to request rated motor current'.format(
    #                 self.__class__.__name__,
    #                 sys._getframe().f_code.co_name))
    #             return None, False
    #         aux = int.from_bytes(aux, 'little')
    #         motorConfig.append({'Rated motor current [A]': aux * 0.1 })

    #         # request nominal motor speed
    #         aux = self.readObject(0x2042, 0x5)
    #         if aux is None:
    #             self.logger.info('[{0}:{1}] Failed to request rated motor speed'.format(
    #                 self.__class__.__name__,
    #                 sys._getframe().f_code.co_name))
    #             return None, False
    #         aux = int.from_bytes(aux, 'little')
    #         motorConfig.append({'Rated motor speed [RPM]': aux })

    #         # request max. output frequency
    #         aux = self.readObject(0x2001, 0x4)
    #         if aux is None:
    #             self.logger.info('[{0}:{1}] Failed to request Max output frequency'.format(
    #                 self.__class__.__name__,
    #                 sys._getframe().f_code.co_name))
    #             return None, False
    #         aux = int.from_bytes(aux, 'little')
    #         motorConfig.append({'Max output frequency [Hz]': aux * 0.1 })

    #         # request motor thermal current

    #         # request Acceleration ramp time

    #         # request deceleration ramp time

    #         # request motor control type
    #         aux = self.readObject(0x2042, 0x8)
    #         if aux is None:
    #             self.logger.info('[{0}:{1}] Failed to request motor control type'.format(
    #                 self.__class__.__name__,
    #                 sys._getframe().f_code.co_name))
    #             return None, False
    #         aux = int.from_bytes(aux, 'little')
    #         motorConfig.append({'Motor control type': controlType[aux] })

    #         # request V/F points on 5 points
    #         vPoints = []
    #         fPoints = []
    #         swap = False
    #         for I in range(2,0x10):
    #             if I == 3:
    #                 # f0 is zero. do nothing
    #                 swap = False
    #             else:
    #                 aux = self.readObject(0x205E, I)
    #                 if aux is None:
    #                     self.logger.info('[{0}:{1}] Failed to V/F point at subindex:0x{2:02X}'.format(
    #                     self.__class__.__name__,
    #                     sys._getframe().f_code.co_name, I))
    #                 return None, False
    #                 if swap:
    #                     # if a freq value
    #                     aux = int.from_bytes(aux, 'little')
    #                     fPoints.append(aux * 0.1)
    #                     swap = False
    #                 else:
    #                     # if a voltage value
    #                     aux = int.from_bytes(aux, 'little')
    #                     vPoints.append(aux)
    #                     swap = True

    #         motorConfig.append({'V/F Voltage on 5 points': vPoints })
    #         motorConfig.append({'V/F Frequency on 5 points': fPoints })

    #         # request frequency in constant power

    #         # request drive switching frequency

    #         # request nominal motor slip

    #         # request pole pair number
    #         aux = self.readObject(0x2042, 0x49)
    #         if aux is None:
    #             self.logger.info('[{0}:{1}] Failed to request motor pole pairs'.format(
    #                 self.__class__.__name__,
    #                 sys._getframe().f_code.co_name))
    #             return None, False
    #         aux = int.from_bytes(aux, 'little')
    #         motorConfig.append({'Motor Pole Pairs': aux })
    #         # request custom stator resistence

    #         # request custom adjust mag. current

    #         # request custom leakage indutance

    #         # request custom rotor t constant

    #     except:
    #         pass
    #     return


def main():
    '''Test SINAMICS CANopen communication with some examples.

    Use a few examples to test communication with SINAMICS device using
    a few functions. Also resets the fault error if present.

    Show sample using also the EDS file.
    '''

    import argparse
    if (sys.version_info < (3, 0)):
        print("Please use python version 3")
        return

    parser = argparse.ArgumentParser(add_help=True,
                                     description='Test SINAMICS CANopen Communication')
    parser.add_argument('--channel', '-c', action='store', default='can0',
                        type=str, help='Channel to be used', dest='channel')
    parser.add_argument('--bus', '-b', action='store',
                        default='socketcan', type=str, help='Bus type', dest='bus')
    parser.add_argument('--rate', '-r', action='store', default=None,
                        type=int, help='bitrate, if applicable', dest='bitrate')
    parser.add_argument('--nodeID', action='store', default=2, type=int,
                        help='Node ID [ must be between 1- 127]', dest='nodeID')
    parser.add_argument('--objDict', action='store', default=None,
                        type=str, help='Object dictionary file', dest='objDict')
    args = parser.parse_args()


    # set up logging to file - see previous section for more details
    logging.basicConfig(level=logging.DEBUG,
                    format='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                    datefmt='%d-%m-%Y %H:%M:%S',
                    filename='atv.log',
                    filemode='w')
    # define a Handler which writes INFO messages or higher
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    # instanciate object
    inverter = SINAMICS()

    if not (inverter.begin(args.nodeID, objectDictionary=args.objDict)):
        logging.info('Failed to begin connection with SINAMICS device')
        logging.info('Exiting now')
        return

    # check if EDS file is supplied and print it
    if args.objDict:
        print('----------------------------------------------------------', flush=True)
        print('Printing EDS file entries')
        print('----------------------------------------------------------', flush=True)
        for obj in inverter.node.object_dictionary.values():
            print('0x%X: %s' % (obj.index, obj.name))
            if isinstance(obj, canopen.objectdictionary.Record):
                for subobj in obj.values():
                    print('  %d: %s' % (subobj.subindex, subobj.name))
        print('----------------------------------------------------------', flush=True)
        # test record a single record
        error_log = inverter.node.sdo['Error History']
        # Iterate over arrays or record
        for error in error_log.values():
            print("Error 0x%X was found in the log" % error.raw)

        print('----------------------------------------------------------', flush=True)

    # use simple hex values
    # try to read status word
    statusword = inverter.readObject(0x6041, 0)
    if not statusword:
        print("[SINAMICS] Error trying to read SINAMICS statusword\n")
        return
    else:
        print('----------------------------------------------------------', flush=True)
        print("The statusword is \n Hex={0:#06X} Bin={0:#018b}".format(
            int.from_bytes(statusword, 'little')))
    # try to read operation mode
    opMode = inverter.readObject(0x6060, 0)
    if not opMode:
        print("[SINAMICS] Error trying to read SINAMICS opMode\n")
        return
    else:
        print('----------------------------------------------------------', flush=True)
        print("The opMode is \n Hex={0:#06X} Bin={0:#018b}".format(
            int.from_bytes(opMode, 'little')))
    # test printStatusWord and state
    print('----------------------------------------------------------', flush=True)
    print('Testing print of StatusWord and State and ControlWord')
    print('----------------------------------------------------------', flush=True)
    inverter.printSINAMICSState()
    print('----------------------------------------------------------', flush=True)
    inverter.printStatusWord()
    print('----------------------------------------------------------', flush=True)
    inverter.printControlWord()
    print('----------------------------------------------------------', flush=True)
    return

if __name__ == '__main__':
    main()
