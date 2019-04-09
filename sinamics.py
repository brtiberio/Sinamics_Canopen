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
from time import sleep
import struct
from can import CanError  # TODO restudy need of this import since canopen already use it

# import pydevd

# pydevd.settrace('192.168.31.124', port=8000, stdoutToServer=True, stderrToServer=True)


class SINAMICS:
    network = None
    logger = None
    node = None
    _connected = False
    # If EDS file is present, this is not necessary, all codes can be gotten
    # from object dictionary.
    objectIndex = {  # control unit objects, independent of drive:
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
    errorIndex = {0x00000000: 'Error code: no error',
                  # 0x050x xxxx
                  0x05030000: 'Error code: toggle bit not alternated',
                  0x05040000: 'Error code: SDO protocol timeout',
                  0x05040001: 'Error code: Client/server command specifier not valid or unknown',
                  0x05040002: 'Error code: invalid block size',
                  0x05040003: 'Error code: invalid sequence number',
                  0x05040004: 'Error code: CRC error',
                  0x05040005: 'Error code: out of memory',
                  # 0x060x xxxx
                  0x06010000: 'Error code: Unsupported access to an object',
                  0x06010001: 'Error code: Attempt to read a write-only object',
                  0x06010002: 'Error code: Attempt to write a read-only object',
                  0x06020000: 'Error code: object does not exist',
                  0x06040041: 'Error code: object can not be mapped to the PDO',
                  0x06040042: 'Error code: the number and length of the objects to be mapped would exceed PDO length',
                  0x06040043: 'Error code: general parameter incompatibility',
                  0x06040047: 'Error code: general internal incompatibility in the device',
                  0x06060000: 'Error code: access failed due to an hardware error',
                  0x06070010: 'Error code: data type does not match, length of service parameter does not match',
                  0x06070012: 'Error code: data type does not match, length of service parameter too high',
                  0x06070013: 'Error code: data type does not match, length of service parameter too low',
                  0x06090011: 'Error code: subindex does not exist',
                  0x06090030: 'Error code: value range of parameter exceeded',
                  0x06090031: 'Error code: value of parameter written is too high',
                  0x06090032: 'Error code: value of parameter written is too low',
                  0x06090036: 'Error code: maximum value is less than minimum value',
                  0x060A0023: 'Error code: resource not available: SDO connection',
                  # 0x0800 xxxx
                  0x08000000: 'Error code: General error',
                  0x08000020: 'Error code: Data cannot be transferred or stored to the application',
                  0x08000021: 'Error code: Data cannot be transferred or stored to the application because of local control',
                  0x08000022: 'Error code: Wrong Device State. Data can not be transfered',
                  0x08000023: 'Error code: Object dictionary dynamic generation failed or no object dictionary present',
                  # Maxon defined error codes
                  0x0f00ffc0: 'Error code: wrong NMT state',
                  0x0f00ffbf: 'Error code: rs232 command illegeal',
                  0x0f00ffbe: 'Error code: password incorrect',
                  0x0f00ffbc: 'Error code: device not in service mode',
                  0x0f00ffB9: 'Error code: error in Node-ID'
                  }
    # dictionary sinamics fault number list
    sinamics_fault_number = {  # 1000 - 3999 - Control Unit
                             1000: 'Internal software error',
                             1002: 'Internal software error',
                             1004: 'Internal software error (N)',
                             1009: 'CU: Control module overtemperature',
                             1015: 'Internal software error',
                             1023: 'Software timeout (internal)',
                             1030: 'Sign-of-life failure for master control',
                             3506: '24 V power supply  missing',
                             # 5000 - 5999 - Power Section
                             5000: 'Power unit: Overtemperature heat sink AC inverter',
                             # 7000 - 7999 - Drive
                             7011: 'Drive: Motor overtemperature',
                             7012: 'Drive: Motor temperature model 1/3 overtemperature',
                             7013: 'Drive: Motor temperature model configuration fault',
                             7014: 'Drive: Motor temperature configuration alarm',
                             7015: 'Drive: Motor temperature sensor alarm',
                             7016: 'Drive: Motor temperature sensor fault',
                             7801: 'Drive: Motor overcurrent',
                             7802: 'Drive: Infeed or power unit not ready',
                             7910: 'Drive: Motor overtemperature',
                             # 30000 Drive CliQ
                             30001: 'Power unit: Overcurrent',
                             30002: 'Power unit: DC link voltage overvoltage',
                             30003: 'Power unit: DC link voltage undervoltage',
                             30004: 'Power unit: Overtemperature heat sink AC inverter',
                             30021: 'Power unit: Ground fault',
                             30024: 'Power unit: Overtemperature thermal model',
                             30034: 'Power unit: Internal overtemperature',
                             30036: 'Power unit: Internal overtemperature',
                             30045: 'Power unit: Supply undervoltage'}

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

    def log_info(self, message=None):
        """ Log a message

        A wrap around logging.
        The log message will have the following structure\:
        [class name \: function name ] message

        Args:
            message: a string with the message.
        """
        if message is None:
            # do nothing
            return
        self.logger.info('[{0}:{1}] {2}'.format(
            self.__class__.__name__,
            sys._getframe(1).f_code.co_name,
            message))
        return

    def log_debug(self, message=None):
        """ Log a message with debug level

        A wrap around logging.
        The log message will have the following structure\:
        [class name \: function name ] message

        the function name will be the caller function retrieved automatically
        by using sys._getframe(1).f_code.co_name

        Args:
            message: a string with the message.
        """
        if message is None:
            # do nothing
            return

        self.logger.debug('[{0}:{1}] {2}'.format(
            self.__class__.__name__,
            sys._getframe(1).f_code.co_name,
            message))
        return

    def begin(self, nodeID, _channel='can0', _bustype='socketcan', object_dictionary=None):
        """Initialize SINAMICS device

        Configure and setup SINAMICS device.

        Args:
            nodeID:    Node ID of the device.
            _channel (optional):   Port used for communication. Default can0
            _bustype (optional):   Port type used. Default socketcan.
            object_dictionary (optional):   Name of EDS file, if any available.
        Return:
            bool: A boolean if all went ok.
        """
        try:
            self.node = self.network.add_node(
                nodeID, object_dictionary=object_dictionary)
            self.network.connect(channel=_channel, bustype=_bustype)
            self._connected = True
            val, _ = self.read_statusword()  # test if we really have response or is only connected to CAN bus
            if val is None:
                self._connected = False
        except Exception as e:
            self.log_info("Exception caught:{0}".format(str(e)))
            self._connected = False
        finally:
            return self._connected

    def disconnect(self):
        self.network.disconect()
        self._connected = False
        return

    # --------------------------------------------------------------
    # Basic set of functions
    # --------------------------------------------------------------
    def read_object(self, index, subindex):
        """Reads an object

         Request a read from dictionary object referenced by index and subindex.

         Args:
             index:     reference of dictionary object index
             subindex:  reference of dictionary object subindex
         Returns:
             bytes:  message returned by SINAMICS or empty if unsucessfull
        """
        if self._connected:
            try:
                return self.node.sdo.upload(index, subindex)
            except Exception as e:
                self.log_info('Exception caught:{0}'.format(str(e)))
                return None
        else:
            self.log_info(' Error: {0} is not connected'.format(
                self.__class__.__name__))
            return None

    def write_object(self, index, subindex, data):
        """Write an object

         Request a write to dictionary object referenced by index and subindex.

         Args:
             index:     reference of dictionary object index
             subindex:  reference of dictionary object subindex
             data:      data to be stored
         Returns:
             bool:      boolean if all went ok or not
        """
        if self._connected:
            try:
                self.node.sdo.download(index, subindex, data)
                return True
            except canopen.SdoAbortedError as e:
                text = "Code 0x{:08X}".format(e.code)
                if e.code in self.errorIndex:
                    text = text + ", " + self.errorIndex[e.code]
                self.log_info('SdoAbortedError: ' + text)
                return False
            except canopen.SdoCommunicationError:
                self.log_info('SdoAbortedError: Timeout or unexpected response')
                return False
        else:
            self.log_info(' Error: {0} is not connected'.format(
                self.__class__.__name__))
            return False

    # ------------------------------------------------------------------------------
    # High level functions
    # ------------------------------------------------------------------------------
    def read_statusword(self):
        """Read statusword from device

        Returns:
            tuple: A tuple containing:

            :statusword:  the current value or None if any error.
            :Ok: A boolean if all went ok or not.
        """
        index = self.objectIndex['StatusWord']
        subindex = 0
        statusword = self.read_object(index, subindex)
        # failed to request?
        if not statusword:
            self.log_info('Error trying to read {0} statusword'.format(
                self.__class__.__name__))
            return statusword, False

        # return statusword as an int type
        statusword = int.from_bytes(statusword, 'little')
        return statusword, True

    def write_controlword(self, controlword):
        """Send controlword to device

        Args:
            controlword: word to be sent.

        Returns:
            bool: a boolean if all went ok.
        """
        # sending new controlword
        self.log_debug('Sending controlword Hex={0:#06X} Bin={0:#018b}'.format(controlword))
        controlword = controlword.to_bytes(2, 'little')
        return self.write_object(0x6040, 0, controlword)

    def read_controlword(self):
        """Read controlword from device

        Returns:
            tuple: A tuple containing:

            :controlword:  the current value or None if any error.
            :Ok: A boolean if all went ok or not.
        """
        index = self.objectIndex['ControlWord']
        subindex = 0
        controlword = self.read_object(index, subindex)
        # failed to request?
        if not controlword:
            self.log_info('Error trying to read {0} controlword'.format(
                self.__class__.__name__))
            return controlword, False

        # return controlword as an int type
        controlword = int.from_bytes(controlword, 'little')
        return controlword, True

    def change_state(self, new_state):
        """Change SINAMICS state

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
            new_state: string with state witch user want to switch.

        Returns:
            bool: boolean if all went ok and no error was received.
        """
        state_order = ['shutdown', 'switch on', 'disable voltage', 'quick stop',
                       'disable operation', 'enable operation', 'fault reset']

        if not (new_state in state_order):
            self.log_info('Unknown state: {0}'.format(new_state))
            return False
        else:
            controlword, ok = self.read_controlword()
            if not ok:
                self.log_info('Failed to retrieve controlword')
                return False
            # shutdown  0xxx x110
            if new_state == 'shutdown':
                # clear bits
                mask = ~ (1 << 7 | 1 << 0)
                controlword = controlword & mask
                # set bits
                mask = (1 << 2 | 1 << 1)
                controlword = controlword | mask
                return self.write_controlword(controlword)
            # switch on 0xxx x111
            if new_state == 'switch on':
                # clear bits
                mask = ~ (1 << 7)
                controlword = controlword & mask
                # set bits
                mask = (1 << 2 | 1 << 1 | 1 << 0)
                controlword = controlword | mask
                return self.write_controlword(controlword)
            # disable voltage 0xxx xx0x
            if new_state == 'switch on':
                # clear bits
                mask = ~ (1 << 7 | 1 << 1)
                controlword = controlword & mask
                return self.write_controlword(controlword)
            # quick stop 0xxx x01x
            if new_state == 'quick stop':
                # clear bits
                mask = ~ (1 << 7 | 1 << 2)
                controlword = controlword & mask
                # set bits
                mask = (1 << 1)
                controlword = controlword | mask
                return self.write_controlword(controlword)
            # disable operation 0xxx 0111
            if new_state == 'disable operation':
                # clear bits
                mask = ~ (1 << 7 | 1 << 3)
                controlword = controlword & mask
                # set bits
                mask = (1 << 2 | 1 << 1 | 1 << 0)
                controlword = controlword | mask
                return self.write_controlword(controlword)
            # enable operation 0xxx 1111
            if new_state == 'enable operation':
                # clear bits
                mask = ~ (1 << 7)
                controlword = controlword & mask
                # set bits
                mask = (1 << 3 | 1 << 2 | 1 << 1 | 1 << 0)
                controlword = controlword | mask
                return self.write_controlword(controlword)
            # fault reset 1xxx xxxx
            if new_state == 'fault reset':
                # set bits
                mask = (1 << 7)
                controlword = controlword | mask
                return self.write_controlword(controlword)

    def check_state(self, statusword=None):
        """Check current state of SINAMICS

        Ask the StatusWord of SINAMICS and parse it to return the current state
        of SINAMICS.

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

        Args:
            statusword: if not supplied, request statusword from device.
        Returns:
            int: numeric identification of the state or -1 in case of fail.
        """
        if not statusword:
            statusword, ok = self.read_statusword()
        else:
            ok = True
        if not ok:
            self.log_info('Failed to request StatusWord')
        else:

            # state 'start' (0)
            # statusWord == x0xx xxx0  x000 0000
            bitmask = 0b0100000101111111
            if (bitmask & statusword == 0):
                ID = 0
                return ID

            # state 'not ready to switch on' (1)
            # statusWord == xxxx xxxx  x0xx 0000
            bitmask = 0b0000000001001111
            if (bitmask & statusword == 0):
                ID = 1
                return ID

            # state 'switch on disabled' (2)
            # statusWord == xxxx xxxx  x1xx x000
            bitmask = 0b0000000001000111
            if (bitmask & statusword == 64):
                ID = 2
                return ID

            # state 'ready to switch on' (3)
            # statusWord == xxxx xxxx  x011 0001
            bitmask = 0b0000000001111111
            if (bitmask & statusword == 49):
                ID = 3
                return ID

            # state 'switched on' (4)
            # statusWord == xxxx xxxx  x011 0011
            bitmask = 0b0000000001111111
            if (bitmask & statusword == 51):
                ID = 4
                return ID

            # state 'refresh' (5)
            # statusWord == x1xx xxx1  x010 0011
            bitmask = 0b0100000101111111
            if (bitmask & statusword == 16675):
                ID = 5
                return ID

            # state 'measure init' (6)
            # statusWord == x1xx xxx1  x011 0011
            bitmask = 0b0100000101111111
            if (bitmask & statusword == 16691):
                ID = 6
                return ID
            # state 'operation enable' (7)
            # statusWord == xxxx xxxx  x011 0111
            bitmask = 0b0000000001111111
            if (bitmask & statusword == 55):
                ID = 7
                return ID

            # state 'Quick Stop Active' (8)
            # statusWord == x0xx xxx1  x001 0111
            bitmask = 0b0100000101111111
            if (bitmask & statusword == 279):
                ID = 8
                return ID

            # state 'fault reaction active (disabled)' (9)
            # statusWord == x0xx xxx1  x000 1111
            bitmask = 0b0100000101111111
            if (bitmask & statusword == 271):
                ID = 9
                return ID

            # state 'fault reaction active (enabled)' (10)
            # statusWord == xxxx xxxx  x0xx 1xx1
            bitmask = 0b0000000001001001
            if (bitmask & statusword == 73):
                ID = 10
                return ID

            # state 'fault' (11)
            # statusWord == xxxx xxxx  xxxx 1xxx
            bitmask = 0b0000000000001000
            if (bitmask & statusword == 8):
                ID = 11
                return ID

        # in case of unknown state or fail
        self.log_info('Error: Unknown state. Statusword is Bin={0:#018b}'.format(statusword))
        return -1

    def print_statusword(self):
        """ Print meaning of status word.

        See manual_ page 30 for meaning of each bit value.

        .. _manual: https://w5.siemens.com/web/cz/cz/corporate/portal/home/produkty_a_sluzby/IBT/mereni_a_regulace/frekvencni_menice/Documents/014_Parameter_Manual_CU230P_V441_en.pdf

        """
        statusword, Ok = self.read_statusword()
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
            print('Bit 15: Alarm drive converter overload (1=No, 0=Yes):          {0}'.format(
                (statusword & (1 << 15)) >> 15))
            print('Bit 14: Motor rotates forwards:                                {0}'.format(
                (statusword & (1 << 14)) >> 14))
            print('Bit 13: Reserved:                                              {0}'.format(
                (statusword & (1 << 13)) >> 13))
            print('Bit 12: velocity equal to zero:                                {0}'.format(
                (statusword & (1 << 12)) >> 12))
            print('Bit 11: I, M, P limit reached:                                 {0}'.format(
                (statusword & (1 << 11)) >> 11))
            print('Bit 10: Target reached:                                        {0}'.format(
                (statusword & (1 << 10)) >> 10))
            print('Bit 09: Control Request:                                       {0}'.format(
                (statusword & (1 << 9)) >> 9))
            print('Bit 08: Deviation, setpoint/actual speed(1=No, 0=Yes):         {0}'.format(
                (statusword & (1 << 8)) >> 8))
            print('Bit 07: Alarm:                                                 {0}'.format(
                (statusword & (1 << 7)) >> 7))
            print('Bit 06: Switch on disable:                                     {0}'.format(
                (statusword & (1 << 6)) >> 6))
            print('Bit 05: No Quick stop (OFF3):                                  {0}'.format(
                (statusword & (1 << 5)) >> 5))
            print('Bit 04: No Coast down active (OFF2):                           {0}'.format(
                (statusword & (1 << 4)) >> 4))
            print('Bit 03: Fault:                                                 {0}'.format(
                (statusword & (1 << 3)) >> 3))
            print('Bit 02: Operation enable:                                      {0}'.format(
                (statusword & (1 << 2)) >> 2))
            print('Bit 01: Ready:                                                 {0}'.format(
                (statusword & (1 << 1)) >> 1))
            print('Bit 00: Ready to switch on:                                    {0}'.format(statusword & 1))
        return

    def print_controlword(self, controlword=None):
        """Print the meaning of controlword

        Check the meaning of current controlword of device or check the meaning of your own controlword.
        Usefull to check your own controlword before actually sending it to device.

        Args:
            controlword (optional): If None, request the controlword of device.

        """
        if not controlword:
            controlword, Ok = self.read_controlword()
            if not Ok:
                print('[{0}:{1}] Failed to retreive controlword\n'.format(
                    self.__class__.__name__,
                    sys._getframe().f_code.co_name))
                return
        print("[{0}:{1}] The controlword is Hex={2:#06X} Bin={2:#018b}\n".format(
            self.__class__.__name__,
            sys._getframe().f_code.co_name,
            controlword))
        print('Bit 15: CDS bit 0:                                      {0}'.format((controlword & (1 << 15)) >> 15))
        print('Bit 14: Motorized potentiometer lower:                  {0}'.format((controlword & (1 << 14)) >> 14))
        print('Bit 13: Motorized potentiometer raise:                  {0}'.format((controlword & (1 << 13)) >> 13))
        print('Bit 12: Can be assigned to command:                     {0}'.format((controlword & (1 << 12)) >> 12))
        print('Bit 11: Direction of rotation [0 Foward | 1 Reverse]:   {0}'.format((controlword & (1 << 11)) >> 11))
        print('Bit 10: Master ctrl by PLC:                             {0}'.format((controlword & (1 << 10)) >> 10))
        print('Bit 09: Jog bit 1:                                      {0}'.format((controlword & (1 << 9)) >> 9))
        print('Bit 08: Jog bit 0:                                      {0}'.format((controlword & (1 << 8)) >> 8))
        print('Bit 07: Fault reset:                                    {0}'.format((controlword & (1 << 7)) >> 7))
        print('Bit 06: Speed setpoint enable:                          {0}'.format((controlword & (1 << 6)) >> 6))
        print('Bit 05: Continue ramp-function generator:               {0}'.format((controlword & (1 << 5)) >> 5))
        print('Bit 04: Ramp-function generator enable:                 {0}'.format((controlword & (1 << 4)) >> 4))
        print('Bit 03: Enable operation:                               {0}'.format((controlword & (1 << 3)) >> 3))
        print('Bit 02: OC / OFF3 Do not activate quick stop            {0}'.format((controlword & (1 << 2)) >> 2))
        print('Bit 01: OC / OFF2 Do not activate coast:                {0}'.format((controlword & (1 << 1)) >> 1))
        print('Bit 00: ON/OFF1:                                        {0}'.format(controlword & 1))
        return

    def print_state(self):
        ID = self.check_state()
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

    def read_parameter(self, parameter=None):
        """ Read Sinamics parameter value.

        Args:
            parameter: location to be read.
        Returns:
            tuple: A tuple containing:

            :val:  the current value or None if any error.
            :Ok: A boolean if all went ok.
        """
        index = 0x2000 + parameter
        val = self.read_object(index, 0)
        if val is None:
            self.log_info('Failed to request the Sinamics parameter')
            return None, False

        # return controlword as an int type
        return val, True

    def write_parameter(self, parameter=None, new_data=None, length=2):
        """ Write Sinamics parameter value

        Args:
            parameter: location to be written
            new_data: value to be written
            length: byte length
        Returns:
            bool: A boolean if all went ok
        """
        if (parameter is None) or (new_data is None):
            self.log_info('Check arguments. Invalid arguments')
            return False
        index = 0x2000 + parameter
        if type(new_data) is not bytes:
            new_data = new_data.to_bytes(length, 'little')
        return self.write_object(index, 0, new_data)

    def print_parameter(self, parameter=None, is_float=False):
        """Print value of requested SINAMICS parameter.

        Request the SINAMICS for the current value of parameter.
        In CAN, the parameter number, should be converted to hex
        and added with 0x2000 (for the first drive).

        Args:
            parameter: value of Sinamics parameter to be printed.
            is_float: Boolean, if the value to be read is float or not.
        """
        val, ok = self.read_parameter(parameter=parameter)
        if not ok:
            print('[{0}:{1}] Failed to retrieve parameter\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return

        if is_float:
            print('Parameter {0} value is {1}'.format(parameter, struct.unpack('<f', val)[0]))
        else:
            print('Parameter {0} value is {1}'.format(parameter, int.from_bytes(val, 'little')))
        return

    def set_target_velocity(self, rpm=0):
        """
        Set target velocity for sinamics

        Args:
            rpm: velocity in rpms. Must be a signed int32
        Returns:
            A boolean if all went ok or not.
        """
        index = self.objectIndex['TargetVelocity']
        subindex = 0
        if rpm > 2 ** 31 or rpm < -2 ** 31:
            self.log_info("RPM value outside range: {0}".format(rpm))
            return False

        return self.write_object(index, subindex, rpm.to_bytes(4, 'little', signed=True))

    def read_vof_min_voltage(self):
        """
        Read minimum V/F voltage for frequency equal to zero

        Return:
            int: current value of V/F voltage for f=0 or None if failed
        """
        val, ok = self.read_parameter(1319)
        if not ok:
            return None
        else:
            return struct.unpack('<f', val)[0]  # struct unpack returns a tupple even if only one value

    def set_vof_min_voltage(self, voltage=None):
        """
        Write minimum V/F voltage for frequency equal to zero

        Return:
            bool: a boolean if all went ok or not.

        """
        if voltage is None:
            self.log_info('Invalid arguments. No voltage value supplied')
            return False
        if voltage < 0 or voltage > 50:
            self.log_info('Voltage limits exceeded: {0}. Value must be between 0 and 50.'.format(voltage))
            return False
        voltage = struct.pack('<f', voltage)
        return self.write_parameter(parameter=1319, new_data=voltage, length=4)

    def print_vof_min_voltage(self):
        """
        Print value of voltage for frequency equal to zero
        """
        val = self.read_vof_min_voltage()
        if not val:
            print('[{0}:{1}] Failed to retrieve parameter\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return
        else:
            print('VOF min Voltage value is {0}V'.format(val))
            # print('VOF min Voltage value is {0}V'.format(int.from_bytes(val, 'little')))
        return

    def read_vof_char_voltage(self):
        """
        Read minimum V/F voltage for characteristic frequency.

        Return:
            int: current value of V/F voltage for characteristic frequency or None if failed
        """
        val, ok = self.read_parameter(1327)
        if not ok:
            return None
        else:
            return struct.unpack('<f', val)[0]  # struct unpack returns a tupple even if only one value

    def set_vof_char_voltage(self, voltage=None):
        """
        Write  V/F voltage for characteristic frequency

        Return:
            bool: a boolean if all went ok or not.

        """
        if voltage is None:
            self.log_info('Invalid arguments. No voltage value supplied')
            return False
        if voltage < 0 or voltage > 10000:
            self.log_info('Voltage limits exceeded: {0}. Value must be between 0 and 10000.'.format(voltage))
            return False
        voltage = struct.pack('<f', voltage)
        return self.write_parameter(parameter=1327, new_data=voltage, length=4)

    def print_vof_char_voltage(self):
        """
        Print value of voltage for characteristic frequency
        """
        val = self.read_vof_char_voltage()
        if not val:
            print('[{0}:{1}] Failed to retrieve parameter\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return
        else:
            print('VOF characteristic Voltage value is {0} V'.format(val))
        return

    def read_vof_char_frequency(self):
        """
        Read minimum V/F voltage for characteristic frequency.

        Return:
            int: current value of V/F voltage for characteristic frequency or None if failed
        """
        val, ok = self.read_parameter(1326)
        if not ok:
            return None
        else:
            return struct.unpack('<f', val)[0]

    def set_vof_char_frequency(self, frequency=None):
        """
        Write  V/F voltage for characteristic frequency

        Return:
            bool: a boolean if all went ok or not.

        """
        if frequency is None:
            self.log_info('Invalid arguments. No voltage value supplied')
            return False
        if frequency < 0 or frequency > 10000:
            self.log_info('Frequency limits exceeded: {0}. Value must be between 0 and 10000.'.format(frequency))
            return False
        frequency = struct.pack('<f', frequency)
        return self.write_parameter(parameter=1326, new_data=frequency, length=4)

    def print_vof_char_frequency(self):
        """
        Print value of characteristic frequency
        """
        val = self.read_vof_char_frequency()
        if not val:
            print('[{0}:{1}] Failed to retrieve parameter\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return
        else:
            print('VOF characteristic frequency value is {0} Hz'.format(val))
        return

    def read_torque_smoothed(self):
        """
        Read torque smoothed value.

        Return:
            float: current value of torque smoothed.
        """
        val, ok = self.read_parameter(parameter=31)
        if not ok:
            self.log_info("Failed to retrieve torque value")
            return None
        else:
            return struct.unpack('<f', val)[0]

    def print_torque_smoothed(self):
        """
        Print value of smoothed torque
        """
        val = self.read_torque_smoothed()
        if val is None:
            print('[{0}:{1}] Failed to retrieve parameter\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return
        else:
            print('Torque smoothed value is {0} N'.format(val))
        return

    def read_current_smoothed(self):
        """ Read current smoothed value.

        Return:
            float: value of current smoothed.
        """
        val, ok = self.read_parameter(parameter=27)
        if not ok:
            self.log_info("Failed to retrieve current value")
            return None
        else:
            return struct.unpack('<f', val)[0]

    def print_current_smoothed(self):
        """
        Print value of smoothed current
        """
        val = self.read_current_smoothed()
        if val is None:
            print('[{0}:{1}] Failed to retrieve parameter\n'.format(
                self.__class__.__name__,
                sys._getframe().f_code.co_name))
            return
        else:
            print('Current smoothed value is {0} A rms'.format(val))
        return

    def emcy_error_print(self, emcy_error):
        """Print any EMCY Error Received on CAN BUS
        """
        if emcy_error.code is 0:
            return
        else:
            fault_number = int.from_bytes(emcy_error.data[0:2], 'little')
            drive_object_number = int(emcy_error.data[2])
            if fault_number in self.sinamics_fault_number:
                description = self.sinamics_fault_number[fault_number]
            else:
                description = "unknown"
            self.log_info('Got an EMCY message {0}'.format(emcy_error))
            self.log_info('Sinamics error number: {0} with \'{1}\' in drive unit {2}'.format(fault_number,
                                                                                             description,
                                                                                             drive_object_number))
        return


def main():
    """Test SINAMICS CANopen communication with some examples.

    Use a few examples to test communication with SINAMICS device using
    a few functions. Also resets the fault error if present.

    Show sample using also the EDS file.
    """

    def print_velocity(message):
        """Print velocity value received from PDO

        Args:
            message: message received in PDO
        """
        logging.debug('{0} received'.format(message.name))
        for var in message:
            logging.debug('{0} = {1:06X}'.format(var.name, var.raw))
            if var.index == 0x6041:
                pass
            if var.index == 0x606C:
                logging.info('{0:+05d} RPM'.format(var.raw))

    import argparse
    if sys.version_info < (3, 0):
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
    parser.add_argument('--objDict', action='store', default='sinamics_s120.eds',
                        type=str, help='Object dictionary file', dest='objDict')
    parser.add_argument("--log-level", action="store", type=str,
                        dest="logLevel", default='info',
                        help='Log level to be used. See logging module for more info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'])

    args = parser.parse_args()
    log_level = {'error': logging.ERROR,
                 'debug': logging.DEBUG,
                 'info': logging.INFO,
                 'warning': logging.WARNING,
                 'critical': logging.CRITICAL
                 }
    # set up logging to file - see previous section for more details
    logging.basicConfig(level=log_level[args.logLevel],
                        format='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                        datefmt='%d-%m-%Y %H:%M:%S',
                        filename='sinamics.log',
                        filemode='w')

    # define a Handler which writes INFO messages or higher
    console = logging.StreamHandler()
    console.setLevel(log_level[args.logLevel])
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    # instantiate object
    inverter = SINAMICS()

    if not (inverter.begin(args.nodeID, object_dictionary=args.objDict)):
        logging.info('Failed to begin connection with SINAMICS device')
        logging.info('Exiting now')
        return
    # -------------------------------------------------------------------------------------
    # change default parameters for checking collisions.
    # -------------------------------------------------------------------------------------
    inverter.node.sdo.MAX_RETRIES = 2
    # inverter.node.sdo.PAUSE_BEFORE_SEND = 0.02
    # inverter.node.sdo.RESPONSE_TIMEOUT = 0.02

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
    statusword = inverter.read_object(0x6041, 0)
    if not statusword:
        print("[SINAMICS] Error trying to read SINAMICS statusword\n")
        return
    else:
        print('----------------------------------------------------------', flush=True)
        print("The statusword is \n Hex={0:#06X} Bin={0:#018b}".format(
            int.from_bytes(statusword, 'little')))
    # try to read operation mode
    opMode = inverter.read_object(0x6060, 0)
    if not opMode:
        print("[SINAMICS] Error trying to read SINAMICS opMode\n")
        return
    else:
        print('----------------------------------------------------------', flush=True)
        print("The opMode is \n Hex={0:#06X} Bin={0:#018b}".format(
            int.from_bytes(opMode, 'little')))
    # test print_statusword and state
    print('----------------------------------------------------------', flush=True)
    print('Testing print of StatusWord and State and ControlWord')
    print('----------------------------------------------------------', flush=True)
    inverter.print_state()
    print('----------------------------------------------------------', flush=True)
    inverter.print_statusword()
    print('----------------------------------------------------------', flush=True)
    inverter.print_controlword()
    print('----------------------------------------------------------', flush=True)
    # print VOF vars
    print('Testing print of VOF related variables')
    print('----------------------------------------------------------', flush=True)
    inverter.print_vof_min_voltage()
    inverter.print_vof_char_voltage()
    inverter.print_vof_char_frequency()
    print('----------------------------------------------------------', flush=True)
    print('Testing print of Torque and current values')
    print('----------------------------------------------------------', flush=True)
    inverter.print_torque_smoothed()
    inverter.print_current_smoothed()
    print('----------------------------------------------------------', flush=True)

    # emcy messages handles
    inverter.node.emcy.add_callback(inverter.emcy_error_print)

    # testing pdo objects
    inverter.node.pdo.read()

    inverter.node.nmt.state = 'PRE-OPERATIONAL'

    inverter.node.pdo.tx[1].clear()
    inverter.node.pdo.tx[2].clear()
    inverter.node.pdo.tx[3].clear()
    inverter.node.pdo.tx[4].clear()

    inverter.node.pdo.rx[1].clear()
    inverter.node.pdo.rx[2].clear()
    inverter.node.pdo.rx[3].clear()
    inverter.node.pdo.rx[4].clear()

    # Do some changes to TxPDO2
    inverter.node.pdo.tx[2].clear()
    inverter.node.pdo.tx[2].add_variable(0x6041, 0, 16)
    inverter.node.pdo.tx[2].add_variable(0x606C, 0, 32)
    inverter.node.pdo.tx[2].enabled = True
    # inverter.node.pdo.tx[2].event_timer = 2000
    inverter.node.pdo.tx[2].trans_type = 254

    inverter.change_state('fault reset')
    sleep(0.1)
    # Save parameters to device
    inverter.node.pdo.tx[2].save()

    # Add callback for message reception
    inverter.node.pdo.tx[2].add_callback(print_velocity)

    # Set back into operational mode
    inverter.node.nmt.state = 'OPERATIONAL'
    # TODO change State is failing. to be checked
    sleep(0.1)
    inverter.change_state('shutdown')
    sleep(0.1)
    inverter.change_state('switch on')
    sleep(0.1)
    inverter.change_state('enable operation')
    inverter.print_current_smoothed()

    try:
        print("Ctrl+C to exit... ")
        while True:
            velocity = int(input('Set your velocity...'))
            if velocity is None:
                pass
            else:
                print('Setting velocity to {0}'.format(velocity))
                inverter.set_target_velocity(velocity)
            sleep(3)
            inverter.print_current_smoothed()
    except KeyboardInterrupt as e:
        print('Got {0}\nexiting now'.format(e))
    except CanError:
        print("Message NOT sent")
    except ValueError:
        print("Invalid value")
    finally:
        # inverter.network.sync.stop()
        inverter.node.nmt.state = 'PRE-OPERATIONAL'
        inverter.set_target_velocity(0)
        inverter.change_state('shutdown')
    return


if __name__ == '__main__':
    main()
