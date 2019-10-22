#!/usr/bin/env python3
#
#Author: Andrew Maule
#Purpose: Create a micasense rededge mavlink proxy for communicating with a Pixhawk controller through
# its telemetry radio

'''
Runs a mavlink proxy translator/proxy between the Micasense RedEdge HTTPApi and a serial/udp port.
'''

#Camera definition file useful info: https://mavlink.io/en/services/camera_def.html
#Camera information message: https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION
#Camera definition file protocol definition: https://mavlink.io/en/services/camera_def.html#protocol-definition
#Camera request all parameters from the camera: https://mavlink.io/en/messages/common.html#PARAM_EXT_REQUEST_LIST
#Camera emit value of parameter: https://mavlink.io/en/messages/common.html#PARAM_EXT_VALUE
#Camera set a parameter value: https://mavlink.io/en/messages/common.html#PARAM_EXT_SET
#Camera response from PARAM_EXT_SET: https://mavlink.io/en/messages/common.html#PARAM_EXT_ACK
#Camera PARAM_ACK values: https://mavlink.io/en/messages/common.html#PARAM_ACK

import sys
assert sys.version_info >= (3,4)

from concurrent.futures import *
import copy
from datetime import datetime
import glob
import io
import json
from optparse import OptionParser
import os
from PIL import Image
from pymavlink import mavutil, mavparm
try:
    import queue as Queue
except ImportError:
    import Queue
import requests
import select
import signal
import subprocess as sp
import struct
from threading import Lock, Timer, Thread
from time import time, sleep
import xml.etree.ElementTree as ET
if sys.version_info < (3,7):
    #Fix to python versions 3.4, 3.5, and 3.6 that lack datetime/date's fromisoformat()
    from backports.datetime_fromisoformat import MonkeyPatch
    MonkeyPatch.patch_fromisoformat()

TIMER_INFINITE_COUNT = -1
MB_PER_GB = 1024 #Number of megabytes per gigabyte
dataPackets = frozenset(['BAD_DATA','LOG_DATA'])

def parse_args():
    parser = OptionParser("micamavproxy.py [options]")

    parser.add_option("--master", dest="master",
                      metavar="DEVICE", help="MAVLink master port and optional baud rate")
    parser.add_option("--ip", "--camera", dest='ip', default="192.168.87.1", help="IP address of MicaSense RedEdgeMX camera.")
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="default serial baud rate", default=57600)
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=1, help='MAVLink camera source system for this vehicle -- Defaults to autopilot vehicle')
    parser.add_option("--source-component", dest='SOURCE_COMPONENT', type='int', 
                      default=100, help='MAVLink source component for this GCS')
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=254, help='MAVLink target master system')
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=0, help='MAVLink target master component')
    parser.add_option("--camera_defs_local", default="micasense_rededge_mx.xml", help="If extra parameters are supported for this camera, this is the location of the camera definitions file (XML).")
    parser.add_option("--camera_defs_url", default="https://raw.githubusercontent.com/bliptrip/RedEdgeMXProxy/master/micasense_rededge_mx.xml", help="If extra parameters are supported for this camera, this is the location of the camera definitions file (XML).")
    parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
    parser.add_option("--mav20", action='store_true', default=True, help="Use MAVLink protocol 2.0")
    parser.add_option("--version", action='store_true', help="version information")
    parser.add_option("--dialect",  default="common", help="MAVLink dialect")
    parser.add_option("--rtscts", action='store_true', help="enable hardware RTS/CTS flow control")
    parser.add_option("--stream_port", type='int', default=5600, help='Stream live video feed to designated UDP port')
    parser.add_option("--stream_ip", default='127.0.0.1', help='Stream live video feed to designated IP address.')
    parser.add_option("--log_file", help='Stream all traffic in/out of master to specified log file.')
    parser.add_option("--multicast", action='store_true', default=False, help="Stream video as multicast.  NOTE: This will not work for some wireless cards.")
    return(parser.parse_args())


class RedEdgeAPISession():
    def __init__(self, ip, mavl):
        self.ip         = ip
        self.mavl       = mavl
        self.session    = ElapsedFutureSession()
        return

    def get_capabilities(self):
        return mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_IMAGE | mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE

    def get_network_status(self, callback):
        self.session.request('get', url="http://%s/networkstatus" % self.ip, hooks={'response': [callback]})
        return

def http_post(ip, route, payload):
    rd = None
    try:
        #Insert a timeout, or can hang indefinitely
        r = requests.post("http://{0}/{1}".format(ip,route), data=payload, timeout=5.0)
    except requests.exceptions.ConnectionError:
        pass
    except requests.exceptions.Timeout:
        pass
    else:
        if(r.status_code == requests.codes.ok):
            try:
                rd = r.json()
            except ValueError:
                pass
    return(rd)

def http_get(ip, route):
    rd = None
    try:
        #Insert a timeout, or can hang indefinitely
        r = requests.get("http://{0}/{1}".format(ip,route), timeout=5.0)
    except requests.exceptions.Timeout:
        pass
    else:
        if(r.status_code == requests.codes.ok):
            try:
                rd = r.json()
            except ValueError:
                pass
    return(rd)

def param_decode(value, param_id, mavcamparams, as_byte_array=False):
    """
    Takes 'value' of extra parameter as string and converts to appropriate type.
    """
    if( as_byte_array == True ):
        rvalue = value
    else:
        rtype = mavcamparams.rtype(param_id)
        rvalue = mav_param_ext_types[rtype]['castfunc'](value)
    return(rvalue)

def param_encode(value, param_type, as_byte_array=False):
    """
    Take raw 'value' of extra parameter and converts to string for sending over
    mavlink.
    """
    if( as_byte_array == True ):
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT8 ):
            rvalue = struct.pack('<B', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_INT8 ):
            rvalue = struct.pack('<b', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT16 ):
            rvalue = struct.pack('<H', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_INT16 ):
            rvalue = struct.pack('<h', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32 ):
            rvalue = struct.pack('<I', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_INT32 ):
            rvalue = struct.pack('<i', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT64 ):
            rvalue = struct.pack('<Q', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_INT64 ):
            rvalue = struct.pack('<q', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_REAL32 ):
            rvalue = struct.pack('<f', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_REAL64 ):
            rvalue = struct.pack('<d', value)
        if( param_type == mavutil.mavlink.MAV_PARAM_EXT_TYPE_CUSTOM ):
            rvalue = 0 #Unsupported as of now with this setup
    else:
        rvalue = str(value).encode('utf-8')
    rvalue = struct.pack('<128s', rvalue)
    return(rvalue)


class CameraIntervalTimer(object):
    """
    Python periodic Thread using Timer with instant cancellation
    """

    def __init__(self, callback, period, name, count, *args, **kwargs):
        self.name = name
        self.args = args
        self.kwargs = kwargs
        self.callback = callback
        self.count      = count
        self.period = period
        self.stop = False
        self.running = False
        self.current_timer = None
        self.schedule_lock = Lock()
        self.time_started = 0

    def start(self):
        """
        Mimics Thread standard start method
        """
        with self.schedule_lock:
            self.running = True
            self.schedule_timer()

    def run(self):
        """
        By default run callback. Override it if you want to use inheritance
        """
        if self.callback is not None:
            self.callback(*self.args, **self.kwargs)

    def _run(self):
        """
        Run desired callback and then reschedule Timer (if thread is not stopped)
        """
        try:
            self.run()
        except Exception as e:
            print("Exception in running periodic thread: {}".format(e))
        finally:
            with self.schedule_lock:
                if not self.stop:
                    self.schedule_timer()

    def schedule_timer(self):
        """
        Schedules next Timer run
        """
        if( (self.count > 0) or (self.count == TIMER_INFINITE_COUNT) ):
            if( self.count > 0 ):
                self.count = self.count - 1
            self.current_timer = Timer(self.period, self._run)
            if self.name:
                self.current_timer.name = self.name
            self.current_timer.start()
        else:
                self.running = False
                self.stop = True
                if self.current_timer is not None:
                    self.current_timer.cancel()
                    self.current_timer = None

    def cancel(self):
        """
        Mimics Timer standard cancel method
        """
        with self.schedule_lock:
            self.stop = True
            if self.current_timer is not None:
                self.current_timer.cancel()
                self.current_timer = None

    def join(self):
        """
        Mimics Thread standard join method
        """
        self.current_timer.join()


class MavCamParams():
    def __init__(self, ip, camera_defs):
        self.ip                 = ip #Camera IP address
        self.params             = {} #Store the camera extra parameters stored in the camera definitions xml file.
        self._model             = ""
        self._vendor            = ""
        #r = requests.get(camera_defs)
        #if(r.status_code == requests.codes.ok):
        #    stream = io.BytesIO(r.content)
        #self.cam_defs_tree      = ET.parse(stream) #Parse the mavlink camera definitions xml file to populate a dictionary of parameters that we will track/store
        self.cam_defs_tree      = ET.parse(camera_defs) #Parse the mavlink camera definitions xml file to populate a dictionary of parameters that we will track/store
        self.populate_ext_params(self.cam_defs_tree)

    def populate_ext_params(self, tree):
        root = tree.getroot()
        if root.tag == 'mavlinkcamera':
            definitions = root.findall('./definition')
            for d in definitions:
                if d.tag == 'model':
                    self._model = d.text
                elif d.tag == 'vendor':
                    self._vendor = d.text
            params = root.findall('./parameters/parameter')
            for p in params:
                p.attrib['mtype'] = -1
                if 'type' in p.attrib:
                    ptype = p.attrib['type']
                    if( ptype in mav_param_ext_types ):
                        p.attrib['mtype'] = mav_param_ext_types[ptype]['type']
                        p.attrib['stale'] = True #Mark parameters as initially stale to force them to be read in from the camera
                        if( 'default' in p.attrib ):
                            p.attrib['value'] = mav_param_ext_types[ptype]['castfunc'](p.attrib['default'])
                        else:
                            p.attrib['value'] = mav_param_ext_types[ptype]['default']
                paramname = p.attrib['name']
                self.params[paramname] = {'attrib':p.attrib, 'property': '_{0}_'.format(paramname)}
        return

    def keys(self):
        return(self.params.keys())

    def __getitem__(self, key):
        return getattr(self, '_{0}_'.format(key))

    def __setitem__(self, key, value):
        return setattr(self, '_{0}_'.format(key), value)

    def type(self, key):
        """
        Return the mavlink type for this parameter.
        """
        return(self.params[key]['attrib']['mtype'])

    def rtype(self, key):
        """
        Return the raw 'xml' type, not the mavlink type.
        """
        return(self.params[key]['attrib']['type'])
        
    @property
    def model(self):
        return(self._model)

    @property
    def vendor(self):
        return(self._vendor)

    #Extra parameters getters/setters
    @property
    def _CAM_EXP_MAN_(self):
        if( self.params["CAM_EXP_MAN"]['attrib']['stale'] == True ):
            rd = http_get(self.ip, 'exposure')
            if rd is not None:
                value = rd['enable_man_exposure']
                self.params["CAM_EXP_MAN"]['attrib']['value'] = int(value)
                self.params["CAM_EXP_MAN"]['attrib']['stale'] = False
        return(self.params["CAM_EXP_MAN"]['attrib']['value'])

    @_CAM_EXP_MAN_.setter
    def _CAM_EXP_MAN_(self, value):
        success = False 
        mvalue = bool(value)
        rd = http_post(self.ip, 'exposure', {'enable_man_exposure': mvalue})
        if rd is not None:
            rvalue = rd['enable_man_exposure']
            if( mvalue == rvalue ):
                success = True
                self.params["CAM_EXP_MAN"]['attrib']['value'] = value
        return(success)

    @property
    def _CAM_DET_PAN_(self):
        return(self.params["CAM_DET_PAN"]['attrib']['value'])

    @_CAM_DET_PAN_.setter
    def _CAM_DET_PAN_(self, value):
        self.params["CAM_DET_PAN"]['attrib']['value'] = value
        return(True)

    def _cam_exp_read(self, id):
        CAM_EXP = 'CAM_EXP{}'.format(id)
        exposure = 'exposure{}'.format(id)
        if(self.params[CAM_EXP]['attrib']['stale'] == True):
            rd = http_get(self.ip, 'exposure')
            if rd is not None:
                value = rd[exposure]
                self.params[CAM_EXP]['attrib']['value'] = value
                self.params[CAM_EXP]['attrib']['stale'] = False
        return(self.params[CAM_EXP]['attrib']['value'])

    def _cam_exp_write(self, id, value):
        CAM_EXP = 'CAM_EXP{}'.format(id)
        exposure = 'exposure{}'.format(id)
        success = False
        rd = http_post(self.ip, 'exposure', {exposure: value})
        if rd is not None:
            rvalue = rd[exposure]
            if( value == rvalue ):
                success = True
                self.params[CAM_EXP]['attrib']['value'] = value
        return(success)

    @property
    def _CAM_EXP1_(self):
        return(self._cam_exp_read(1))

    @_CAM_EXP1_.setter
    def _CAM_EXP1_(self, value):
        return(self._cam_exp_write(1, value))

    @property
    def _CAM_EXP2_(self):
        return(self._cam_exp_read(2))

    @_CAM_EXP2_.setter
    def _CAM_EXP2_(self, value):
        return(self._cam_exp_write(2, value))

    @property
    def _CAM_EXP3_(self):
        return(self._cam_exp_read(3))

    @_CAM_EXP3_.setter
    def _CAM_EXP3_(self, value):
        return(self._cam_exp_write(3, value))

    @property
    def _CAM_EXP4_(self):
        return(self._cam_exp_read(4))

    @_CAM_EXP4_.setter
    def _CAM_EXP4_(self, value):
        return(self._cam_exp_write(4, value))

    @property
    def _CAM_EXP5_(self):
        return(self._cam_exp_read(5))

    @_CAM_EXP5_.setter
    def _CAM_EXP5_(self, value):
        return(self._cam_exp_write(5, value))

    def _cam_gain_read(self, id):
        CAM_EXP_GAIN = 'CAM_EXP_GAIN{}'.format(id)
        gain = 'gain{}'.format(id)
        if(self.params[CAM_EXP_GAIN]['attrib']['stale'] == True):
            rd = http_get(self.ip, 'exposure')
            if rd is not None:
                value = rd[gain]
                self.params[CAM_EXP_GAIN]['attrib']['value'] = value
                self.params[CAM_EXP_GAIN]['attrib']['stale'] = False
        return(self.params[CAM_EXP_GAIN]['attrib']['value'])

    def _cam_gain_write(self, id, value):
        CAM_EXP_GAIN = 'CAM_EXP_GAIN{}'.format(id)
        gain = 'gain{}'.format(id)
        success = False
        rd = http_post(self.ip, 'exposure', {gain: value})
        if rd is not None:
            rvalue = rd[gain]
            if( value == rvalue ):
                success = True
                self.params[CAM_EXP_GAIN]['attrib']['value'] = value
        return(success)

    @property
    def _CAM_EXP_GAIN1_(self):
        return(self._cam_gain_read(1))

    @_CAM_EXP_GAIN1_.setter
    def _CAM_EXP_GAIN1_(self, value):
        return(self._cam_gain_write(1, value))

    @property
    def _CAM_EXP_GAIN2_(self):
        return(self._cam_gain_read(2))

    @_CAM_EXP_GAIN2_.setter
    def _CAM_EXP_GAIN2_(self, value):
        return(self._cam_gain_write(2, value))

    @property
    def _CAM_EXP_GAIN3_(self):
        return(self._cam_gain_read(3))

    @_CAM_EXP_GAIN3_.setter
    def _CAM_EXP_GAIN3_(self, value):
        return(self._cam_gain_write(3, value))

    @property
    def _CAM_EXP_GAIN4_(self):
        return(self._cam_gain_read(4))

    @_CAM_EXP_GAIN4_.setter
    def _CAM_EXP_GAIN4_(self, value):
        return(self._cam_gain_write(4, value))

    @property
    def _CAM_EXP_GAIN5_(self):
        return(self._cam_gain_read(5))

    @_CAM_EXP_GAIN5_.setter
    def _CAM_EXP_GAIN5_(self, value):
        return(self._cam_gain_write(5, value))

    @property
    def _CAM_AC_PHI_(self):
        return(self.params["CAM_AC_PHI"]['attrib']['value'])

    @_CAM_AC_PHI_.setter
    def _CAM_AC_PHI_(self, value):
        self.params["CAM_AC_PHI"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_AC_THETA_(self):
        return(self.params["CAM_AC_THETA"]['attrib']['value'])

    @_CAM_AC_THETA_.setter
    def _CAM_AC_THETA_(self, value):
        self.params["CAM_AC_THETA"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_AC_PSI_(self):
        return(self.params["CAM_AC_PSI"]['attrib']['value'])

    @_CAM_AC_PSI_.setter
    def _CAM_AC_PSI_(self, value):
        self.params["CAM_AC_PSI"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_CAM_PHI_(self):
        return(self.params["CAM_CAM_PHI"]['attrib']['value'])

    @_CAM_CAM_PHI_.setter
    def _CAM_CAM_PHI_(self, value):
        self.params["CAM_CAM_PHI"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_CAM_THETA_(self):
        return(self.params["CAM_CAM_THETA"]['attrib']['value'])

    @_CAM_CAM_THETA_.setter
    def _CAM_CAM_THETA_(self, value):
        self.params["CAM_CAM_THETA"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_CAM_PSI_(self):
        return(self.params["CAM_CAM_PSI"]['attrib']['value'])

    @_CAM_CAM_PSI_.setter
    def _CAM_CAM_PSI_(self, value):
        self.params["CAM_CAM_PSI"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_ALT_(self):
        return(self.params["CAM_ALT"]['attrib']['value'])

    @_CAM_ALT_.setter
    def _CAM_ALT_(self, value):
        self.params["CAM_ALT"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_A_OVERLAP_(self):
        return(self.params["CAM_A_OVERLAP"]['attrib']['value'])

    @_CAM_A_OVERLAP_.setter
    def _CAM_A_OVERLAP_(self, value):
        self.params["CAM_A_OVERLAP"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_X_OVERLAP_(self):
        return(self.params["CAM_X_OVERLAP"]['attrib']['value'])

    @_CAM_X_OVERLAP_.setter
    def _CAM_X_OVERLAP_(self, value):
        self.params["CAM_X_OVERLAP"]['attrib']['value'] = value
        return(True)

    @property
    def _CAM_AUTO_CAP_(self):
        if(self.params["CAM_AUTO_CAP"]['attrib']['stale'] == True):
            rd = http_get(self.ip, 'config')
            if rd is not None:
                prevalue = rd['auto_cap_mode']
                if( prevalue == 'disabled' ):
                    value = 0
                elif( prevalue == 'overlap' ):
                    value = 1
                elif( prevalue == 'timer' ):
                    value = 2
                elif( prevalue == 'ext' ):
                    value = 3
                else:
                    value = -1 #invalid
                self.params["CAM_AUTO_CAP"]['attrib']['value'] = value
                self.params["CAM_AUTO_CAP"]['attrib']['stale'] = False
        return(self.params["CAM_AUTO_CAP"]['attrib']['value'])

    @_CAM_AUTO_CAP_.setter
    def _CAM_AUTO_CAP_(self, value):
        success = False
        if( value == 0 ):
            postvalue = 'disabled'
        elif( value == 1 ):
            postvalue = 'overlap'
        elif( value == 2 ):
            postvalue = 'timer'
        elif( value == 3 ):
            postvalue = 'ext'
        else:
            return(False) #Invalid value
        rd = http_post(self.ip, 'config', {'auto_cap_mode': postvalue})
        if rd is not None:
            rvalue = rd['auto_cap_mode']
            if( postvalue == rvalue ):
                success = True
                self.params['CAM_AUTO_CAP']['attrib']['value'] = value
        return(success)

    @property
    def _CAM_INT_TIMER_(self):
        if( self.params["CAM_INT_TIMER"]['attrib']['stale'] == True ):
            rd = http_get(self.ip, 'config')
            if rd is not None:
                value = rd['timer_period']
                self.params["CAM_INT_TIMER"]['attrib']['value'] = value
                self.params["CAM_INT_TIMER"]['attrib']['stale'] = False
        return(self.params["CAM_INT_TIMER"]['attrib']['value'])

    @_CAM_INT_TIMER_.setter
    def _CAM_INT_TIMER_(self, value):
        success = False 
        rd = http_post(self.ip, 'config', {'timer_period': value})
        if rd is not None:
            rvalue = rd['timer_period']
            if( value == rvalue ):
                success = True
                self.params["CAM_INT_TIMER"]['attrib']['value'] = value
        return(success)

    @property
    def _CAM_EXT_TRIG_MOD_(self):
        if(self.params["CAM_EXT_TRIG_MOD"]['attrib']['stale'] == True):
            rd = http_get(self.ip, 'config')
            if rd is not None:
                prevalue = rd['ext_trigger_mode']
                if( prevalue == 'rising' ):
                    value = 0
                elif( prevalue == 'falling' ):
                    value = 1
                elif( prevalue == 'longpwm' ):
                    value = 2
                elif( prevalue == 'shortpwm' ):
                    value = 3
                else:
                    value = -1 #invalid
            if rd is not None:
                self.params["CAM_EXT_TRIG_MOD"]['attrib']['value'] = value
                self.params["CAM_EXT_TRIG_MOD"]['attrib']['stale'] = False
        return(self.params["CAM_EXT_TRIG_MOD"]['attrib']['value'])

    @_CAM_EXT_TRIG_MOD_.setter
    def _CAM_EXT_TRIG_MOD_(self, value):
        success = False
        if( value == 0 ):
            postvalue = 'rising'
        elif( value == 1 ):
            postvalue = 'falling'
        elif( value == 2 ):
            postvalue = 'longpwm'
        elif( value == 3 ):
            postvalue = 'shortpwm'
        else:
            return(False) #Invalid value
        rd = http_post(self.ip, 'config', {'ext_trigger_mode': postvalue})
        if rd is not None:
            rvalue = rd['timer_period']
            if( postvalue == rvalue ):
                success = True
                self.params[CAM_EXT_TRIG_MOD]['attrib']['value'] = value
        return(success)

    @property
    def _CAM_PWM_THR_(self):
        if( self.params["CAM_PWM_THR"]['attrib']['stale'] == True ):
            rd = http_get(self.ip, 'config')
            if rd is not None:
                value = rd['pwm_trigger_threshold']
                self.params["CAM_PWM_THR"]['attrib']['value'] = value
                self.params["CAM_PWM_THR"]['attrib']['stale'] = False
        return(self.params["CAM_PWM_THR"]['attrib']['value'])

    @_CAM_PWM_THR_.setter
    def _CAM_PWM_THR_(self, value):
        success = False 
        rd = http_post(self.ip, 'config', {'pwm_trigger_threshold': value})
        if rd is not None:
            rvalue = rd['pwm_trigger_threshold']
            if( value == rvalue ):
                success = True
                self.params["CAM_PWM_THR"]['attrib']['value'] = value
        return(success)

    @property
    def _CAM_RAW_FORMAT_(self):
        if(self.params["CAM_RAW_FORMAT"]['attrib']['stale'] == True):
            rd = http_get(self.ip, 'config')
            if rd is not None:
                prevalue = rd['raw_format']
                if( prevalue == 'DNG' ):
                    value = 0
                elif( prevalue == 'TIFF' ):
                    value = 1
                else:
                    value = -1 #invalid
            if rd is not None:
                self.params["CAM_RAW_FORMAT"]['attrib']['value'] = value
                self.params["CAM_RAW_FORMAT"]['attrib']['stale'] = False
        return(self.params["CAM_RAW_FORMAT"]['attrib']['value'])

    @_CAM_RAW_FORMAT_.setter
    def _CAM_RAW_FORMAT_(self, value):
        success = False
        if( value == 0 ):
            postvalue = 'DNG'
        elif( value == 1 ):
            postvalue = 'TIFF'
        else:
            return(False) #Invalid value
        rd = http_post(self.ip, 'config', {'raw_format': postvalue})
        if rd is not None:
            rvalue = rd['raw_format']
            if( postvalue == rvalue ):
                success = True
                self.params['CAM_RAW_FORMAT']['attrib']['value'] = value
        return(success)

    @property
    def _CAM_AGC_MIN_MEAN_(self):
        return(self.params["CAM_AGC_MIN_MEAN"]['attrib']['value'])

    @_CAM_AGC_MIN_MEAN_.setter
    def _CAM_AGC_MIN_MEAN_(self, value):
        self.params["CAM_AGC_MIN_MEAN"]['attrib']['value'] = value
        return(True)

    @property
    def _GCS_STREAM_PORT_(self):
        return(self.params["GCS_STREAM_PORT"]['attrib']['value'])

    @_GCS_STREAM_PORT_.setter
    def _GCS_STREAM_PORT_(self, value):
        self.params["GCS_STREAM_PORT"]['attrib']['value'] = value
        return(True)

    @property
    def _GCS_STREAM_IP3_(self):
        return(self.params["GCS_STREAM_IP3"]['attrib']['value'])

    @_GCS_STREAM_IP3_.setter
    def _GCS_STREAM_IP3_(self, value):
        self.params["GCS_STREAM_IP3"]['attrib']['value'] = value
        return(True)

    @property
    def _GCS_STREAM_IP2_(self):
        return(self.params["GCS_STREAM_IP2"]['attrib']['value'])

    @_GCS_STREAM_IP2_.setter
    def _GCS_STREAM_IP2_(self, value):
        self.params["GCS_STREAM_IP2"]['attrib']['value'] = value
        return(True)

    @property
    def _GCS_STREAM_IP1_(self):
        return(self.params["GCS_STREAM_IP1"]['attrib']['value'])

    @_GCS_STREAM_IP1_.setter
    def _GCS_STREAM_IP1_(self, value):
        self.params["GCS_STREAM_IP1"]['attrib']['value'] = value
        return(True)

    @property
    def _GCS_STREAM_IP0_(self):
        return(self.params["GCS_STREAM_IP0"]['attrib']['value'])

    @_GCS_STREAM_IP0_.setter
    def _GCS_STREAM_IP0_(self, value):
        self.params["GCS_STREAM_IP0"]['attrib']['value'] = value
        return(True)

#Rather than a thread pool, consider using queues to process messages and send responses in a meaningful way

class MicaMavLink:
    def __init__(self, ip, mav10, mav20, source_system, source_component, target_system, target_component, rtscts, baudrate, descriptor, stream_ip, stream_port, camera_defs, camera_url, log_file, isMulticast):
        self.ip                 = ip
        self.source_system      = source_system
        self.source_component   = source_component
        self.target_system      = target_system
        self.target_component   = target_component
        self.descriptor         = descriptor 
        self.baudrate           = baudrate
        self.rtscts             = rtscts
        self.flushlogs          = True
        self.log_exit           = False
        self.log_file           = log_file
        if( log_file ):
            self.logfd          = open(log_file, 'wb')
            self.logq           = Queue.Queue()
            self.logq.queue.clear() #Flush the queue before starting
            self.logt           = Thread(target=self.logqt, name='logqt: {}'.format(self))
            #self.logt.daemon    = True
            self.logt.start()
        self.recvq              = Queue.Queue() #Mavlink receive/process message queue
        self.recvq_exit         = False
        self.recvt              = Thread(target=self.recvq_process, name='recvq_process: {}'.format(self))
        self.recvt.start()
        self.link_add(descriptor)
        self.cam_stream_timer   = None
        self.cam_interval_timer = None
        self.mav_cam_params     = MavCamParams(ip, camera_defs)
        self.camera_defs        = camera_defs
        self.camera_url         = camera_url.encode('utf-8')
        self.heartbeat_period   = mavutil.periodic_event(1)
        self.gst_command        = ['gst-launch-1.0', '-v',
                                   'fdsrc',
                                   '!', 'jpegdec', #Need to decode jpeg
                                   '!', 'jpegenc', #and then re-encode for it to work?  Not sure why.
                                   '!', 'progressreport',
                                   '!', 'rtpjpegpay',
                                   '!', 'udpsink', 'host={}'.format(stream_ip), 'port={}'.format(stream_port)]
        if( isMulticast ):
            self.gst_command        = self.gst_command + ['auto-multicast=true']
        self.vendor             = struct.pack('<32s',"MicaSense".encode('utf-8'))
        self.model              = struct.pack('<32s',"RedEdgeMX".encode('utf-8'))
        self.mav_cmd_video_start_streaming(None, self.master, {})


    def shutdown(self):
        #Shutdown video streaming
        self.mav_cmd_video_stop_streaming(None, None, {})
        #Shutdown the mavlink connection callbacks
        self.master.mav.set_callback(None, self.master)
        if hasattr(self.master.mav, 'set_send_callback'):
            self.master.mav.set_send_callback(None, self.master)
        #Shutdown the receive message thread
        self.recvq_exit = True
        self.recvt.join() #Wait for thread to fully die
        #Shutdown the log_writer() thread
        if( self.log_file ):
            self.log_exit = True
            self.logt.join()
            self.logfd.close()

    def logqt(self):
        '''log queue thread'''
        while not self.log_exit:
            timeout = time() + 10
            while not self.logq.empty() and time() < timeout:
                msg = self.logq.get()
                #Do a sanity check to make sure the message at offset 8 is the mavlink start byte
                self.logfd.write(msg)
            if self.flushlogs or time() >= timeout:
                self.logfd.flush()

    def master_send_callback(self, m, master):
        '''called on sending a message'''
        if( self.logq ):
            sysid = m.get_srcSystem()
            mtype = m.get_type()
            message = copy.deepcopy(m.get_msgbuf())
            self.logq.put(message)
        return

    def read(self, timeout):
        master = self.master
        rin = []
        if master.fd is not None and not master.portdead:
            rin.append(master.fd)
        if rin != []:
            try:
                (rin, win, xin) = select.select(rin, [], [], timeout)
            except select.error:
                rin = []
                pass 
        for fd in rin:
            if fd == master.fd:
                process_master(master, self)
        return

    def send_heartbeat(self):
        if self.master.mavlink10():
            self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_CAMERA, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

    def heartbeat_trigger(self):
        if( self.heartbeat_period.trigger() ):
                self.send_heartbeat()


    def parse_link_attributes(self, some_json):
        '''return a dict based on some_json (empty if json invalid)'''
        try:
            return json.loads(some_json)
        except ValueError:
            print('Invalid JSON argument: {0}'.format(some_json))
        return {}

    def parse_link_descriptor(self, descriptor):
        '''parse e.g. 'udpin:127.0.0.1:9877:{"foo":"bar"}' into
        python structure ("udpin:127.0.0.1:9877", {"foo":"bar"})'''
        optional_attributes = {}
        link_components = descriptor.split(":{", 1)
        device = link_components[0]
        if (len(link_components) == 2 and link_components[1].endswith("}")):
            # assume json
            some_json = "{" + link_components[1]
            optional_attributes = self.parse_link_attributes(some_json)
        return (device, optional_attributes)

    def apply_link_attributes(self, conn, optional_attributes):
        for attr in optional_attributes:
            print("Applying attribute to link: %s = %s" % (attr, optional_attributes[attr]))
            setattr(conn, attr, optional_attributes[attr])

    def link_add(self, descriptor, force_connected=False):
        '''add new link'''
        try:
            (device, optional_attributes) = self.parse_link_descriptor(descriptor)
            print("Connect %s source_system=%d" % (device, self.source_system))
            try:
                conn = mavutil.mavlink_connection(device, autoreconnect=True,
                                                    source_system=self.source_system,
                                                    baud=self.baudrate,
                                                    force_connected=force_connected)
            except Exception as e:
                # try the same thing but without force-connected for
                # backwards-compatability
                conn = mavutil.mavlink_connection(device, autoreconnect=True,
                                                    source_system=self.source_system,
                                                    baud=self.baudrate)
            conn.mav.srcComponent = self.source_component
        except Exception as msg:
            print("Failed to connect to %s : %s" % (descriptor, msg))
            return False
        if self.rtscts:
            conn.set_rtscts(True)
        conn.mav.set_callback(self.master_callback, conn)
        if hasattr(conn.mav, 'set_send_callback'):
            conn.mav.set_send_callback(self.master_send_callback, conn)
        #conn.linknum = len(self.mpstate.mav_master)
        conn.linknum = 1
        conn.linkerror = False
        conn.link_delayed = False
        conn.last_heartbeat = 0
        conn.last_message = 0
        conn.highest_msec = 0
        conn.target_system = self.target_system
        self.apply_link_attributes(conn, optional_attributes)
        #self.mpstate.mav_master.append(conn)
        #self.status.counters['MasterIn'].append(0)
        try:
            mp_util.child_fd_list_add(conn.port.fileno())
        except Exception:
            pass
        self.master = conn
        return True

    def master_msg_handling(self, m, master):
        '''link message handling for an upstream link'''
        if self.target_system != 0 and master.target_system != self.target_system:
            # keep the pymavlink level target system aligned with the MAVProxy setting
            print("change target_system %u" % self.target_system)
            master.target_system = self.target_system

        if self.target_component != mavutil.mavlink.MAV_COMP_ID_ALL and master.target_component != self.target_component:
            # keep the pymavlink level target component aligned with the MAVProxy setting
            print("change target_component %u" % self.target_component)
            master.target_component = self.target_component

        #Any messages not coming from our targeted system (GCS) will be ignored
        if (self.target_system == 0) or (m.get_srcSystem() == self.target_system):
            mtype = m.get_type()
            if( mtype in mav_cam_input_messages ):
                #print('master_msg_handling(): Received mavlink message with source system id {} and source component {}, type: {}'.format(m.get_srcSystem(), m.get_srcComponent(), mtype)) 
                decoded_message_dict = self.mav_decode(m, master)
                #If the target_system and target_component are embedded in a message, then check that they are valid for this component
                if ('target_system' in decoded_message_dict) and ('target_component' in decoded_message_dict):
                    target_system       = decoded_message_dict['target_system']
                    target_component    = decoded_message_dict['target_component']
                    if (target_system == self.source_system) and (target_component == self.source_component):
                        self.recvq.put((getattr(self, mav_cam_input_messages[mtype]), m, master, decoded_message_dict))
                else:
                    self.recvq.put((getattr(self, mav_cam_input_messages[mtype]), m, master, decoded_message_dict))

    def recvq_process(self):
        while not self.recvq_exit:
            (cb, m, master, decoded_message_dict) = self.recvq.get()
            cb(m, master, decoded_message_dict) #Make a call to process appropriate message processing callback
        return

    def master_callback(self, m, master):
        '''process mavlink message m on master, sending any messages to recipients'''
        # see if it is handled by a specialised sysid connection
        sysid = m.get_srcSystem()
        compid = m.get_srcComponent()
        mtype = m.get_type()
        
        #if( (sysid == self.target_system) and (compid == self.target_component) and (mtype in mav_cam_input_messages) and self.logq ):
        if( (sysid == self.target_system) and (compid == self.target_component) and self.logq ):
            #print("Received master callback with source system {}, type {}".format(sysid,mtype))
            message = copy.deepcopy(m.get_msgbuf())
            self.logq.put(message)

        if getattr(m, '_timestamp', None) is None:
            master.post_message(m)

        self.master_msg_handling(m, master)

    def mav_decode(self, m, master):
        b = m.get_msgbuf()
        dm = master.mav.decode(b)
        dmd = dm.to_dict()
        return(dmd)

    def mav_cmd_long(self, m, master, decoded_message_dict):
        cmdid = decoded_message_dict['command']
        if cmdid in mav_cam_cmds:
            #print("Received COMMAND_LONG with command id {}".format(cmdid))
            cmdfunc = getattr(self, mav_cam_cmds[cmdid], None)
            #print(decoded_message_dict)
            #print(cmdfunc)
            try:
                result = cmdfunc(m=m, master=master, decoded_message_dict=decoded_message_dict)
            except Exception as e:
                result = mavutil.mavlink.MAV_RESULT_FAILED
                print("Exception received: {}".format(e))
                pass
            #print(result)
        else:
            result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        master.mav.command_ack_send(command=decoded_message_dict['command'],
                         result=result,
                         target_system=m.get_srcSystem(),
                         target_component=m.get_srcComponent())
        return


    def mav_cmd_request_camera_information(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        request_information = decoded_message_dict['param1']
        if request_information:
            #Initialize variables in case of failure to access any of these values
            sw_version = "unknown"
            focal_length = 0.0
            res_h = 0
            res_v = 0
            flags = mavutil.mavlink.CAMERA_CAP_FLAGS_CAPTURE_IMAGE | mavutil.mavlink.CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE
            rd = http_get(self.ip, "version")
            if( rd is not None ):
                sw_version = rd['sw_version']
            rd = http_get(self.ip, "camera_info")
            if( rd is not None ):
                focal_length = rd['1']['focal_length_px']
                res_h = rd['1']['image_width']
                res_v = rd['1']['image_height']
                master.mav.camera_information_send(time_boot_ms = 0, 
                                                vendor_name = self.vendor,
                                                model_name = self.model,
                                                firmware_version=1,
                                                focal_length=float(focal_length), 
                                                sensor_size_h=0.0,
                                                sensor_size_v=0.0,
                                                resolution_h=res_h,
                                                resolution_v=res_v,
                                                lens_id=0,
                                                flags=flags,
                                                cam_definition_version=1,
                                                cam_definition_uri=self.camera_url);
            else:
                result = mavutil.mavlink.MAV_RESULT_FAILED
        return(result)


    def mav_cmd_request_camera_settings(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        request_settings = decoded_message_dict['param1']
        if request_settings:
            auto_cap_mode = 'disabled'
            rd = http_get(self.ip, "config")
            if rd is not None:
                auto_cap_mode = rd['auto_cap_mode']
                if auto_cap_mode is "overlap":
                    camera_mode = mavutil.mavlink.CAMERA_MODE_IMAGE_SURVEY
                else:
                    camera_mode = mavutil.mavlink.CAMERA_MODE_IMAGE
                master.mav.camera_settings_send(time_boot_ms=0, 
                                                mode_id=camera_mode, 
                                                zoomLevel=float('nan'), 
                                                focusLevel=float('nan'))
            else:
                result = mavutil.mavlink.MAV_RESULT_FAILED
        return(result)

    def mav_cmd_request_storage_information(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        request_storage = decoded_message_dict['param2']
        if request_storage:
            #Ignore the requested storage id, since there is only one storage source on the camera -- SD card
            status = mavutil.mavlink.STORAGE_STATUS_EMPTY #Assume no SD card inserted until we can get a response from camera
            total_capacity = 0.0
            used_capacity = 0.0
            available_capacity = 0.0
            read_speed = 0.0
            write_speed = 0.0
            rd = http_get(self.ip, "status")
            if( rd is not None ):
                if rd['sd_status'] in ['Full', 'Ok']:
                    status              = mavutil.mavlink.STORAGE_STATUS_READY
                    total_capacity      = rd['sd_gb_total'] * MB_PER_GB
                    available_capacity  = rd['sd_gb_free'] * MB_PER_GB
                    used_capacity       = total_capacity - available_capacity
                if rd['sd_status'] is 'Full':
                    used_capacity = total_capacity
                master.mav.storage_information_send(time_boot_ms = 0,
                                                    storage_id = 1,
                                                    storage_count = 1,
                                                    status = status,
                                                    total_capacity = total_capacity,
                                                    used_capacity = used_capacity,
                                                    available_capacity = available_capacity,
                                                    read_speed = read_speed,
                                                    write_speed = write_speed)
            else:
                result = mavutil.mavlink.MAV_RESULT_FAILED
        return(result)

    def mav_cmd_storage_format(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        format_storage = decoded_message_dict['param2']
        if format_storage:
            http_post(self.ip, "reformatsdcard", {'erase_all_data': True})
            #Since MAVLINK defines that a STORAGE_INFORMATION response message is returned as a consequence of this request,
            #we simply call as if a storage information request command was sent.
            result = self.mav_cmd_request_storage_information(m, master, decoded_message_dict)
        return(result)

    def mav_cmd_request_camera_capture_status(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        capture_status = decoded_message_dict['param1']
        if capture_status:
            image_status = 0 #Assume idle at first
            image_interval = 0.0
            available_capacity = 0.0
            time_since_start = 0
            if( self.cam_interval_timer and self.cam_interval_timer.running ):
                image_status = 3
                image_interval = self.cam_interval_timer.period
                time_since_start = (datetime.now().timestamp() - self.cam_interval_timer.time_started) * 1000
            rd = http_get(self.ip, "status")
            if( rd is not None ):
                if rd['sd_status'] in ['Full', 'Ok']:
                    available_capacity  = rd['sd_gb_free'] * MB_PER_GB
            master.mav.camera_capture_status_send(time_boot_ms = 0,
                                             image_status = image_status, #0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress
                                             video_status = 0, #0: idle, 1: capture in progress
                                             image_interval =  float(image_interval),
                                             recording_time_ms = int(time_since_start),
                                             available_capacity = float(available_capacity))
            #master.mav.camera_image_captured_send(time_boot_ms = 0,
            #                                    time_utc = 0,
            #                                    camera_id = 1,
            #                                    lat = 0,
            #                                    lon = 0,
            #                                    alt = 0,
            #                                    relative_alt = 0,
            #                                    q = [0.0, 0.0, 0.0, 0.0],
            #                                    image_index = 1,
            #                                    capture_result = 0,
            #                                    file_url = "test.jpg".encode('utf-8'))
        return(result)

    def mav_cmd_reset_camera_settings(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        reset = decoded_message_dict['param1']
        if reset:
            result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_set_camera_mode(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        mode = decoded_message_dict['param2']
        if mode in [mavutil.mavlink.CAMERA_MODE_IMAGE, mavutil.mavlink.CAMERA_MODE_IMAGE_SURVEY]:
            if mode == mavutil.mavlink.CAMERA_MODE_IMAGE:
                auto_cap_mode = 'disabled'
            elif mode == mavutil.mavlink.CAMERA_MODE_IMAGE_SURVEY:
                auto_cap_mode = 'overlap'
            rd = http_post(self.ip, "config", {'auto_cap_mode': auto_cap_mode})
            if rd is not None:
                if rd['auto_cap_mode'] != auto_cap_mode:
                    result = mavutil.mavlink.MAV_RESULT_FAILED
            else:
                result = mavutil.mavlink.MAV_RESULT_FAILED
        else:
            result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_set_camera_zoom(self, m, master, decoded_message_dict):
        return(mavutil.mavlink.MAV_RESULT_UNSUPPORTED)

    def mav_cmd_set_camera_focus(self, m, master, decoded_message_dict):
        return(mavutil.mavlink.MAV_RESULT_UNSUPPORTED)

    def mav_cmd_image_capture(self, m, master, decoded_message_dict, image_index = 0):
        '''The internal function that issues an actual capture command'''
        def loop_captured_images(rd, time_utc, files):
            for cam_id in files:
                master.mav.camera_image_captured_send(time_boot_ms = 0, 
                                                    time_utc = time_utc,
                                                    camera_id = cam_id, 
                                                    lat = 0, 
                                                    lon = 0, 
                                                    alt = 0, 
                                                    relative_alt = 0, 
                                                    q = (0,0,0,0), 
                                                    image_index = image_index, 
                                                    capture_result = 1, 
                                                    file_url = 'http://{0}{1}'.format(self.ip,files[cam_id]))

        rd = http_post(self.ip, "capture", {'block': True, 'store_capture': True})
        if rd is not None:
            if rd['status'] == 'complete':
                #Now get the capture status to get the file URLs
                rd = http_get(self.ip, "capture/{0}".format(rd['id']))
                if rd is not None:
                    datetime_utc = datetime.fromisoformat(rd['time'][:-1]).replace(tzinfo=datetime.timezone.utc)
                    time_utc = datetime_utc.timestamp()
                    #Send captured message for all JPEG images
                    loop_captured_images(rd, time_utc, rd['jpeg_storage_path'])
                    #Send captured message for all RAW images
                    loop_captured_images(rd, time_utc, rd['raw_storage_path'])


    def mav_cmd_image_start_capture(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        interval = decoded_message_dict['param2']
        count = decoded_message_dict['param3']
        if( count == 0 ):
            count = TIMER_INFINITE_COUNT
        rd = http_post(self.ip, "config", {'auto_cap_mode': 'disabled'})
        if rd is not None:
            if rd['auto_cap_mode'] != 'disabled':
                result = mavutil.mavlink.MAV_RESULT_FAILED
            else:
                #Stop any previous timers and start a new one
                if( self.cam_interval_timer ):
                    self.cam_interval_timer.stop()
                    self.cam_interval_timer = None
                self.cam_interval_timer = CameraIntervalTimer(self.mav_cmd_image_capture, interval, 'timer_capture', count, m, master, decoded_message_dict)
                if self.cam_interval_timer is None:
                    result = mavutil.mavlink.MAV_RESULT_FAILED
                else:
                    self.cam_interval_timer.start()
        else:
            result = mavutil.mavlink.MAV_RESULT_FAILED
        return(result)

    def mav_cmd_image_stop_capture(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        if( self.cam_interval_timer ):
            self.cam_interval_timer.stop()
            self.cam_interval_timer = None
        return(result)

    def mav_cmd_request_camera_image_capture(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_do_trigger_control(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_video_start_capture(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_video_stop_capture(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_video_start_streaming(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        if( self.cam_stream_timer ):
            self.cam_stream_timer.stop()
            self.cam_stream_timer = None
        #Enable streaming on camera
        http_post(self.ip, 'config', {'streaming_enable': True, 'streaming_allowed': True, 'preview_band': 'multi'})
        #Create pipe
        self.pipe = sp.Popen(self.gst_command, stdin=sp.PIPE)
        self.cam_stream_timer = CameraIntervalTimer(self.mav_stream_capture, 0.5, "video_start_streaming", TIMER_INFINITE_COUNT, m, master, decoded_message_dict, pipe=self.pipe)
        if self.cam_stream_timer is None:
            result = mavutil.mavlink.MAV_RESULT_FAILED
        else:
            self.cam_stream_timer.start()
        return(result)

    def mav_cmd_video_stop_streaming(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED
        if( self.cam_stream_timer ):
            self.cam_stream_timer.cancel()
            self.cam_stream_timer = None
        if( self.pipe is not None ):
            #Close the gstreamer pipe
            self.pipe.kill()
            self.pipe.wait()
        return(result)

    def mav_cmd_request_video_stream_information(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_request_video_stream_status(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_do_control_video(self, m, master, decoded_message_dict):
        result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_stream_capture(self, m, master, decoded_message_dict, **kwargs):
        if ('pipe' in kwargs):
            pipe = kwargs['pipe']

        rd = http_get(self.ip, "jpeg_url")
        if rd is not None:
            jpeg_url = rd['jpeg_url']
            r = requests.get('http://' + self.ip + jpeg_url)
            if(r.status_code == requests.codes.ok):
                stream = io.BytesIO(r.content)
                img = Image.open(stream)
                imgc = img.convert('RGB') #Need to convert to RGB otherwise rtpjpegpay fails. 
                imgc.save(pipe.stdin, 'JPEG')

    def param_ext_request_read(self, m, master, decoded_message_dict):
        if( self.mav_cam_params ):
            param = decoded_message_dict['param_id']
            if( param in self.mav_cam_params.keys() ):
                value = self.mav_cam_params[param]
                param_type = self.mav_cam_params.type(param)
                param_value = param_encode(value, param_type)
                master.mav.param_ext_value_send(param_id = param.encode('utf-8'),
                                                param_value = param_value,
                                                param_type = self.mav_cam_params.type(param),
                                                param_count = 1,
                                                param_index = 0)


        return

    def param_ext_request_list(self, m, master, decoded_message_dict):
        try:
            if( self.mav_cam_params ):
                for param in self.mav_cam_params.keys():
                    decoded_message_dict['param_id'] = param
                    decoded_message_dict['param_index'] = -1
                    self.param_ext_request_read(m, master, decoded_message_dict)
        except Exception as e:
            print("Exception: param_ext_request_list(): {}".format(e))
            pass
        return

    def param_ext_request_set(self, m, master, decoded_message_dict):
        param_ack_status = mavutil.mavlink.PARAM_ACK_VALUE_UNSUPPORTED #Assume guilty until proven innocent
        param = decoded_message_dict['param_id']
        value = decoded_message_dict['param_value']
        ptype = decoded_message_dict['param_type']
        param_value = param_decode(value, param, self.mav_cam_params)
        #print("param_id: {}, type: {}".format(param, type(param)))
        #print("param_value: {}, type: {}".format(param_value, type(param_value)))
        if (self.mav_cam_params) and (param in self.mav_cam_params.keys()) and (ptype == self.mav_cam_params.type(param)):
            self.mav_cam_params[param] = param_value
            param_ack_status = mavutil.mavlink.PARAM_ACK_ACCEPTED
        master.mav.param_ext_ack_send(param_id = param.encode('utf-8'),
                                      param_value = value.encode('utf-8'),
                                      param_type = ptype,
                                      param_result = param_ack_status)
        return


def get_usec():
    '''time since 1970 in microseconds'''
    return int(time() * 1.0e6)


def set_mav_version(mav10, mav20):
    global mavversion
    '''Set the Mavlink version based on commandline options'''
    #sanity check the options
    if mav10 == True and mav20 == True:
        print("Error: Can't have --mav10 and --mav20 both True")
        sys.exit(1)

    #and set the specific mavlink version (False = autodetect)
    if mav10 == True:
        os.environ['MAVLINK09'] = '1'
        mavversion = "1"
    else:
        os.environ['MAVLINK20'] = '1'
        mavversion = "2"

def process_master(m, mavl):
    '''process packets from the MAVLink master'''
    try:
        s = m.recv(16*1024)
    except Exception:
        sleep(0.1)
        return
    # prevent a dead serial port from causing the CPU to spin. The user hitting enter will
    # cause it to try and reconnect
    if len(s) == 0:
        sleep(0.1)
        return

    msgs = m.mav.parse_buffer(s)
    if msgs:
        for msg in msgs:
            if getattr(m, '_timestamp', None) is None:
                m.post_message(msg)

def main():
    global mav_component_ids
    global mav_cam_cmds
    global mav_cam_input_messages
    global mav_param_ext_types
    global exit
    exit = False
    (opts, args) = parse_args()
    if len(args) != 0:
          print("ERROR: %s takes no position arguments; got (%s)" % (sys.argv[0],str(args)))
          sys.exit(1)
    set_mav_version(opts.mav10, opts.mav20)
    mavutil.set_dialect(opts.dialect)
    #import mavutil.mavlink import * #Import all parameter names, enumerations, etc., associated with this mavlink protocol
    #Relevant mavlink component ids for this module
    mav_component_ids = [mavutil.mavlink.MAV_COMP_ID_ALL,
                                mavutil.mavlink.MAV_COMP_ID_CAMERA,
                                mavutil.mavlink.MAV_COMP_ID_CAMERA2,
                                mavutil.mavlink.MAV_COMP_ID_CAMERA3,
                                mavutil.mavlink.MAV_COMP_ID_CAMERA4,
                                mavutil.mavlink.MAV_COMP_ID_CAMERA5,
                                mavutil.mavlink.MAV_COMP_ID_CAMERA6]

    #Relevant Camera commands issued by GCS system and corresponding functions to parse them
    mav_cam_cmds = {
                        mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION: "mav_cmd_request_camera_information",
                        mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_SETTINGS: "mav_cmd_request_camera_settings",
                        mavutil.mavlink.MAV_CMD_REQUEST_STORAGE_INFORMATION: "mav_cmd_request_storage_information",
                        mavutil.mavlink.MAV_CMD_STORAGE_FORMAT: "mav_cmd_storage_format",
                        mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS: "mav_cmd_request_camera_capture_status",
                        mavutil.mavlink.MAV_CMD_RESET_CAMERA_SETTINGS: "mav_cmd_reset_camera_settings",
                        mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE: "mav_cmd_set_camera_mode",
                        mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM: "mav_cmd_set_camera_zoom",
                        mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS: "mav_cmd_set_camera_focus",
                        mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE: "mav_cmd_image_start_capture",
                        mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE: "mav_cmd_image_stop_capture",
                        mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE: "mav_cmd_request_camera_image_capture",
                        mavutil.mavlink.MAV_CMD_DO_TRIGGER_CONTROL: "mav_cmd_do_trigger_control",
                        mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE: "mav_cmd_video_start_capture",
                        mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE: "mav_cmd_video_stop_capture",
                        mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING: "mav_cmd_video_start_streaming",
                        mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING: "mav_cmd_video_stop_streaming",
                        mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION: "mav_cmd_request_video_stream_information",
                        mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS: "mav_cmd_request_video_stream_status",
                        mavutil.mavlink.MAV_CMD_DO_CONTROL_VIDEO: "mav_cmd_do_control_video"
                        }


    mav_cam_input_messages = {
                                    "COMMAND_LONG": "mav_cmd_long",
                                    "PARAM_EXT_REQUEST_READ": "param_ext_request_read",
                                    "PARAM_EXT_REQUEST_LIST": "param_ext_request_list",
                                    "PARAM_EXT_SET": "param_ext_request_set"
                                    }

    mav_param_ext_types = {
                        "bool": {'type':    mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT8, 'default': 0, 'castfunc': int},
                        "uint8": {'type':   mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT8, 'default': 0, 'castfunc': int},
                        "int8": {'type':    mavutil.mavlink.MAV_PARAM_EXT_TYPE_INT8, 'default': 0, 'castfunc': int},
                        "uint16": {'type':  mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT16, 'default': 0, 'castfunc': int},
                        "int16": {'type':   mavutil.mavlink.MAV_PARAM_EXT_TYPE_INT16, 'default': 0, 'castfunc': int},
                        "uint32": {'type':  mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT32, 'default': 0, 'castfunc': int},
                        "int32": {'type':   mavutil.mavlink.MAV_PARAM_EXT_TYPE_INT32, 'default': 0, 'castfunc': int},
                        "uint64": {'type':  mavutil.mavlink.MAV_PARAM_EXT_TYPE_UINT64, 'default': 0, 'castfunc': int},
                        "int64": {'type':   mavutil.mavlink.MAV_PARAM_EXT_TYPE_INT64, 'default': 0, 'castfunc': int},
                        "float": {'type':   mavutil.mavlink.MAV_PARAM_EXT_TYPE_REAL32, 'default': 0.0, 'castfunc': float},
                        "double": {'type':  mavutil.mavlink.MAV_PARAM_EXT_TYPE_REAL64, 'default': 0.0, 'castfunc': float},
                        "custom": {'type':  mavutil.mavlink.MAV_PARAM_EXT_TYPE_CUSTOM, 'default': None, 'castfunc': int }
                        }

    #version information
    if opts.version:
        #pkg_resources doesn't work in the windows exe build, so read the version file
        try:
            import pkg_resources
            version = pkg_resources.require("micamavproxy")[0].version
        except:
            start_script = os.path.join(os.environ['LOCALAPPDATA'], "micamavproxy", "version.txt")
            f = open(start_script, 'r')
            version = f.readline()

        print("Micamavproxy is a Micasense RedEdge MX Camera HTTPApi/Mavlink Proxy")
        print("Micamavproxy Version: " + version)
        sys.exit(1)

    def quit_handler(signum = None, frame = None):
        global exit
        print('Signal handler called with signal', signum)
        if exit:
            print('Clean shutdown impossible, forcing an exit')
            sys.exit(0)
        else:
            exit = True

    # Listen for kill signals to cleanly shutdown modules
    fatalsignals = [signal.SIGTERM, signal.SIGINT]
    try:
        fatalsignals.append(signal.SIGHUP)
        fatalsignals.append(signal.SIGQUIT)
    except Exception:
        pass
    for sig in fatalsignals:
        signal.signal(sig, quit_handler)

    if not opts.master:
        descriptor = '0.0.0.0:14551'
    else:
        descriptor = opts.master
    if( not opts.SOURCE_COMPONENT ):
        source_component = mavutil.mavlink.MAV_COMP_ID_CAMERA
    else:
        source_component = opts.SOURCE_COMPONENT

    mavl = MicaMavLink(opts.ip, opts.mav10, opts.mav20, opts.SOURCE_SYSTEM, source_component, opts.TARGET_SYSTEM, opts.TARGET_COMPONENT, opts.rtscts, opts.baudrate, descriptor, opts.stream_ip, opts.stream_port, camera_defs=opts.camera_defs_local, camera_url=opts.camera_defs_url, log_file=opts.log_file, isMulticast = opts.multicast)

    #Main Loop
    while exit == False:
        mavl.heartbeat_trigger()
        mavl.read(timeout=1.0)

    #Cleanup
    mavl.shutdown()

if __name__ == "__main__":
    main()
