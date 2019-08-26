#!/usr/bin/env python
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

from concurrent.futures import *
from __future__ import print_function
import json
from mavutil.mavlink import *
from optparse import OptionParser
from pymavlink import mavutil, mavparm
import requests
import signal
import struct
import sys
from threading import Timer
from time import time
import xml.etree.ElementTree as ET

MB_PER_GB = 1024 #Number of megabytes per gigabyte

#Relevant mavlink component ids for this module
global mav_component_ids = [MAV_COMP_ID_ALL,
                            MAV_COMP_ID_CAMERA,
                            MAV_COMP_ID_CAMERA2,
                            MAV_COMP_ID_CAMERA3,
                            MAV_COMP_ID_CAMERA4
                            MAV_COMP_ID_CAMERA5,
                            MAV_COMP_ID_CAMERA6]

#Relevant Camera commands issued by GCS system and corresponding functions to parse them
global mav_cam_cmds = {
                       MAV_CMD_REQUEST_CAMERA_INFORMATION: "mav_cmd_request_camera_information",
                       MAV_CMD_REQUEST_CAMERA_SETTINGS: "mav_cmd_request_camera_settings",
                       MAV_CMD_REQUEST_STORAGE_INFORMATION: "mav_cmd_request_storage_information",
                       MAV_CMD_STORAGE_FORMAT: "mav_cmd_storage_format",
                       MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS: "mav_cmd_request_camera_capture_status",
                       MAV_CMD_RESET_CAMERA_SETTINGS: "mav_cmd_reset_camera_settings",
                       MAV_CMD_SET_CAMERA_MODE: "mav_cmd_set_camera_mode",
                       MAV_CMD_SET_CAMERA_ZOOM: "mav_cmd_set_camera_zoom",
                       MAV_CMD_SET_CAMERA_FOCUS: "mav_cmd_set_camera_focus",
                       MAV_CMD_IMAGE_START_CAPTURE: "mav_cmd_image_start_capture",
                       MAV_CMD_IMAGE_STOP_CAPTURE: "mav_cmd_image_stop_capture",
                       MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE: "mav_cmd_request_camera_image_capture",
                       MAV_CMD_DO_TRIGGER_CONTROL: "mav_cmd_do_trigger_control",
                       MAV_CMD_VIDEO_START_CAPTURE: "mav_cmd_video_start_capture",
                       MAV_CMD_VIDEO_STOP_CAPTURE: "mav_cmd_video_stop_capture",
                       MAV_CMD_VIDEO_START_STREAMING: "mav_cmd_video_start_streaming",
                       MAV_CMD_VIDEO_STOP_STREAMING: "mav_cmd_video_stop_streaming",
                       MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION: "mav_cmd_request_video_stream_information",
                       MAV_CMD_REQUEST_VIDEO_STREAM_STATUS: "mav_cmd_request_video_stream_status",
                       MAV_CMD_DO_CONTROL_VIDEO: "mav_cmd_do_control_video"
                      }


global mav_cam_input_messages = {
                                 MAV_CMD_LONG, "mav_cmd_long",
                                 PARAM_EXT_REQUEST_READ, "param_ext_request_read",
                                 PARAM_EXT_REQUEST_LIST, "param_ext_request_list",
                                 PARAM_EXT_SET, "param_ext_request_set"
                                }



def parse_args():
    parser = OptionParser("micamavproxy.py [options]")

    parser.add_option("--master", dest="master", action='append',
                      metavar="DEVICE", help="MAVLink master port and optional baud rate",
                      default=[])
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="default serial baud rate", default=57600)
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=1, help='MAVLink camera source system for this vehicle -- Defaults to autopilot vehicle')
    parser.add_option("--source-component", dest='SOURCE_COMPONENT', type='int',
                      default=mavutil.mavlink.MAV_COMP_ID_CAMERA, help='MAVLink source component for this GCS')
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=0, help='MAVLink target master system')
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=0, help='MAVLink target master component')
    parser.add_option("--camera_defs", default="micasense_rededge_mx.xml", help="If extra parameters are supported for this camera, this is the location of the camera definitions file (XML).")
    parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
    parser.add_option("--mav20", action='store_true', default=True, help="Use MAVLink protocol 2.0")
    parser.add_option("--version", action='store_true', help="version information")
    parser.add_option("--dialect",  default="common", help="MAVLink dialect")
    parser.add_option("--rtscts",  action='store_true', help="enable hardware RTS/CTS flow control")


class RedEdgeAPISession():
    def __init__(self, ip, mavl):
        self.ip         = ip
        self.mavl       = mavl
        self.session    = ElapsedFutureSession()
        return

    def get_capabilities(self):
        return struct.pack('<I', CAMERA_CAP_FLAGS_CAPTURE_IMAGE | CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE)

    def get_network_status(self, callback):
        self.session.request('get', url="http://%s/networkstatus" % self.ip, hooks={'response': [callback]})
        return

def http_post(ip, route, payload):
    rd = None
    try:
        #Insert a timeout, or can hang indefinitely
        r = requests.post("http://{0}/{1}".format(self.ip,route), data=payload, timeout=5.0)
    except requests.exceptions.Timeout:
        pass
    else:
        if(r.status_code == requests.codes.ok):
            try:
                rd = r.json()
            except ValueError:: 
                pass
    return(rd)

def http_get(ip, route):
    rd = None
    try:
        #Insert a timeout, or can hang indefinitely
        r = requests.get("http://{0}/{1}".format(self.ip,route), timeout=5.0)
    except requests.exceptions.Timeout:
        pass
    else:
        if(r.status_code == requests.codes.ok):
            try:
                rd = r.json()
            except ValueError:: 
                pass
    return(rd)


class CameraIntervalTimer(object):
    def __init__(self, interval, function, count, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.image_index = 0
        self.count      = count
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.count = self.count - 1
        if( self.count >= 0 ):
            self.start()
            self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

mav_param_ext_types = {
                       "bool": MAV_PARAM_EXT_TYPE_UINT8,
                       "uint8": MAV_PARAM_EXT_TYPE_UINT8,
                       "int8": MAV_PARAM_EXT_TYPE_INT8,
                       "uint16": MAV_PARAM_EXT_TYPE_UINT16,
                       "int16": MAV_PARAM_EXT_TYPE_INT16,
                       "uint32": MAV_PARAM_EXT_TYPE_UINT32,
                       "int32": MAV_PARAM_EXT_TYPE_INT32,
                       "uint64": MAV_PARAM_EXT_TYPE_UINT64,
                       "int64": MAV_PARAM_EXT_TYPE_INT64,
                       "float": MAV_PARAM_EXT_TYPE_REAL32,
                       "double": MAV_PARAM_EXT_TYPE_REAL64,
                       "custom": MAV_PARAM_EXT_TYPE_CUSTOM
                      }

class MavCamParams():
    def __init__(self, camera_defs):
        self.params             = {} #Store the camera extra parameters stored in the camera definitions xml file.
        self._model             = ""
        self._vendor            = ""
        self.cam_defs_tree      = ET.parse(camera_defs) #Parse the mavlink camera definitions xml file to populate a dictionary of parameters that we will track/store
        self.populate_ext_params(self.cam_defs_etree)

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
                        p.attrib['mtype'] = mav_param_ext_types[ptype]
                self.params[p.tag] = {'attrib':p.attrib, 'property': '_{0}_'.format(p.tag)}
        return

    def keys(self):
        return(self.params.keys())

    def __getitem__(self, key):
        return getattr(self, '_{0}_'.format(key))

    def __setitem__(self, key, value):
        return setattr(self, '_{0}_'.format(key), value)

    def type(self, key):
        return(self.params[key]['attrib']['mtype'])

    @property
    def model(self):
        return(self._model)

    @property
    def vendor(self):
        return(self._vendor)

    #Extra parameters getters/setters
    @property
    def _CAM_EXP_MAN_(self):
        return(self.params["CAM_EXP_MAN"]['attrib']['value'])

    @_CAM_EXP_MAN_.setter
    def _CAM_EXP_MAN_(self, value):
        self.params["CAM_EXP_MAN"]['attrib']['value'] = value


    @property
    def _CAM_DET_PAN_(self):
        return(self.params["CAM_DET_PAN"]['attrib']['value'])

    @_CAM_DET_PAN_.setter
    def _CAM_DET_PAN_(self, value):
        self.params["CAM_DET_PAN"]['attrib']['value'] = value


    @property
    def _CAM_EXP1_(self):
        return(self.params["CAM_EXP1"]['attrib']['value'])

    @_CAM_EXP1_.setter
    def _CAM_EXP1_(self, value):
        self.params["CAM_EXP1"]['attrib']['value'] = value


    @property
    def _CAM_EXP2_(self):
        return(self.params["CAM_EXP2"]['attrib']['value'])

    @_CAM_EXP2_.setter
    def _CAM_EXP2_(self, value):
        self.params["CAM_EXP2"]['attrib']['value'] = value


    @property
    def _CAM_EXP3_(self):
        return(self.params["CAM_EXP3"]['attrib']['value'])

    @_CAM_EXP3_.setter
    def _CAM_EXP3_(self, value):
        self.params["CAM_EXP3"]['attrib']['value'] = value


    @property
    def _CAM_EXP4_(self):
        return(self.params["CAM_EXP4"]['attrib']['value'])

    @_CAM_EXP4_.setter
    def _CAM_EXP4_(self, value):
        self.params["CAM_EXP4"]['attrib']['value'] = value


    @property
    def _CAM_EXP5_(self):
        return(self.params["CAM_EXP5"]['attrib']['value'])

    @_CAM_EXP5_.setter
    def _CAM_EXP5_(self, value):
        self.params["CAM_EXP5"]['attrib']['value'] = value


    @property
    def _CAM_EXP_GAIN1_(self):
        return(self.params["CAM_EXP_GAIN1"]['attrib']['value'])

    @_CAM_EXP_GAIN1_.setter
    def _CAM_EXP_GAIN1_(self, value):
        self.params["CAM_EXP_GAIN1"]['attrib']['value'] = value


    @property
    def _CAM_EXP_GAIN2_(self):
        return(self.params["CAM_EXP_GAIN2"]['attrib']['value'])

    @_CAM_EXP_GAIN2_.setter
    def _CAM_EXP_GAIN2_(self, value):
        self.params["CAM_EXP_GAIN2"]['attrib']['value'] = value


    @property
    def _CAM_EXP_GAIN3_(self):
        return(self.params["CAM_EXP_GAIN3"]['attrib']['value'])

    @_CAM_EXP_GAIN3_.setter
    def _CAM_EXP_GAIN3_(self, value):
        self.params["CAM_EXP_GAIN3"]['attrib']['value'] = value


    @property
    def _CAM_EXP_GAIN4_(self):
        return(self.params["CAM_EXP_GAIN4"]['attrib']['value'])

    @_CAM_EXP_GAIN4_.setter
    def _CAM_EXP_GAIN4_(self, value):
        self.params["CAM_EXP_GAIN4"]['attrib']['value'] = value


    @property
    def _CAM_EXP_GAIN5_(self):
        return(self.params["CAM_EXP_GAIN5"]['attrib']['value'])

    @_CAM_EXP_GAIN5_.setter
    def _CAM_EXP_GAIN5_(self, value):
        self.params["CAM_EXP_GAIN5"]['attrib']['value'] = value


    @property
    def _CAMP_(self):
        return(self.params["CAMP"]['attrib']['value'])

    @_CAMP_.setter
    def _CAMP_(self, value):
        self.params["CAMP"]['attrib']['value'] = value


    @property
    def _CAM_AC_PHI_(self):
        return(self.params["CAM_AC_PHI"]['attrib']['value'])

    @_CAM_AC_PHI_.setter
    def _CAM_AC_PHI_(self, value):
        self.params["CAM_AC_PHI"]['attrib']['value'] = value


    @property
    def _CAM_AC_THETA_(self):
        return(self.params["CAM_AC_THETA"]['attrib']['value'])

    @_CAM_AC_THETA_.setter
    def _CAM_AC_THETA_(self, value):
        self.params["CAM_AC_THETA"]['attrib']['value'] = value


    @property
    def _CAM_AC_PSI_(self):
        return(self.params["CAM_AC_PSI"]['attrib']['value'])

    @_CAM_AC_PSI_.setter
    def _CAM_AC_PSI_(self, value):
        self.params["CAM_AC_PSI"]['attrib']['value'] = value


    @property
    def _CAM_CAM_PHI_(self):
        return(self.params["CAM_CAM_PHI"]['attrib']['value'])

    @_CAM_CAM_PHI_.setter
    def _CAM_CAM_PHI_(self, value):
        self.params["CAM_CAM_PHI"]['attrib']['value'] = value


    @property
    def _CAM_CAM_THETA_(self):
        return(self.params["CAM_CAM_THETA"]['attrib']['value'])

    @_CAM_CAM_THETA_.setter
    def _CAM_CAM_THETA_(self, value):
        self.params["CAM_CAM_THETA"]['attrib']['value'] = value


    @property
    def _CAM_CAM_PSI_(self):
        return(self.params["CAM_CAM_PSI"]['attrib']['value'])

    @_CAM_CAM_PSI_.setter
    def _CAM_CAM_PSI_(self, value):
        self.params["CAM_CAM_PSI"]['attrib']['value'] = value


    @property
    def _CAM_ALT_(self):
        return(self.params["CAM_ALT"]['attrib']['value'])

    @_CAM_ALT_.setter
    def _CAM_ALT_(self, value):
        self.params["CAM_ALT"]['attrib']['value'] = value


    @property
    def _CAM_A_OVERLAP_(self):
        return(self.params["CAM_A_OVERLAP"]['attrib']['value'])

    @_CAM_A_OVERLAP_.setter
    def _CAM_A_OVERLAP_(self, value):
        self.params["CAM_A_OVERLAP"]['attrib']['value'] = value


    @property
    def _CAM_X_OVERLAP_(self):
        return(self.params["CAM_X_OVERLAP"]['attrib']['value'])

    @_CAM_X_OVERLAP_.setter
    def _CAM_X_OVERLAP_(self, value):
        self.params["CAM_X_OVERLAP"]['attrib']['value'] = value


    @property
    def _CAM_AUTO_CAP_(self):
        return(self.params["CAM_AUTO_CAP"]['attrib']['value'])

    @_CAM_AUTO_CAP_.setter
    def _CAM_AUTO_CAP_(self, value):
        self.params["CAM_AUTO_CAP"]['attrib']['value'] = value


    @property
    def _CAM_INT_TIMER_(self):
        return(self.params["CAM_INT_TIMER"]['attrib']['value'])

    @_CAM_INT_TIMER_.setter
    def _CAM_INT_TIMER_(self, value):
        self.params["CAM_INT_TIMER"]['attrib']['value'] = value


    @property
    def _CAM_EXT_TRIG_MOD_(self):
        return(self.params["CAM_EXT_TRIG_MOD"]['attrib']['value'])

    @_CAM_EXT_TRIG_MOD_.setter
    def _CAM_EXT_TRIG_MOD_(self, value):
        self.params["CAM_EXT_TRIG_MOD"]['attrib']['value'] = value


    @property
    def _CAM_PWM_THR_(self):
        return(self.params["CAM_PWM_THR"]['attrib']['value'])

    @_CAM_PWM_THR_.setter
    def _CAM_PWM_THR_(self, value):
        self.params["CAM_PWM_THR"]['attrib']['value'] = value


    @property
    def _CAM_RAW_FORMAT_(self):
        return(self.params["CAM_RAW_FORMAT"]['attrib']['value'])

    @_CAM_RAW_FORMAT_.setter
    def _CAM_RAW_FORMAT_(self, value):
        self.params["CAM_RAW_FORMAT"]['attrib']['value'] = value


    @property
    def _CAM_AGC_MIN_MEAN_(self):
        return(self.params["CAM_AGC_MIN_MEAN"]['attrib']['value'])

    @_CAM_AGC_MIN_MEAN_.setter
    def _CAM_AGC_MIN_MEAN_(self, value):
        self.params["CAM_AGC_MIN_MEAN"]['attrib']['value'] = value




class MavLink:
    def __init__(self, cam_ip, mav10, mav20, source_system, source_component, target_system, target_component, rtscts, baudrate, descriptor, camera_defs="micasense_rededge_mx.xml", max_workers=10):
        self.source_system      = source_system
        self.source_component   = source_component
        self.target_system      = target_system
        self.target_component   = target_component
        self.descriptor         = descriptor 
        self.baudrate           = baudrate
        self.rtscts             = rtscts
        self.set_mav_version(mav10, mav20)
        self.link_add(descriptor)
        self.pool               = ThreadPoolExecutor(max_workers=max_workers)
        self.mav_cam_params     = MavCamParams(camera_defs)

    def set_mav_version(self, mav10, mav20):
        '''Set the Mavlink version based on commandline options'''
        #sanity check the options
        if mav10 == True and mav20 == True:
            print("Error: Can't have --mav10 and --mav20 both True")
            sys.exit(1)

        #and set the specific mavlink version (False = autodetect)
        if mav10 == True:
            os.environ['MAVLINK09'] = '1'
            self.mavversion = "1"
        else:
            os.environ['MAVLINK20'] = '1'
            self.mavversion = "2"

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
#        if self.target_system != 0 and m.get_srcSystem() != self.target_system:
#            # don't process messages not from our target
#            if m.get_type() == "BAD_DATA":
#                if self.mpstate.settings.shownoise and mavutil.all_printable(m.data):
#                    out = m.data
#                    if type(m.data) == bytearray:
#                        out = m.data.decode('ascii')
#                    #self.mpstate.console.write(out, bg='red')
#            return

        if self.target_system != 0 and master.target_system != self.target_system:
            # keep the pymavlink level target system aligned with the MAVProxy setting
            print("change target_system %u" % self.target_system)
            master.target_system = self.target_system

        if self.target_component != MAV_COMP_ID_ALL and master.target_component != self.target_component:
            # keep the pymavlink level target component aligned with the MAVProxy setting
            print("change target_component %u" % self.target_component)
            master.target_component = self.target_component
            
        mtype = m.get_type()
        if( mtype in mav_cam_input_messages ):
            decoded_message_dict = self.mav_decode(m, master)
            #If the target_system and target_component are embedded in a message, then check that they are valid for this component
            if ('target_system' in decoded_message_dict) and ('target_component' in decoded_message_dict):
                if (target_system == self.source_system) and (target_component == self.source_component):
                    self.pool.submit(getattr(self, mav_cam_input_messages[mtype]), m, master, decoded_message_dict)
            else:
                self.pool.submit(getattr(self, mav_cam_input_messages[mtype]), m, master, decoded_message_dict)


    def master_callback(self, m, master):
        '''process mavlink message m on master, sending any messages to recipients'''

        # see if it is handled by a specialised sysid connection
        sysid = m.get_srcSystem()
        mtype = m.get_type()

        if getattr(m, '_timestamp', None) is None:
            master.post_message(m)

        self.master_msg_handling(m, master)

    def mav_decode(self, m, master):
        b = m.get_msgbuf()
        dm = master.mav.decode(b)
        dmd = dm.to_dict()
        return(dmd)


    def mav_cmd_long(self, m, master, decoded_message_dict):
        result = getattr(self, dmd['command'])(m, master, decoded_message_dict)
        command_ack_send(command=decoded_message_dict['command'],
                         result=result,
                         target_system=m.get_srcSystem(),
                         target_component=m.get_srcComponent())
        return


    def mav_cmd_request_camera_information(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        request_information = decoded_message_dict['param1']
        if request_information:
            #Initialize variables in case of failure to access any of these values
            sw_version = "unknown"
            focal_length = 0.0
            res_h = 0
            res_v = 0
            flags = struct.pack('<I', CAMERA_CAP_FLAGS_CAPTURE_IMAGE | CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE)
            rd = http_get(self.ip, "networkstatus")
            if( rd is not None ):
                sw_version = rd['network_map'][0]['sw_version']
            rd = http_get(self.ip, "camera_info")
            if( rd is not None ):
                focal_length = rd['1']['focal_length_px']
                res_h = rd['1']['image_width']
                res_v = rd['1']['image_height']
                master.mav.camera_information_send(time_boot_ms = 0, 
                                                vendor_name = "MicaSense", 
                                                model_name="RedEdgeMX", 
                                                firmware_version=sw_version,
                                                focal_length=focal_length, 
                                                sensor_size_h=0,
                                                sensor_size_v=0,
                                                resolution_h=res_h,
                                                resolution_v=res_v,
                                                lens_id=0x0,
                                                flags=flags,
                                                cam_definition_version="1",
                                                cam_definition_uri="micasense_rededge_mx.xml");
            else:
                result = MAV_RESULT_FAILED
        return(result)


    def mav_cmd_request_camera_settings(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        request_settings = decoded_message_dict['param1']
        if request_settings:
            auto_cap_mode = 'disabled'
            rd = http_get(self.ip, "config")
            if rd is not None:
                auto_cap_mode = rd['auto_cap_mode']
                if auto_cap_mode is "overlap":
                    camera_mode = CAMERA_MODE_IMAGE_SURVEY
                else:
                    camera_mode = CAMERA_MODE_IMAGE
                master.mav.camera_settings_send(time_boot_ms=0, 
                                                mode_id=camera_mode, 
                                                zoomLevel=float('nan'), 
                                                focusLevel=float('nan'))
            else:
                result = MAV_RESULT_FAILED
        return(result)

    def mav_cmd_request_storage_information(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        request_storage = decoded_message_dict['param2']
        if request_storage:
            #Ignore the requested storage id, since there is only one storage source on the camera -- SD card
            status = STORAGE_STATUS_EMPTY #Assume no SD card inserted until we can get a response from camera
            total_capacity = 0.0
            used_capacity = 0.0
            available_capacity = 0.0
            read_speed = 0.0
            write_speed = 0.0
            rd = http_get(self.ip, "status")
            if( rd is not None ):
                if rd['sd_status'] in ['Full', 'Ok']:
                    status              = STORAGE_STATUS_READY
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
                result = MAV_RESULT_FAILED
        return(result)

    def mav_cmd_storage_format(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        format_storage = decoded_message_dict['param2']
        if format_storage:
            http_post(self.ip, "reformatsdcard", {'erase_all_data': True})
            #Since MAVLINK defines that a STORAGE_INFORMATION response message is returned as a consequence of this request,
            #we simply call as if a storage information request command was sent.
            self.mav_cmd_request_storage_information(m, master, decoded_message_dict)
        return(result)

    def mav_cmd_request_camera_capture_status(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        capture_status = decoded_message_dict['param1']
        if capture_status:
            master.mav.camera_image_captured_send(time_boot_ms = 0,
                                                time_utc = 0,
                                                camera_id = 1,
                                                lat = 0,
                                                lon = 0,
                                                alt = 0,
                                                relative_alt = 0,
                                                q = [0.0, 0.0, 0.0, 0.0],
                                                image_index = 0,
                                                capture_result = 0,
                                                file_url = "")
        return(result)

    def mav_cmd_reset_camera_settings(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        reset = decoded_message_dict['param1']
        if reset:
            result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_set_camera_mode(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        mode = decoded_message_dict['param2']
        if mode in [CAMERA_MODE_IMAGE, CAMERA_MODE_IMAGE_SURVEY]:
            if mode == CAMERA_MODE_IMAGE:
                auto_cap_mode = 'disabled'
            elif mode == CAMERA_MODE_IMAGE_SURVEY:
                auto_cap_mode = 'overlap'
            rd = http_post(self.ip, "config", {'auto_cap_mode': auto_cap_mode})
            if rd is not None:
                if rd['auto_cap_mode'] != auto_cap_mode:
                    result = MAV_RESULT_FAILED
            else:
                result = MAV_RESULT_FAILED
        else:
            result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_set_camera_zoom(self, m, master, decoded_message_dict):
        return(MAV_RESULT_UNSUPPORTED)

    def mav_cmd_set_camera_focus(self, m, master, decoded_message_dict):
        return(MAV_RESULT_UNSUPPORTED)

    def mav_cmd_image_capture(self, m, master, decoded_message_dict, image_index = 0):
        '''The internal function that issues an actual capture command'''
        def loop_captured_images(files):
            for cam_id in files:
                master.mav.camera_image_captured_send(time_boot_ms = 0, 
                                                    time_utc = datetime.datetime.fromisoformat(rd['time'][:-1]) * 10000000.0, 
                                                    camera_id = cam_id, 
                                                    lat = 0, 
                                                    lon = 0, 
                                                    alt = 0, 
                                                    relative_alt 0, 
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
                    #Send captured message for all JPEG images
                    loop_captured_images(rd['jpeg_storage_path'])
                    #Send captured message for all RAW images
                    loop_captured_images(rd['raw_storage_path'])


    def mav_cmd_image_start_capture(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        interval = decoded_message_dict['param2']
        count = decoded_message_dict['param3']
        rd = http_post(self.ip, "config", {'auto_cap_mode': 'disabled'})
        if rd is not None:
            if rd['auto_cap_mode'] != 'disabled':
                result = MAV_RESULT_FAILED
            else:
                #Stop any previous timers and start a new one
                if( self.camera_interval_timer ):
                    self.cam_interval_timer.stop()
                    self.camera_interval_timer = None
                self.cam_interval_timer = CameraIntervalTimer(interval, self.mav_cmd_image_capture, m, master, decoded_message_dict)
                if self.cam_interval_timer is None:
                    result = MAV_RESULT_FAILED
        else:
            result = MAV_RESULT_FAILED
        return(result)

    def mav_cmd_image_stop_capture(self, m, master, decoded_message_dict):
        result = MAV_RESULT_ACCEPTED
        if( self.cam_interval_timer ):
            self.cam_interval_timer.stop()
            self.cam_interval_timer = None
        return(result)

    def mav_cmd_request_camera_image_capture(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_do_trigger_control(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_video_start_capture(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_video_stop_capture(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_video_start_streaming(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_video_stop_streaming(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_request_video_stream_information(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_request_video_stream_status(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def mav_cmd_do_control_video(self, m, master, decoded_message_dict):
        result = MAV_RESULT_UNSUPPORTED
        return(result)

    def param_ext_ack(self,m,master):
        return

    def param_ext_request_read(self, m, master, decoded_message_dict):
        if( self.mav_cam_params ):
            param = decoded_message_dict['param_id']
            if( param in self.mav_cam_params ):
                value = self.mav_cam_params[param]
                master.mav.param_ext_value_send(param_id = param,
                                                param_value = value,
                                                param_type = self.mav_cam_params.type(param),
                                                param_count = 1,
                                                param_index = -1)


        return

    def param_ext_request_list(self, m, master, decoded_message_dict):
        if( self.mav_cam_params ):
            for param in self.mav_cam_params.keys():
                decoded_message_dict['param_id'] = param
                decoded_message_dict['param_id'] = -1
                self.param_ext_request_read(m, master, decoded_message_dict)
        return

    def param_ext_request_set(self,m,master):
        param_ack_status = PARAM_ACK_VALUE_UNSUPPORTED #Assume guilty until proven innocent
        param = decoded_message_dict['param_id']
        value = decoded_message_dict['param_value']
        ptype = decoded_message_dict['param_type']
        if (self.mav_cam_params) and (param in self.mav_cam_params) and (ptype == self.mav_cam_params.type(param)):
            self.mav_cam_params[param] = value
            param_ack_status = PARAM_ACK_ACCEPTED
        master.mav.param_ext_ack_send(param_id = param,
                                      param_value = value,
                                      param_type = ptype,
                                      param_result = param_ack_status)

        return


if __name__ == "__main__":
    exit = False
    (opts, args) = parse_args()
    if len(args) != 0:
          print("ERROR: %s takes no position arguments; got (%s)" % (sys.argv[0],str(args)))
          sys.exit(1)
    set_mav_version(opts.mav10, opts.mav20)
    mavutil.set_dialect(opts.dialect)
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
        #print('Signal handler called with signal', signum)
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
        descriptor = '0.0.0.0:14550'
    else:
        descriptor = opts.master
    mavl = MavLink(opts.mav10, opts.mav20, opts.SOURCE_SYSTEM, opts.SOURCE_COMPONENT, opts.TARGET_SYSTEM, opts.TARGET_COMPONENT, opts.rtscts, opts.baudrate, descriptor)

    #Main Loop
    while exit == False:

    #Cleanup

