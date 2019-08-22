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

import json
from __future__ import print_function
from optparse import OptionParser
from pymavlink import mavutil, mavparm
from mavutil.mavlink import *
import signal
import sys
import time

#Relevant mavlink component ids for this module
global mav_component_ids = [MAV_COMP_ID_ALL,
                            MAV_COMP_ID_CAMERA,
                            MAV_COMP_ID_CAMERA2,
                            MAV_COMP_ID_CAMERA3,
                            MAV_COMP_ID_CAMERA4
                            MAV_COMP_ID_CAMERA5,
                            MAV_COMP_ID_CAMERA6]

#Camera bitflags
global cam_cap_flags = [CAMERA_CAP_FLAGS_CAPTURE_VIDEO,
                        CAMERA_CAP_FLAGS_CAPTURE_IMAGE,
                        CAMERA_CAP_FLAGS_HAS_MODES,
                        CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE,
                        CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE,
                        CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE,
                        CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM,
                        CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS,
                        CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM]
#Camera storage status
global mav_storage_statuses = [ STORAGE_STATUS_EMPTY,
                                STORAGE_STATUS_UNFORMATTED,
                                STORAGE_STATUS_READY,
                                STORAGE_STATUS_NOT_SUPPORTED]

#Relevant Camera commands issued by GCS
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

#Valid camera zoom type options in MAV_CMD_SET_CAMERA_ZOOM
global mav_camera_zoom_types = [ZOOM_TYPE_STEP,
                                ZOOM_TYPE_CONTINUOUS,
                                ZOOM_TYPE_RANGE]


#Valid focus types for MAV_CMD_SET_CAMERA_FOCUS
global mav_camera_focus_types = [FOCUS_TYPE_STEP,
                                 FOCUS_TYPE_CONTINUOUS,
                                 FOCUS_TYPE_RANGE]

#Valid camera modes for MAV_CMD_SET_CAMERA_MODE
global mav_camera_modes = [CAMERA_MODE_IMAGE,
                           CAMERA_MODE_VIDEO,
                           CAMERA_MODE_IMAGE_SURVEY]


def parse_args():
    parser = OptionParser("mavproxy.py [options]")

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
    parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
    parser.add_option("--mav20", action='store_true', default=True, help="Use MAVLink protocol 2.0")
    parser.add_option("--version", action='store_true', help="version information")
    parser.add_option("--dialect",  default="common", help="MAVLink dialect")
    parser.add_option("--rtscts",  action='store_true', help="enable hardware RTS/CTS flow control")

class MavLink:
    def __init__(self, mav10, mav20, source_system, source_component, target_system, target_component, rtscts, baudrate, descriptor):
        self.source_system      = source_system
        self.source_component   = source_component
        self.target_system      = target_system
        self.target_component   = target_component
        self.descriptor         = descriptor 
        self.baudrate           = baudrate
        self.rtscts             = rtscts
        self.set_mav_version(mav10, mav20)
        self.link_add(descriptor)

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
        conn.linknum = len(self.mpstate.mav_master)
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
        if( mtype in mav_cam_cmds ):
            getattr(self, mav_cam_cmds[mtype])(m, master)

#        if mtype == 'HEARTBEAT' and m.type == mavutil.mavlink.MAV_TYPE_CAMERA:
#            if self.target_system == 0 and self.target_system != m.get_srcSystem():
#                self.target_system = m.get_srcSystem()
#                self.say("online system %u" % self.target_system,'message')
#                for mav in self.mpstate.mav_master:
#                    mav.target_system = self.target_system
#
#            if master.linkerror:
#                master.linkerror = False
#                self.say("link %s OK" % (self.link_label(master)))
            #self.status.last_heartbeat = time.time()
            #master.last_heartbeat = self.status.last_heartbeat

        #Already checked that the target system and target components match

    def master_callback(self, m, master):
        '''process mavlink message m on master, sending any messages to recipients'''

        # see if it is handled by a specialised sysid connection
        sysid = m.get_srcSystem()
        mtype = m.get_type()

        if getattr(m, '_timestamp', None) is None:
            master.post_message(m)

        self.master_msg_handling(m, master)

    def mav_cmd_request_camera_information(self, m, master):
    def mav_cmd_request_camera_settings(self, m, master):
    def mav_cmd_request_storage_information(self, m, master):
    def mav_cmd_storage_format(self, m, master):
    def mav_cmd_request_camera_capture_status(self, m, master):
    def mav_cmd_reset_camera_settings(self, m, master):
    def mav_cmd_set_camera_mode(self, m, master):
    def mav_cmd_set_camera_focus(self, m, master):
    def mav_cmd_image_start_capture(self, m, master):
    def mav_cmd_image_stop_capture(self, m, master):
    def mav_cmd_request_camera_image_capture(self, m, master):
    def mav_cmd_do_trigger_control(self, m, master):
    def mav_cmd_video_start_capture(self, m, master):
    def mav_cmd_video_stop_capture(self, m, master):
    def mav_cmd_video_start_streaming(self, m, master):
    def mav_cmd_video_stop_streaming(self, m, master):
    def mav_cmd_request_video_stream_information(self, m, master):
    def mav_cmd_request_video_stream_status(self, m, master):
    def mav_cmd_do_control_video(self, m, master):


if __name__ == "__main__":
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
        if mpstate.status.exit:
            print('Clean shutdown impossible, forcing an exit')
            sys.exit(0)
        else:
            mpstate.status.exit = True

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
