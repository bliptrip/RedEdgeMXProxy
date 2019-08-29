#!/usr/bin/env python3
#
#Objective: Test streaming MJPEG video over RTP using ffmpeg framework.

import io
import json
from PIL import Image
import requests
import subprocess as sp
import time


def http_post(ip, route, payload):
    rd = None
    try:
        #Insert a timeout, or can hang indefinitely
        r = requests.post("http://{0}/{1}".format(ip,route), data=payload, timeout=5.0)
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

cmd_out = ['/Library/Frameworks/GStreamer.framework/Commands/gst-launch-1.0', '-v',
           'fdsrc',
           '!', 'jpegdec', #Need to decode jpeg
           '!', 'jpegenc', #and then re-encode for it to work?  Not sure why.
           '!', 'progressreport',
           '!', 'rtpjpegpay',
           '!', 'udpsink', 'host=127.0.0.1', 'port=5000']
pipe = sp.Popen(cmd_out, stdin=sp.PIPE)
ip = "192.168.1.83"
count = 120
while count > 0:
    rd = http_post(ip, 'config', {'streaming_enable': True, 'streaming_allowed': True, 'preview_band': 'multi'})
    time.sleep(5) #Give it some time for the streaming to start
    rd = http_get(ip, "jpeg_url")
    if rd is not None:
        jpeg_url = rd['jpeg_url']
        r = requests.get('http://' + ip + jpeg_url)
        if(r.status_code == requests.codes.ok):
            stream = io.BytesIO(r.content)
            img = Image.open(stream)
            imgc = img.convert('RGB') #Need to convert to RGB otherwise rtpjpegpay fails. 
            imgc.save(pipe.stdin, 'JPEG')
    count -= 1
    time.sleep(0.5)
pipe.close()
