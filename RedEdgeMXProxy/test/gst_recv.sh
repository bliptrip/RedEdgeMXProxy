#!/bin/sh
#Objective: Test receiving a MJPEG-type stream over RTP-UDP stream for receiving 'video' feed from MicaSense RedEdge Camera
#
#Shamelessly patched together from examples at https://github.com/mavlink/qgroundcontrol/blob/master/src/VideoStreaming/README.md
# and https://gstreamer.freedesktop.org/data/doc/gstreamer/head/gst-plugins-good/html/gst-plugins-good-plugins-jpegdec.html.
#
# RTP streaming example (with H264 encoding instead of MJPEG): 
#     gst-launch-1.0 udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! autovideosink fps-update-interval=1000 sync=false
# MJPEG decoding example (with file as source instead of RTP stream)): 
#     gst-launch-1.0 -v filesrc location=mjpeg.avi ! avidemux !  queue ! jpegdec ! videoconvert ! videoscale ! autovideosink

UDP_PORT=5000

if [ $# -ge 1 ]; then
    UDP_PORT=$1
fi

cmd="gst-launch-1.0 udpsrc port=${UDP_PORT} ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink"
echo $cmd
eval $cmd
