# RedEdgeMXProxy
Micasense RedEdgeMX Multispectral camera proxy that runs on an Pixhawk offboard computer.  

---

Motivated by Andrew Tridgell's [MAVProxy](http://ardupilot.github.io/MAVProxy/), a MicaSense
[RedEdgeMX](https://www.micasense.com/rededge-mx) camera MavLink proxy for use in 
[MavLink-supported](https://mavlink.io) unmanned aerial system (UAS) autopilots.  The RedEdgeMX proxy translates MavLink camera
commands and MavLink extended parameter settings to/from the [MicaSense HTTPApi](http://micasense.github.io/rededge-api/api/http.html).
By default the proxy forwards MavLink-protocol packets to/from a serial UART port on an autopilot (typically connected through
an available telemetry/UART port on the autopilot FMU), and sends corresponding camera configuration requests over the MicaSense
HTTPApi (over wifi or ethernet).

---

# Installation
## Required Packages
`RedEdgeMXProxy` depends on `gstreamer`, as it will stream short time-lapse video from the camera to a ground control
station like [QGroundControl](http://qgroundcontrol.com/).  To use with `QGroundControl`, install a version of
`gstreamer` that is well-supported as defined in the 
[README](https://github.com/mavlink/qgroundcontrol/blob/master/src/VideoStreaming/README.md)

## Setup of `RedEdgeMXProxy` Package and Service
Installation on a Linux OS can be done by executing `./install.sh` on the commandline.
