from setuptools import setup
import os, platform
import sys

assert sys.version_info >= (3,4)

version = "1.0.0"

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

package_data = ['RedEdgeMXProxy/*.xml']

# note that we do not include all the real dependencies here (like matplotlib etc)
# as that breaks the pip install. It seems that pip is not smart enough to
# use the system versions of these dependencies, so it tries to download and install
# large numbers of modules like numpy etc which may be already installed
with open('RedEdgeMXProxy/requirements.txt', 'r') as req_fh:
    requirements=[r.rstrip('\n') for r in req_fh.read_lines()]
    req_fh.close()

if sys.version_info < (3,6):
    requirements.append('backports-datetime-fromisoformat')

setup(name='RedEdgeMXProxy',
      version=version,
      zip_safe=True,
      description='MicaSense RedEdge MX Camera Proxy',
      long_description='''Motivated by Andrew Tridgell's MAVProxy (http://ardupilot.github.io/MAVProxy/), a MicaSense RedEdge MX camera 
      MavLink proxy for use in MavLink-supported unmanned aerial system (UAS) autopilots.  The RedEdgeMX proxy translates MavLink camera
      commands and MavLink extended parameter settings to/from the MicaSense HTTPApi (http://micasense.github.io/rededge-api/api/http.html).
      By default the proxy forwards MavLink-protocol packets to/from a serial UART port on an autopilot (typically connected through
      an available telemetry/UART port on the autopilot FMU), and sends corresponding camera configuration requests over the MicaSense
      HTTPApi (over wifi or ethernet).  More information can be found at https://github.com/bliptrip/RedEdgeMX.'''
      url='https://github.com/bliptrip/RedEdgeMX',
      author='Andrew Maule',
      author_email='developer@bliptrip.net',
      classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 3.4+',
        'Topic :: Scientific/Engineering'],
      license='GPLv3',
      packages=['RedEdgeMXProxy'],
      install_requires=requirements,
      scripts=['RedEdgeMXProxy/micamavproxy.py'],
      package_data={'RedEdgeMXProxy': package_data}
    )
