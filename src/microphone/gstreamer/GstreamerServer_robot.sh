# This shell runs on robot
# Host IP is pc client IP
gst-launch-1.0 -v pulsesrc ! audioconvert ! audioresample ! opusenc ! rtpopuspay ! udpsink host=169.254.2.172 port=5000
