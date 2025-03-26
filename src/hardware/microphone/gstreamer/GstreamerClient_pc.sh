# This shell runs on PC 
gst-launch-1.0 -v udpsrc port=5000 ! "application/x-rtp, media=(string)audio, clock-rate=(int)48000, encoding-name=(string)OPUS, payload=(int)96" ! rtpopusdepay ! opusdec ! audioconvert ! audioresample ! jackaudiosink connect=1
