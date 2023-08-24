qjackctl &

gst-launch-1.0 -v udpsrc uri=udp://169.254.2.173:6000 caps="application/x-rtp,channels=(int)2,format=(string)S16LE,media=(string)audio,payload=(int)96,clock-rate=(int)44100,encoding-name=(string)L24" ! queue ! rtpL24depay ! audioconvert ! autoaudiosink

