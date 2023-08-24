gst-launch-1.0 -v autoaudiosrc ! audioconvert ! audioresample ! audio/x-raw, rate= 44100, channels=2 ! audioconvert ! rtpL24pay ! udpsink host=192.168.195.88 auto-multicast=true port=6000
