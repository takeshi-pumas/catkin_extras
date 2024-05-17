#!/bin/bash
# 2. Start JACK server
echo "Starting JACK server..."
jack_control start
jack_control ds alsa
jack_control dps device hw:0
jack_control dps rate 44100
jack_control dps nperiods 2
jack_control dps period 1024

# 3. Run gstreamer client
echo "Running gstreamer client..."
gst-launch-1.0 -v udpsrc port=5000 ! "application/x-rtp, media=(string)audio, clock-rate=(int)48000, encoding-name=(string)OPUS, payload=(int)96" ! rtpopusdepay ! opusdec ! audioconvert ! audioresample ! jackaudiosink connect=1

# Wait for the client to start
sleep 5

# 4. Connect JACK clients
echo "Connecting JACK clients..."
jack_connect "gst-launch-1.0:out_0" "system:playback_1"
jack_connect "gst-launch-1.0:out_1" "system:playback_2"
jack_connect "PulseAudio JACK Source:front-left" "system:playback_1"
jack_connect "PulseAudio JACK Source:front-right" "system:playback_2"
