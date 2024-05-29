In order to use HSRB PS-EYE (microphone) on ROS laptop you MUST do the following:

1. Run gstreamer server:
    ssh administrator@hsrb.local_et
    #sudo nano GstreamerServer_robot.sh (change host ip to your ROS laptop ip) (No needed if you're using takeshi or tamagawa laptop)
    ./GstreamerServer_robot.sh

2. Run gstreamer client:
    qjackctl
    On gui:
        click on Start button, then click on play button
    cd ~/catkin_extras/src/microphone/gstreamer
    ./GstreamerClient_pc.sh
    On gui:
        click on Connect button, then connect Readable client "gst-launch-1.0" (both) with  Writable client "PulseAudio Jack Source" (both)

3. Check on your ROS laptop if the connection is successful:
    Open settings, then sound settings make sure "Jack Source (PulseAudio JACK source) is set as input

    