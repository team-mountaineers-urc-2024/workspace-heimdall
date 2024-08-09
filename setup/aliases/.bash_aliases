#!/bin/bash

h ()
{
  echo "start <camera_name> <lowest|low|mid|high|insane|incomprehensible>"
  echo "    Starts a Camera with the given preset"
  echo ""
  echo "decypher <camera_name>"
  echo "    Uncompresses a Camera using Theora"
  echo ""
  echo "recall <camera_name>"
  echo "    Resends the Theora Header for a camera"
  echo ""
  echo "replug <port>"
  echo "    Unbinds and Binds a USB port simulating un and replugging it"
  echo ""
  echo "flie <camera_name>"
  echo "    Republishes a camera with the image rotated 180 degrees"
  echo ""
  echo "contrast <camera_name> <value>"
  echo "    Changes the output contrast of a given camera"
  echo ""
  echo "brightness <camera_name> <value>"
  echo "    Changes the output brightness of a given camera"
  echo ""
  echo "pico <value>"
  echo "    Launches the Docker for a pico on port ttyACM<value>"
  echo ""
  echo "signal_strength"
  echo "    Prints out the radio signal strength every 3 seconds"
  echo ""
  echo "launch <category>"
  echo "    Launches a setup component. Run with no arguments to list options"
  echo ""

}

start ()
{
  if [[ "$2" == "lowest" || "$2" == "low" || "$2" == "mid" || "$2" == "high" || "$2" == "pano" || "$2" == "insane" || "$2" == "incomprehensible" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/components/dynamic_cam.launch.py camera_name:="$1" param_file:="$2.yaml"
  else
    echo "$2 is not valid"
    echo "Please use lowest | low | mid | high | insane | pano | incomprehensible"
  fi
}

recall ()
{
  ros2 service call "$1_mux/recall_header" std_srvs/srv/Trigger
}

decypher ()
{
  cd ~/image-transport-ws && source install/setup.bash && ros2 run image_transport republish --ros-args -p in_transport:='theora' -p out_transport:='raw' -r "in/theora":="/$1/image_raw/theora_mux" -r "out":="/$1/image_uncompressed" -r __node:="$1_decypherer"
}

replug ()
{
  echo "$1" | sudo tee /sys/bus/usb/drivers/usb/unbind
  sleep 3
  echo "$1" | sudo tee /sys/bus/usb/drivers/usb/bind
}

flip ()
{
  cd ~/workspace-heimdall && source install/setup.bash && ros2 run image_mod_pkg image_flip --ros-args -r "/input":="/$1/image_uncompressed" -r "/output:=/$1/image_flipped"
}

contrast ()
{
  ros2 param set "$1" contrast "$2"
}

brightness ()
{
  ros2 param set "$1" brightness "$2"
}

pico ()
{
  sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM"$1" -b 115200
}

signal_strength()
{
  cd ~/workspace-heimdall && ./scripts/comms/print_signal_strength.py
}

launch ()
{
  if [[ "$1" == "es" || "$1" == "er" || "$1" == "dm" || "$1" == "d" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/heimdall/heimdall_es_comms.launch.py
  elif [[ "$1" == "science" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/heimdall/heimdall_science.launch.py
  elif [[ "$1" == "gui" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch heimdall_gui gui_bringup.launch.py
  elif [[ "$1" == "es_tethered" || "$1" == "er_tethered" || "$1" == "dm_tethered" || "$1" == "d_tethered" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/heimdall/heimdall_es_tethered.launch.py
  elif [[ "$1" == "teleop" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/heimdall/heimdall_teleop.launch.py
  elif [[ "$1" == "es_base" || "$1" == "er_base" || "$1" == "dm_base" || "$1" == "d_base" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/base_station/base_station_es.launch.py
  elif [[ "$1" == "autonomy" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/heimdall/heimdall_full_autonomy.launch.py
  elif [[ "$1" == "science_base" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/base_station/base_station_science.launch.py
  elif [[ "$1" == "autonomy_base" ]]; then
    cd ~/workspace-heimdall && source install/setup.bash && ros2 launch launches/base_station/base_station_autonomy.launch.py
  else
    echo "The Following Arguments are Valid:"
    echo "es"
    echo "dm | d | er"
    echo ""
    echo "es_tethered"
    echo "dm_tethered | d_tethered | er_tethered"
    echo ""
    echo "es_base"
    echo "dm_base | d_base | er_base"
    echo ""
    echo "science"
    echo "science_base"
    echo ""
    echo "autonomy"
    echo "autonomy_base"
    echo ""
    echo "gui"
    echo "teleop"
  fi
}