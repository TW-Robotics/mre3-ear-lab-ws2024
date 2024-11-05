# Programms

The following page gives an overview of what these scripts are capable of and how to use them.   
The installTaurobROS.sh script creates a symbolic link of this folder to $HOME/scripts and adds these files to $PATH so they can be executed from anywhere.


## Embedded PC
The following scripts are available on the embedded pc that is mounted on the Taurob Tracker mobile robot.

### del_csv.sh
For Competitions, this script should be executed before starting a run. It deletes all relevant (clicked_humans; gcounts_neu) .csv files and creates empty new ones.

```
$ del_csv.sh
```

### draw_map_symbols
Herewith 2D maps (see following figure) with recognized objects are created. For this the files sign.csv and human.csv from the package detection will be used.

```
$ draw_map_radiological <path_to_map.pgm> <path_to_map.yaml> <path_to_human.csv> <path_to_sign.csv> <resolution | usually 0.10> <draw_legend>
```
![Example of automatically detected objects marked in 2D map](../../../PICs/2DMap_Symbols_example.png)

### draw_map_radiological
Herewith a 2D map of the environment overlayed with radioactive heatmap is created. Currently (Nov 2019) the distribution of the radioactivity is assumed to be gaussian.
```
$ draw_map_radiological <path_to_map.pgm> <path_to_map.yaml> <path_to_gauss_out.csv> <resolution | usually 0.10> <draw_legend>
```
![Example of RN Mapping](../../../PICs/RNMap_example.png)

### save_all_maps.sh
Is a wrapper for draw_map_symbols/radiological.  
Saves the current 2D map via the ROS map_server as well as the current 3D Octomap via the octomap_server. Furthermore draw_map_symbols as well as draw_map_radiological are started and saved all created maps under ∼/maps/%d_%m_%Y/.

```
$ save_all_maps.sh <-m ... Generates a 2D map (.png & .yaml) and the octomap file (.ot) and a 2D map with detected Signs and Humans marked in it> <-r Generates the Radiation Heatmap>
```


### automatic_map_csv_saver.py

The "automatic_map_csv_saver.py script is used to periodically display the map
Files (from Octomap and Cartographer), as .tar.gz, to be saved in case of a connection
to have backed up the data in the event of a loss.

```
$ automatic_map_csv_saver.py -l (looptime in sec) -s (scptransfer) <0|1> 
```
The "-l" parameter is mandatory, while the -s parameter is set to 0 by default. If the scptransfer parameter is activated, the .tar.gz will be transfered automatically to the operator with IP 10.0.0.201.


### xbox_teleop.sh
Is automatically executed when the Xbox360 controller, that is defined in [10-local.rules](../../../config_data/udev/10-local.rules) , is attached to the embedded pc.
The xbox_teleop.sh script starts all necessary packages to remote control the tracker with the controller. For security reasons the operator laptop must still be connected to the embedded via Wifi and start the "taurob_watchdog_client" to provide a software emergency stop. 

```
$ rosrun tarob_watchdog_client taurob_watchdog_client <ip_embedded>
```
Start the taurob_watchdog_client on the operator laptop for teleoperation of the taurob. If connected via OpenVPN the embedded's IP is always 10.0.0.100. When starting the watchdog_client locally on the embedded the IP is 127.0.0.1.


### save_cartographer.sh
Saves the *.pbstream that the cartographer uses to  ∼/maps/%d_%m_%Y/.

```
$ save_cartographer.sh
```

### save_rosbag.bash
Used to cleanly save rosbags by killing them.
```
$ save_rosbag.bash
```

### change_iface.sh
Resets the symbolic link of the interfaces file, which is changed to enable bridged vpn on the embedded pc for the operator, so that code can be pushed to gitlab of fhtw.
```
$ change_iface.sh
```


## Operator

### taurob_tmux.bash
The "taurob_tmux" script is used to run all required ROS packages to map and steer the mobile robot.  Here a tmux session is created and the taurob is accessed via ssh. The skritp can be executed on the operator laptop via a terminal or by double-clicking on the file.
```
$ taurob_tmux.bash
```
Since the configuration of the script often changes, a list of the started ROS packages is omitted here. To get this list please open the script and see what is started.