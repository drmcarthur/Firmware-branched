## Pixhawk Firmware Mods for Vicon Position Control ##

### ADD FILES ###
* src/modules/mc_vicon_pos_control/mc_vicon_pos_control.cpp
	* Used …modules/mc_pos_control as a starting point
* src/modules/mc_vicon_pos_control/modules.mk
	* Used …modules/mc_pos_control/modules.mk as starting point
* src/modules/vicon_receiver/vicon_receiver.c
	* Custom app to receive vicon data via XBee on Serial 5
	* In the future, will use mavlink. Sample code for starting mavlink on SERIAL4 (see [mavlink](https://pixhawk.org/firmware/apps/mavlink) for more details):
    	* <code>mavlink stream -d /dev/ttyS6 -s CUSTOM_STREAM_NAME -r 50</code>
* src/modules/vicon_receiver/modules.mk
	* see ‘modules.mk’ above...

### MODIFY FILES ###
* makefiles/config_px4fmu-v2_default.mk
	* Add mc_vicon_pos_control under Vehicle Control
* ROMFS/px4fmu_common/init.d/rc.mc_apps
	* Change 'mc_pos_control start' to 'mc_vicon_pos_control start'

### BUILD INSTRUCTIONS ###
* First time (use -j6 to run builder with 6 threads):
	* <pre>
	  <code>cd /path/to/Firmware
	  make distclean
	  make archives
	  make
	  make -j6 upload px4fmu-v2_default
	  </code></pre>
* Subsequent builds:
	* <code>make -j6 upload px4fmu-v2_default</code>

### GIT COMMANDS ###
 * Clone
	* <code>git clone https://github.com/drmcarthur/vision-control.git</code>
 * Commit
 	* If you <i>only</i> modified existing files, you can use the "-a" flag to stage all changes automatically (this detects modified and/or removed files): 
 		* <code>git commit -a -m "Short description of this commit"</code>
 	* If you add one or more files, you should stage the file to be commited before calling <code>git commit</code>:
 		* <code>git add "new filename"</code>
 * Push
 	* To push the changes from your local copy (on your computer) to the master online:
 		* <code>git push origin master</code>	
 * Create a New Branch
 	* To create a new branch (or switch to an existing branch):
 		* <code>git checkout -b [name_of_your_new_branch]</code>
	* Add a new remote for your branch:
		* <code>git remote add [name_of_your_remote]</code>
	* Commit changes as usual
	* Push changes to online repository:
		* <code>git push origin [name_of_your_branch]</code>
	* Get updates from online repository:
		* <code>git fetch [name_of_your_branch]</code>
