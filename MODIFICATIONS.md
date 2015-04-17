## Pixhawk Firmware Mods for Vicon Position Control ##

### ADDED FILES ###
* src/modules/vicon_receiver/vicon_receiver.c
	* Custom app to receive vicon data via XBee on Serial 5
	* In the future, will use mavlink. Sample code for starting mavlink on SERIAL4 (see [mavlink](https://pixhawk.org/firmware/apps/mavlink) for more details):
    	* <code>mavlink stream -d /dev/ttyS6 -s CUSTOM_STREAM_NAME -r 50</code>
* src/modules/vicon_receiver/modules.mk
	* Makefile to build vicon_receiver code
	* Based off of ‘modules.mk’ files in the other apps in src/modules

### MODIFIED FILES ###

### FIRST-TIME GIT SETUP ###
* See [complete details](https://pixhawk.org/dev/nuttx/building_and_flashing_console) on the Pixhawk website
	* <pre><code>cd /path/to/Firmware
	  git clone https://github.com/drmcarthur/Firmware-branched.git
	  git submodule init
	  git submodule update</code></pre>

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

### GIT COMMAND REFERENCE ###
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
	* Commit changes as usual
	* Push changes to online repository:
		* <code>git push origin [name_of_your_branch]</code>
	* Get updates from online repository:
		* <code>git fetch [name_of_your_branch]</code>
