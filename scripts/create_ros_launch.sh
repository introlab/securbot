#!/bin/bash

#Creates a .launch file (XML format) executable by roslaunch for a specific
#robot and its specific specs


#Standard input path for templates launch files to use
templ_path=../Robot_Software/securbot_pkg/launch/template_launch_files

#Standard output paths for new launch files
includes_out_path=../Robot_Software/securbot_pkg/launch/includes

base_out_path=bases
frame_out_path=frames
sensor_out_path=sensors

echo Creating launch file...

for part in 0 1 2
do
	echo Creating sub-launch file part $((part+1))/3 of principal launch file...

	if [ $part == 0 ]
	then
	    echo Creating sub_launch file for base from template...
	    echo What would precede _base.launch file name ?
	    echo prefix : 
	    read base_prefix
	    cp $templ_path/template_base.launch $includes_out_path/$base_out_path/"$base_prefix"_base.launch
	    echo TODO : automate base launch configuration...
    	elif [ $part == 1 ]
    	then
	    echo Creating sub_launch file for frame from template...
	    echo What would precede _frame.launch file name ?
	    echo prefix : 
	    read frame_prefix
	    cp $templ_path/template_frame.launch $includes_out_path/$frame_out_path/"$frame_prefix"_frame.launch
	    echo TODO : automate frame launch configuration...
    	else
	    echo Creating sub_launch file for sensor from template...
	    echo What would precede _sensor.launch file name ?
	    echo prefix : 
	    read sensor_prefix
	    cp $templ_path/template_sensor.launch $includes_out_path/$sensor_out_path/"$sensor_prefix"_sensor.launch
	    echo TODO : automate sensor launch configuration...
    	fi
done
