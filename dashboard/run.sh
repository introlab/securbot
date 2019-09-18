#!/bin/bash

# The output file to which to bind the output volume
VOL_MNT=/var/www/html/dashboard

# Check if the settings file is present
if [ ! -f ./settings.json ]
then
    echo "Missing settings.json file"
    exit
fi

# Start by building docker image
echo "Building image"
docker build -t securbot/dashboard .

# If no argument is provided, scheduledRender is started to run as daemon
if [ $# -lt 1 ]
then
    echo "Starting image as deamon"
    docker run -v $VOL_MNT:/usr/src/app/out --restart unless-stopped -d securbot/dashboard

# If arguments are provided to execute specific scripts
elif [ $# -lt 2 ]
then
    echo "Executing specified script"
    docker run -v $VOL_MNT:/usr/src/app/out securbot/dashboard node $1

# If too many arguments are provided, execute nothing
else
    echo "Too many arguments, please only specify node scripts to execute"
fi
