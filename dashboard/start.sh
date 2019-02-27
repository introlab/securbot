#!/bin/bash

sudo docker run -v /var/www/html/dashboard:/usr/src/app/out --restart unless-stopped -d securbot/dashboard
