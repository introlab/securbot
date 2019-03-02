#!/bin/bash
docker run -v /var/www/html/dashboard:/usr/src/app/out securbot/dashboard node renderAndMail
