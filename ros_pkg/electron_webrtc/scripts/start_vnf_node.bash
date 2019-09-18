#! /bin/bash

/usr/bin/xvfb-run -a -s '-screen 0 800x600x16' $(dirname "$0")/../node_modules/.bin/electron --enable-logging --disable-gpu  $(dirname "$0")/.. 1>&2
