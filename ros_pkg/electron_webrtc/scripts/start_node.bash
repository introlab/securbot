#! /bin/bash

export ELECTRON_ENABLE_LOGGING=1

/usr/bin/xvfb-run -a -s '-screen 0 800x600x16' $(dirname "$0")/../node_modules/.bin/electron $(dirname "$0")/.. 1>&2
