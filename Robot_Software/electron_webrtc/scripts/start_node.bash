#! /bin/bash

export ELECTRON_ENABLE_LOGGING=1

$(dirname "$0")/../node_modules/.bin/electron $(dirname "$0")/..
