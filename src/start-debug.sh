#!/bin/sh
DIR=`dirname $0`
$DIR/start.sh --offline-logging --debug --debug-server-connect ${1+"$@"}
