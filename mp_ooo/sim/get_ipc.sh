#/bin/bash

set -e

cd vcs

grep -qE 'Monitor: (Total|Segment) IPC: +?([0-9]+?\.[0-9]+?)$$' simulation.log
sed -nr 's/Monitor: (Total|Segment) IPC: +?([0-9]+?\.[0-9]+?)$$/\2/p' simulation.log
