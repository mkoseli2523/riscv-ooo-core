#/bin/bash

set -e

cd vcs

grep -qE 'Monitor: (Total|Segment) Time: +?([0-9]+?)$$' simulation.log
sed -nr 's/Monitor: (Total|Segment) Time: +?([0-9]+?)$$/\2/p' simulation.log
