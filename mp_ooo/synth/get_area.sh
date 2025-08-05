#/bin/bash

set -e

grep -qE 'Total cell area: +?([0-9]+?)' reports/area.rpt
sed -nr 's/Total cell area: +?([0-9]+?)\.[0-9]+?$$/\1/p' reports/area.rpt
