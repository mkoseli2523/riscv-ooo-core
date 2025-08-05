#/bin/bash

set -e

grep -qE 'Number of cells: +?([0-9]+?)$$' reports/area.rpt
sed -nr 's/Number of cells: +?([0-9]+?)$$/\1/p' reports/area.rpt
