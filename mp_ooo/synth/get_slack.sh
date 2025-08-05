#/bin/bash

set -e

grep -qE ' +slack \([A-Z]+\) +(-?[0-9]+?\.[0-9]+?)$$' reports/timing.rpt
sed -nr 's/ +slack \([A-Z]+\) +(-?[0-9]+?\.[0-9]+?)$$/\1/p' reports/timing.rpt
