#/bin/bash

set -e

if [ ! -f reports/lint.log ]; then
    exit 1
fi

if [ $(sed -nr 's/ +?Reported Messages: +?([0-9]+?) Fatals, +?([0-9]+?) Errors, +?([0-9]+?) Warnings, +?([0-9]+?) Infos$$/\1/p' reports/lint.log) -gt "0" ]; then
    echo -e "\033[0;31mLint with Fatal \033[0m"
    exit 1
fi

if [ $(sed -nr 's/ +?Reported Messages: +?([0-9]+?) Fatals, +?([0-9]+?) Errors, +?([0-9]+?) Warnings, +?([0-9]+?) Infos$$/\2/p' reports/lint.log) -gt "0" ]; then
    echo -e "\033[0;31mLint with Error \033[0m"
    exit 1
fi

if [ $(sed -nr 's/ +?Reported Messages: +?([0-9]+?) Fatals, +?([0-9]+?) Errors, +?([0-9]+?) Warnings, +?([0-9]+?) Infos$$/\3/p' reports/lint.log) -gt "0" ]; then
    echo -e "\033[0;33mLint with Warning \033[0m"
    exit 69
fi

echo -e "\033[0;32mLint Passed \033[0m"
exit 0
