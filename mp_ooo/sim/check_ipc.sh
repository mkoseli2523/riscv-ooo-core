#/bin/bash

set -e

ipc=$(bash get_ipc.sh)
st=$(echo "$ipc < $1" | bc)

cd vcs

if [ ! -f simulation.log ] || [ $st -eq 1 ] ; then
    echo -e "\033[0;31mIPC Not Met \033[0m"
    exit 1
else
    echo -e "\033[0;32mIPC Met \033[0m"
    exit 0
fi
