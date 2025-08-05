#/bin/bash

set -e

cd vcs

if [ ! -f compile.log ] || grep -q 'Error-' compile.log; then
    echo -e "\033[0;31mCompile failed \033[0m"
    exit 1
fi

cat xprop.log

if [ ! -f xprop.log ] || grep -q ' NO' xprop.log; then
    echo -e "\033[0;31mXProp failed \033[0m"
    exit 1
fi

if grep -q 'Warning-\|Lint-' compile.log; then
    echo -e "\033[0;33mCompile finished with warnings \033[0m"
    exit 69
fi

echo -e "\033[0;32mCompile Successful \033[0m"
exit 0
