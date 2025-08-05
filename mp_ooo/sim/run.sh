#!/bin/bash
set -e
set -o pipefail

OUT_FILE="benchmark_results.txt"
SIM_LOG=$(mktemp)

trap "echo -e '\n\033[0;31mScript interrupted. Partial results saved in $OUT_FILE.\033[0m'; exit 1" SIGINT

# List of specific files to exclude
EXCLUDED_FILES=("coremark_imc.elf" "another_file_to_exclude.c")

# List of file extensions to exclude (e.g., "*.elf", "*.c", "*.s")
EXCLUDED_EXTENSIONS=("c")

> "$OUT_FILE"

in_excluded() {
    local item=$1
    for exclude in "${EXCLUDED_FILES[@]}"; do
        if [[ "$item" == "$exclude" ]]; then
            return 0
        fi
    done
    for ext in "${EXCLUDED_EXTENSIONS[@]}"; do
        if [[ "$item" == *.$ext ]]; then
            return 0
        fi
    done

    return 1
}

# Find and loop over all *.elf, *.c, and *.s files
find ../testcode -type f \( -name "*.elf" -o -name "*.c" -o -name "*.s" \) | while read -r file; do
    filename=$(basename "$file")

    if in_excluded "$filename"; then
        echo -e "\033[0;34mSkipping excluded file $filename\033[0m"
        continue
    fi

    echo -e "\033[0;33mTesting $file\033[0m"

    real_start_time=$(date +%s)

    make clean

    if ! make run_vcs_top_tb PROG="$file" | tee >(grep -E "Monitor|finish" > "$SIM_LOG"); then
        echo -e "\033[0;31mError running $file. Stopping.\033[0m"
        exit 1
    fi

    real_end_time=$(date +%s)

    SEGMENT_IPC=$(grep "IPC" "$SIM_LOG" | awk '{print $4}' | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
    SEGMENT_IPC=${SEGMENT_IPC:-"N/A"}

    elapsed_seconds=$((real_end_time - real_start_time))
    elapsed_minutes=$((elapsed_seconds / 60))
    elapsed_seconds_only=$((elapsed_seconds % 60))

    {
        echo "-----------------------------"
        echo "Benchmark: $filename"
        echo "Monitor: Segment IPC: $SEGMENT_IPC"
        echo "Time Elapsed: ${elapsed_minutes} minutes ${elapsed_seconds_only} seconds"
        echo "-----------------------------"
        echo
    } >> "$OUT_FILE"

    echo -e "\033[0;32mFinished $filename\033[0m"
done
