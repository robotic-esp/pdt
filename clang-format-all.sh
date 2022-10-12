#!/bin/bash

for file in $(git ls-tree -r --name-only --full-name master); do
    if [ -f "${file}" ]; then
        if [ "${file#*.}" = "cpp" ] || [ "${file##*.}" = "h" ] || [ "${file#*.}" = "cpp.in" ] || [ "${file#*.}" = "h.in" ]; then
            clang-format -i ${file}
        fi
    fi
done
