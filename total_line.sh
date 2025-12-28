#!/bin/bash

WORK_DIR="$(cd "$(dirname "$0")" && pwd)"

echo -e "${yellow}\n<--- Total Lines --->${reset}"
    total=$(find "$WORK_DIR" \
        -type d \( \
            -path "$BUILD_DIR" -o \
            -path "$WORK_DIR/model" -o \
            -path "$WORK_DIR/3rdparty" -o \
            -path "$WORK_DIR/.cache" \
        \) -prune -o \
        -type f \( \
            -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" \
            -o -name "*.py" -o -name "*.html" -o -name "*.sh" -o -name "*.md" \
            -o -name "*.yaml" -o -name "*.json" -o -name "*.css" -o -name "*.js" \
            -o -name "*.cu" -o -name "*.txt" \
        \) -exec wc -l {} + | awk 'END{print $1}')
echo -e "${blue}        $total${reset}"