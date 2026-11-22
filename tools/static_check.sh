#!/bin/sh
# Static analysis of the flight code (host configuration): cppcheck must report nothing new.
# Reviewed false positives: tools/cppcheck.supp. Optional deeper pass: clang-tidy (bugprone / analyzer),
# printed for review, not a gate:  tools/static_check.sh tidy
set -e
cd "$(dirname "$0")/.."
cppcheck --enable=warning,performance,portability --inconclusive --std=c++17 -DALLQUAD_HOST --suppress=missingIncludeSystem \
  --suppressions-list=tools/cppcheck.supp --error-exitcode=1 -q --template='{file}:{line}: {severity}/{id}: {message}' \
  -i src/bridge src/
echo "cppcheck: clean"
if [ "$1" = "tidy" ]; then
  find src -name '*.cpp' ! -path 'src/bridge/*' ! -path 'src/hal/hal_*' ! -path 'src/hal/stm32/*' ! -path 'src/os/os_freertos.cpp' \
       ! -path 'src/main.cpp' ! -path 'src/web/web_esp32.cpp' | xargs -P 8 -I{} clang-tidy {} -quiet \
    --checks='-*,bugprone-*,clang-analyzer-*,-bugprone-easily-swappable-parameters,-bugprone-narrowing-conversions,-bugprone-implicit-widening-of-multiplication-result,-bugprone-reserved-identifier,-bugprone-branch-clone,-bugprone-integer-division,-clang-analyzer-optin.performance.Padding,-bugprone-switch-missing-default-case,-bugprone-assignment-in-if-condition,-clang-analyzer-security.insecureAPI.*,-bugprone-macro-parentheses,-bugprone-signed-char-misuse,-bugprone-incorrect-roundings,-bugprone-suspicious-string-compare' \
    -- -std=c++17 -DALLQUAD_HOST -Isrc 2>/dev/null | grep "warning:" || echo "clang-tidy: nothing"
fi
