#!/usr/bin/env bash

set -e

srcs=$(git ls-files '*.c' '*.cc' '*.cpp' '*.cxx' '*.c++' '*.h' '*.hpp' '*.hh')
srcs=${srcs//$'\n'/ }

GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}clang-tidy -p build/compile_commands.json $srcs${NC}"
clang-tidy -p build/compile_commands.json $srcs
