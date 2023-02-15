#!/bin/bash
# Format all files in a git repo and output the resulting diff.
# Should be run with a clean working directory.
# If a git ref is provided as first argument, only format the changes between the ref and HEAD.
# The clang-format binary can be set with git setting clangFormat.binary or environment variable CF_BINARY.
# return values:
#   0  formatting correct
#   2  incorrect formatting (=non-empty diff)
#   3  wrong version of clang-format

MIN_VERSION=13

# Set clang-format binary
CF_BINARY_GIT=$(git config clangFormat.binary)
[ -z "${CF_BINARY}" ] && CF_BINARY=${CF_BINARY_GIT:-'clang-format'}

# Check version
VERSION=$("${CF_BINARY}" --version | sed -e 's/[^0-9]*//')
MAJ_VERSION=$(echo ${VERSION} | sed -e 's/\([0-9]*\)\.[0-9]*\.[0-9]*.*/\1/')
if [ "${MAJ_VERSION}" -lt "${MIN_VERSION}" ]; then
  echo >&2 "requirement clang-format>=${MIN_VERSION} not met (provided ${VERSION})"
  exit 3
fi

# Set patterns to exclude from formatting
[ -z ${CF_EXCLUDE} ] && CF_EXCLUDE=(
  ':!/Util/GameController/**'
  ':!/Util/libqxt/**'
  ':!/Util/qtpropertybrowser/**'
  ':!/Util/visualmesh/**'
)

if [ -n "$1" ]; then
  echo "using ref for checking changes: origin/$1"
  F=$(git diff --name-only "origin/$1" -- "*.cpp" "*.h" ${CF_EXCLUDE[*]})
else
  echo "no ref provided: checking all files"
  F=$(git ls-files -- "*.cpp" "*.h" ${CF_EXCLUDE[*]})
fi

for f in $F; do
  msg="$("$CF_BINARY" --style=file -i "$f" 2>&1)"
  [[ -z "$msg" ]] || echo "$f: $msg"
done

git diff --exit-code -- . ${CF_EXCLUDE[*]} || exit 2

echo "ok"
