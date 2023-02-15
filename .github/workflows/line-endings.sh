#!/bin/bash
# Set the line ending of all text files in a git repo to LF and output the resulting diff.
# Should be run with a clean working directory.
# If a git ref is provided as first argument, only touch the changed files between the ref and HEAD.
# return values:
#   0  line endings were correct
#   2  incorrect line endings (=non-empty diff)

# Set patterns to exclude from formatting
[ -z ${LE_EXCLUDE} ] && LE_EXCLUDE=(
  ':!/Util/GameController/**'
  ':!/Util/libqxt/**'
  ':!/Util/qtpropertybrowser/**'
  ':!/Util/visualmesh/**'
)

if [ -n "$1" ]; then
  echo "using ref for checking changes: origin/$1"
  F=$(git diff --name-only "origin/$1" -- ${LE_EXCLUDE[*]})
else
  echo "no ref provided: checking all files"
  F=$(git ls-files -- ${LE_EXCLUDE[*]})
fi

for f in $F; do
  if file -bi "$f" | grep text >/dev/null; then
    dos2unix -q "$f"
  fi
done

git diff --exit-code -- . ${LE_EXCLUDE[*]} || exit 2

echo "ok"
