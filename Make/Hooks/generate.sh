#!/bin/bash
wdPath=$(pwd)

execute() {
  if [ -x "$1" ]
  then
    $1 >/dev/null
  fi
}

if [ "$('GIT' config hooks.generateProject)" = "true" ]
then
  case "$OSTYPE" in
    "cygwin"|"msys")
      if [ -f "${wdPath}/Make/VS2019/generate.cmd" ]; then
        pushd "${wdPath}/Make/VS2019" >/dev/null
        ./generate
        popd >/dev/null
      fi
      ;;
    linux*)
      if [ -e /proc/version -a ! -z "`grep Microsoft </proc/version`" ]; then
        if [ -f "${wdPath}/Make/VS2019/generate.cmd" ]; then
          pushd "${wdPath}/Make/VS2019" >/dev/null
          ./generate
          popd >/dev/null
        fi
      else
        execute "${wdPath}/Make/LinuxCodeLite/generate"
      fi
      ;;
    darwin*)
      execute "${wdPath}/Make/macOS/generate"
      ;;
    *)
      echo "Warning: Unknown platform. Project files not generated." >&2
      ;;
  esac
fi
