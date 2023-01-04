#!/usr/bin/env bash
set -o errexit -o nounset -o pipefail
IFS=$'\n\t\v'

# This file is used by help.mk to print out helpful messages about your makefile.
# See help.mk for more info.

# if target == "", we assume we need to print generic help instead of explaining a command
target=$1
shift
MAKEFILE_LIST=($@)

function resetvars {
  sectionname=""
  explaintext=""
  helptext=""
}
alreadyseen=":"

resetvars
for file in ${MAKEFILE_LIST[@]}; do
  while IFS="" read -r line || [[ -n "${line}" ]]; do
    # start of a new explain comment section, or continuation of a new one
    if [[ "${line:0:1}" == "#" ]] && ([[ "$explaintext" != "" ]] || [[ "${line:0:2}" == "##" ]]); then
      shopt -s extglob
      line="${line##+(#)*( )}"
      shopt -u extglob
      # might be part of an explain comment block
      if [[ -n "${explaintext}" ]]; then
        explaintext+="\n"
      fi
      [[ -z "${line}" ]] && continue
      explaintext+="${line}"
      # might be a section name, if the next line is not another comment or target
      sectionname="${line}"
    # potential rule
    elif rulematch="$(echo ${line} | grep -Eo '^[%/.a-zA-Z_-]+:')"; then
      rulematch=${rulematch::-1}
      # echo "potential rule ${rulematch}"
      if helptext=$(echo ${line} | grep -Eo '##.*'); then
        shopt -s extglob
        helptext="${helptext##+(#)*( )}"
        shopt -u extglob
      fi
      # `make explain` if it has helptext
      if [[ -n "${helptext}" ]] && [[ -z "${target}" ]]; then
        keycolor="\033[36m"
        valuecolor=""
        if echo "$alreadyseen" | grep ":${rulematch}:" > /dev/null; then
          keycolor="\033[90m"
          valuecolor="\033[90m"
        else
          alreadyseen="${alreadyseen}${rulematch}:"
        fi
        printf "${keycolor}%-30s\033[0m ${valuecolor}%s\033[0m\n" "$rulematch" "$helptext"
      fi
      # `make explain` if the target matches
      if [[ "${rulematch}" == "${target}" ]]; then
        echo ""
        printf "\033[36m%s\033[0m : %s\n" "$rulematch" "$helptext"
        echo ""
        if [[ -z "${explaintext}" ]]; then
          echo "no explanation text provided"
        else
          echo -e "${explaintext}"
        fi
      fi
      
      # don't reset vars if it's a .PHONY or friends (like .ONESHELL)
      # real rule likely follows in the next line
      ! suspect_phony="$(echo ${rulematch} | grep -E '^.[A-Z]+$')"
      # starts with dot, fully uppercase, no help text. probably a PHONY
      if [[ -n "${suspect_phony}" ]] && [[ -z "${helptext}" ]]; then
        continue
      fi
      
      # it is a target, but not the one we are looking for, start new fresh search
      resetvars
    else
      # echo "no match $line"
      if [[ -n "${sectionname}" ]] && [[ -z "${target}" ]]; then
        echo -e "\n\033[33m${sectionname}\033[0m\n"
      fi
      resetvars
    fi
  done < ${file}
done