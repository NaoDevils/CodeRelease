#!/bin/bash
#title           :wifiState.sh
#description     :Reads data from the wpa_supplicant
#author		     :Dominik Brämer
#==============================================================================

function wpa_supplicant_inactive() {
    sudo wpa_cli status > /dev/null 2>&1
    if [[ $? != 0 ]]; then
        return 0
    fi
    
    return 1
}

function status() { 
    if wpa_supplicant_inactive; then
        echo "DISABLED"
    else
        sudo wpa_cli status | grep "wpa_state" | cut -d "=" -f 2
    fi
}

function name() {
    if wpa_supplicant_inactive; then
        echo "NONE"
    else
        # This part is used to check if the correct config has been loaded
        if [ $(sudo wpa_cli status | grep "wpa_state" | cut -d "=" -f 2 | sed 's/ //g') == "SCANNING" ]; then
            # Read the current config (Nao V6) and
            # read the chosen config (Nao V5)
            cat /home/nao/wifi.config | grep "ssid" | sed 's/ //g' | cut -d "=" -f 2 | sed 's/"//g'
        else
            sudo wpa_cli status | grep "^ssid" | cut -d "=" -f 2
        fi
    fi
}

function help() {
    echo "usage: wifiState.sh [-s | -n | -h | --status | --name | --help]"
    echo -e "-s | --status \t show the wpa_cli connection status"
    echo -e "-n | --name \t show the wpa_cli ssid name"
    echo -e "-h | --help \t show this help"
}

function main() {
    key=$1

    if [[ $key == "-s" || $key == "--status" ]]; then
        status
    elif [[ $key == "-n" || $key == "--name" ]]; then
        name
    elif [[ $key == "-h" || $key == "--help" ]]; then
        help
    else
        help
    fi
}

main "$@"