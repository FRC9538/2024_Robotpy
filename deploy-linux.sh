#!/usr/bin/env bash

robotSSID="9538_Bueford"
originalSSID="$(nmcli -t -f active,ssid dev wifi | grep -E '^yes' | cut -c 5-)"
if [[ "$originalSSID" != "$robotSSID" ]]; then
    nmcli connection up "$robotSSID" && robotpy deploy && nmcli connection up "$originalSSID"
else
    robotpy deploy
fi