#!/bin/bash
echo -e "\e[32mRemoving old csv files and creating new ones\e[0m"

path_temp="$(rospack find detection)"
rm -f "$path_temp"/human.csv "$path_temp"/clicked_humans.csv "$path_temp"/sign.csv
touch "$path_temp"/human.csv "$path_temp"/clicked_humans.csv "$path_temp"/sign.csv


path_temp="$(rospack find gcount)"
rm -f "$path_temp"/gcounts_neu.csv "$path_temp"/gcounts_neu_svh.csv
touch "$path_temp"/gcounts_neu.csv "$path_temp"/gcounts_neu_svh.csv

