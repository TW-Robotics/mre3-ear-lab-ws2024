#!/bin/bash
usage() {
    echo "Usage: $0 
    roscore or rosmaster must still be running!

    [-m     Generates a 2D map (.png & .yaml) and the octomap file (.ot)  ]
    [       and a 2D map with detected Signs and Humans marked in it      ]
    [-r     Generates the Radiation Heatmap                               ]" 1>&2
    exit 1
}

create_maps() {
    # Creatin the Maps #
    echo -e "\e[32mSaving the map number $cnt to $path$datum\e[0m"
    echo -e "\e[33mRunning:
    rosrun map_server map_saver -f $path$datum/Map_$cnt\e[0m"
    rosrun map_server map_saver -f "$path""$datum"/Map_"$cnt"
    sed -i "s#$path$datum/Map_$cnt.pgm#Map_$cnt.pgm#g" ./Map_"$cnt".yaml
    echo -e "\e[33mRunning:
    rosrun octomap_server octomap_saver -f $path$datum/Map_$cnt.ot\e[0m"
    rosrun octomap_server octomap_saver -f "$path""$datum"/Map_"$cnt".ot

    # Check if everything for draw_map_symbols is available #
    path_temp="$(rospack find detection)"
    if [ -f "$path_temp"/human.csv ]; then
        flag_humans=true
        cp "$path_temp"/human.csv "$path""$datum"/human_"$cnt".csv
        if [ -f "$path_temp"/sign.csv ]; then
            cp "$path_temp"/sign.csv "$path""$datum"/sign_"$cnt".csv
        fi
    else
        flag_humans=false
        echo -e "\e[31mCould not find all files for Human-Sign Map generation -> skipping \e[0m"
    fi
    echo "humans: $flag_humans"
    if [ "$flag_humans" == true ]; then
        echo -e "\e[33mRunning:
        draw_map_symbols $path$datum/Map_$cnt.pgm $path$datum/Map_$cnt.yaml $path$datum/humans_$cnt.csv 1\e[0m"
        draw_map_symbols "$path""$datum"/Map_"$cnt".pgm "$path""$datum"/Map_"$cnt".yaml "$path""$datum"/human_"$cnt".csv "$path""$datum"/sign_"$cnt".csv 1
        mv ~/git/MR_ELROB2018/Installation/programms/draw_map/build/map_sign.ppm "$path""$datum"/Map_Human_Sign_"$cnt".ppm
    fi
    path_temp="$(rospack find gcount)"
    cp "$path_temp"/gcounts_neu.csv "$path""$datum"/gcounts_neu_"$cnt".csv
    cp "$path_temp"/gcounts_neu_svh.csv "$path""$datum"/gcounts_neu_svh_"$cnt".csv

}

create_radio_map() {
    #if [ $cnt -ge 0 ]; then
    #cnt=$((cnt -1))
    path_temp="$(rospack find using_markers)"
    if [ -f "$(rospack find gcount)"/gcounts_neu.csv ]; then
        flag_rad=true
        cd "$path_temp"/src || exit 1
        echo -e "\e[33mRunning:
            python gaussian.py
        \e[31m...This will take a while...\e[0m"
        python gaussian.py
    else
        flag_rad=false
        echo -e "\e[31mCould not find all files for Radiation Heatmap generation -> skipping \e[0m"
    fi
    path_temp="$(rospack find gcount)"
    cd "$path_temp" || exit 1
    if [ -f "$path_temp"/gauss_out.csv ]; then
        flag_rad=true
        cp gauss_out.csv "$path""$datum"/gauss_out_"$cnt".csv
    else
        flag_rad=false
    fi
    echo "rad: $flag_rad"
    # Draw everything #
    if [ "$flag_rad" == true ]; then
        echo -e "\e[33mRunning:
        draw_map_radiological $path$datum/Map_$cnt.pgm $path$datum/Map_$cnt.yaml $path$datum/gauss_out_$cnt.csv 0.10 0\e[0m"
        draw_map_radiological "$path""$datum"/Map_"$cnt".pgm "$path""$datum"/Map_"$cnt".yaml "$path""$datum"/gauss_out_"$cnt".csv 0.10 0
        mv ~/git/MR_ELROB2018/Installation/programms/draw_map/build/map_gauss.ppm "$path""$datum"/Map_Radiation_"$cnt".ppm
    fi
}

main() {
    if [ "$human" == true ]; then
        create_maps
    fi
    if [ "$radio" == true ]; then
        create_radio_map
    fi

}

if [ "$(pgrep roscore)" ] || [ "$(pgrep rosmaster)" ]; then
    while getopts "mr" o; do
        case "${o}" in
        m)
            human=true
            echo "Human and Sign Map will be generated if the necessary files are available"
            ;;
        r)
            radio=true
            echo "Radiation HeatMap will be generated if the necessary files are available"
            ;;
        *)
            usage
            ;;
        esac
    done

    flag_rad=false
    flag_humans=false

    datum="$(date +%d_%m_%Y)"
    path=$HOME/maps/
    mkdir -p "$path""$datum"
    cd "$path""$datum" || exit 1
    cnt="$(find "$(cd ..; pwd)" -type f -name "*hook" | wc -l)"

    echo "cnt= $cnt"

    main
    if [ -z "$human" ] && [ -z "$radio" ]; then
        echo "  You forgot to provide an argument"
        usage
        exit 2
    fi
    echo -e "\e[32m     Finished saving to $path$datum\e[0m"
    exit 0
else
    echo -e "\e[31mNo roscore or rosmaster running -> aborting\e[0m"
    usage 3
fi
