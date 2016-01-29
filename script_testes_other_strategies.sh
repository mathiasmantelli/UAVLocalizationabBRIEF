#!/bin/bash

numTests=$1

execPath="../build-PhiR2Framework-Desktop-Release/PhiR2Framework"

Maps=(
#"/home/mathias/Documentos/Datasets/ufrgs_google_longterm/adjusted/median_goodplusnew"
"/home/mathias/Documentos/Datasets/ufrgs_google_longterm/adjusted/median_new"
)

MapsNames=(
"median_new"
)
#echo "NumMaps" ${#Maps[@]}

trajsPath="/home/mathias/Documentos/Datasets/ufrgs_dronao_28ago/"
Trajs=(
#"traj4_460_mod_off"
"traj4_460_gt_mod"
#"traj3_mod"
#"traj3_gt_mod"
#"traj3_good"
#"traj3_gt_good"
#"traj3_eq_good"
#"traj3_eq_gt_good"
#"traj2_mod_off"
#"traj2_gt_mod"
)

Trajs=("${Trajs[$2]}")
echo "${Trajs[@]}"

#numConfigs=4
#lenConfig=1
Configs=(

"-s color diff-intensity 9 -s density diff-intensity 9 circular 15"
"-s unscented diff-intensity 9 -s density diff-intensity 9 circular 15"
"-s color diff-intensity 9 -s density diff-intensity 9 circular 10"
"-s unscented diff-intensity 9 -s density diff-intensity 9 circular 10"

"-s color diff-intensity 10 -s density diff-intensity 10 circular 15"
"-s unscented diff-intensity 10 -s density diff-intensity 10 circular 15"
"-s color diff-intensity 10 -s density diff-intensity 10 circular 10"
"-s unscented diff-intensity 10 -s density diff-intensity 10 circular 10"

"-s color diff-intensity 11 -s density diff-intensity 11 circular 15"
"-s unscented diff-intensity 11 -s density diff-intensity 11 circular 15"
"-s color diff-intensity 11 -s density diff-intensity 11 circular 10"
"-s unscented diff-intensity 11 -s density diff-intensity 11 circular 10"

"-s color diff-cie1976 10 -s density diff-cie1976 10 circular 15"
"-s unscented diff-cie1976 10 -s density diff-cie1976 10 circular 15"
"-s color diff-cie1976 10 -s density diff-cie1976 10 circular 10"
"-s unscented diff-cie1976 10 -s density diff-cie1976 10 circular 10"

"-s color diff-cie1976 11 -s density diff-cie1976 11 circular 15"
"-s unscented diff-cie1976 11 -s density diff-cie1976 11 circular 15"
"-s color diff-cie1976 11 -s density diff-cie1976 11 circular 10"
"-s unscented diff-cie1976 11 -s density diff-cie1976 11 circular 10"

"-s color diff-cie1994 11 -s density diff-cie1994 11 circular 15"
"-s unscented diff-cie1994 11 -s density diff-cie1994 11 circular 15"
"-s color diff-cie1994 11 -s density diff-cie1994 11 circular 10"
"-s unscented diff-cie1994 11 -s density diff-cie1994 11 circular 10"

"-s color diff-cie1976 12 -s density diff-cie1976 12 circular 15"
"-s unscented diff-cie1976 12 -s density diff-cie1976 12 circular 15"
"-s color diff-cie1976 12 -s density diff-cie1976 12 circular 10"
"-s unscented diff-cie1976 12 -s density diff-cie1976 12 circular 10"

"-s color diff-cie2000 8 -s density diff-cie2000 8 circular 15"
"-s unscented diff-cie2000 8 -s density diff-cie2000 8 circular 15"
"-s color diff-cie2000 8 -s density diff-cie2000 8 circular 10"
"-s unscented diff-cie2000 8 -s density diff-cie2000 8 circular 10"

"-s color diff-cie2000 9 -s density diff-cie2000 9 circular 15"
"-s unscented diff-cie2000 9 -s density diff-cie2000 9 circular 15"
"-s color diff-cie2000 9 -s density diff-cie2000 9 circular 10"
"-s unscented diff-cie2000 9 -s density diff-cie2000 9 circular 10"

"-s color diff-cie2000 10 -s density diff-cie2000 10 circular 15"
"-s unscented diff-cie2000 10 -s density diff-cie2000 10 circular 15"
"-s color diff-cie2000 10 -s density diff-cie2000 10 circular 10"
"-s unscented diff-cie2000 10 -s density diff-cie2000 10 circular 10"

"-s color diff-cmc1984 9 -s density diff-cmc1984 9 circular 15"
"-s unscented diff-cmc1984 9 -s density diff-cmc1984 9 circular 15"
"-s color diff-cmc1984 9 -s density diff-cmc1984 9 circular 10"
"-s unscented diff-cmc1984 9 -s density diff-cmc1984 9 circular 10"

"-s color diff-cmc1984 10 -s density diff-cmc1984 10 circular 15"
"-s unscented diff-cmc1984 10 -s density diff-cmc1984 10 circular 15"
"-s color diff-cmc1984 10 -s density diff-cmc1984 10 circular 10"
"-s unscented diff-cmc1984 10 -s density diff-cmc1984 10 circular 10"

"-s color diff-cmc1984 11 -s density diff-cmc1984 11 circular 15"
"-s unscented diff-cmc1984 11 -s density diff-cmc1984 11 circular 15"
"-s color diff-cmc1984 11 -s density diff-cmc1984 11 circular 10"
"-s unscented diff-cmc1984 11 -s density diff-cmc1984 11 circular 10"

"-s color diff-rgb 12 -s density diff-rgb 12 circular 15"
"-s unscented diff-rgb 12 -s density diff-rgb 12 circular 15"
"-s color diff-rgb 12 -s density diff-rgb 12 circular 10"
"-s unscented diff-rgb 12 -s density diff-rgb 12 circular 10"

"-s color diff-rgb 13 -s density diff-rgb 13 circular 15"
"-s unscented diff-rgb 13 -s density diff-rgb 13 circular 15"
"-s color diff-rgb 13 -s density diff-rgb 13 circular 10"
"-s unscented diff-rgb 13 -s density diff-rgb 13 circular 10"

"-s color diff-cie1994 12 -s density diff-cie1994 12 circular 15"
"-s unscented diff-cie1994 12 -s density diff-cie1994 12 circular 15"
"-s color diff-cie1994 12 -s density diff-cie1994 12 circular 10"
"-s unscented diff-cie1994 12 -s density diff-cie1994 12 circular 10"


#"-s color diff-intensity 50 -s density diff-intensity 35.640406 circular 10"
#"-s color diff-intensity 50 -s density diff-intensity 35.640406 circular 20"  
#"-s color diff-intensity 50 -s density diff-intensity 35.640406 inverted 5"
#"-s color diff-intensity 50 -s density diff-intensity 35.640406 gaussian 5"
#"-s color diff-intensity 25 -s density diff-intensity 35.640406 circular 10"
#"-s color diff-intensity 25 -s density diff-intensity 35.640406 circular 20"  
#"-s color diff-intensity 25 -s density diff-intensity 35.640406 inverted 5"
#"-s color diff-intensity 25 -s density diff-intensity 35.640406 gaussian 5"
#"-s color diff-cmc1984 25 -s density diff-cmc1984 24.400485 circular 10"
#"-s color diff-cmc1984 25 -s density diff-cmc1984 24.400485 circular 20"
#"-s color diff-cmc1984 25 -s density diff-cmc1984 24.400485 gaussian 10"
#"-s color diff-cmc1984 25 -s density diff-cmc1984 24.400485 inverted 10"
#"-s color diff-cmc1984 40 -s density diff-cmc1984 24.400485 circular 10"
#"-s color diff-cmc1984 40 -s density diff-cmc1984 24.400485 circular 20"
#"-s color diff-cmc1984 40 -s density diff-cmc1984 24.400485 gaussian 10"
#"-s color diff-cmc1984 40 -s density diff-cmc1984 24.400485 inverted 10"
#"-s color diff-rgb 15 -s density diff-rgb 48.630549 circular 10"
#"-s color diff-rgb 15 -s density diff-rgb 63.503281 circular 10"
#"-s color diff-rgb 15 -s density diff-rgb 63.503281 gaussian 5"
#"-s color diff-rgb 15 -s density diff-rgb 63.503281 inverted 5"
#"-s color diff-rgb 40 -s density diff-rgb 48.630549 circular 10"
#"-s color diff-rgb 40 -s density diff-rgb 63.503281 circular 10"
#"-s color diff-rgb 40 -s density diff-rgb 63.503281 gaussian 5"
#"-s color diff-rgb 40 -s density diff-rgb 63.503281 inverted 5"
#"-s color diff-cie2000 15 -s density diff-cie2000 13.889990 circular 10"
#"-s color diff-cie2000 15 -s density diff-cie2000 13.889990 circular 20"
#"-s color diff-cie2000 15 -s density diff-cie2000 13.889990 inverted 10"
#"-s color diff-cie2000 15 -s density diff-cie2000 13.889990 gaussian 10"
#"-s color diff-cie2000 20 -s density diff-cie2000 13.889990 circular 10"
#"-s color diff-cie2000 20 -s density diff-cie2000 13.889990 circular 20"
#"-s color diff-cie2000 20 -s density diff-cie2000 13.889990 inverted 10"
#"-s color diff-cie2000 20 -s density diff-cie2000 13.889990 gaussian 10"
#"-s color diff-intensity 25 -s entropy diff-intensity 2 circular 10"
#"-s color diff-intensity 25 -s entropy diff-intensity 8 circular 10"
#"-s color diff-intensity 25 -s entropy diff-intensity 8 gaussian 10"
#"-s color diff-intensity 25 -s entropy diff-intensity 8 inverted 10"
#"-s color diff-intensity 50 -s entropy diff-intensity 2 circular 10"
#"-s color diff-intensity 50 -s entropy diff-intensity 8 circular 10"
#"-s color diff-intensity 50 -s entropy diff-intensity 8 gaussian 10"
#"-s color diff-intensity 50 -s entropy diff-intensity 8 inverted 10"
#"-s color diff-cie2000 15 -s entropy diff-cie1976 2 circular 10"
#"-s color diff-cie2000 15 -s entropy diff-cie1976 4 circular 10"
#"-s color diff-cie2000 15 -s entropy diff-cie1976 8 circular 10"
#"-s color diff-cie2000 15 -s entropy diff-cie1976 8 gaussian 10"
#"-s color diff-cie2000 15 -s entropy diff-cie1976 8 inverted 10"
#"-s color diff-cie2000 20 -s entropy diff-cie1976 2 circular 10"
#"-s color diff-cie2000 20 -s entropy diff-cie1976 4 circular 10"
#"-s color diff-cie2000 20 -s entropy diff-cie1976 8 circular 10"
#"-s color diff-cie2000 20 -s entropy diff-cie1976 8 gaussian 10"
#"-s color diff-cie2000 20 -s entropy diff-cie1976 8 inverted 10"
#"-s color diff-rgb 15 -s entropy diff-rgb 2 circular 10"
#"-s color diff-rgb 15 -s entropy diff-rgb 8 circular 10"
#"-s color diff-rgb 15 -s entropy diff-rgb 8 gaussian 10"
#"-s color diff-rgb 15 -s entropy diff-rgb 8 inverted 10"
#"-s color diff-rgb 40 -s entropy diff-rgb 2 circular 10"
#"-s color diff-rgb 40 -s entropy diff-rgb 8 circular 10"
#"-s color diff-rgb 40 -s entropy diff-rgb 8 gaussian 10"
#"-s color diff-rgb 40 -s entropy diff-rgb 8 inverted 10"
#"-s color diff-intensity 25 -s mi diff-intensity 2 circular 10"
#"-s color diff-intensity 25 -s mi diff-intensity 8 circular 10"
#"-s color diff-intensity 50 -s mi diff-intensity 2 circular 10"
#"-s color diff-intensity 50 -s mi diff-intensity 8 circular 10"
#"-s color diff-cie2000 15 -s mi diff-cie1976 2 circular 10"
#"-s color diff-cie2000 15 -s mi diff-cie1976 8 circular 10"
#"-s color diff-cie2000 20 -s mi diff-cie1976 2 circular 10"
#"-s color diff-cie2000 20 -s mi diff-cie1976 8 circular 10"
#"-s color diff-rgb 15 -s mi diff-rgb 2 circular 10"
#"-s color diff-rgb 15 -s mi diff-rgb 8 circular 10"
#"-s color diff-rgb 40 -s mi diff-rgb 2 circular 10"
#"-s color diff-rgb 40 -s mi diff-rgb 8 circular 10"
)

Outputs=(
#"s_color_diff-intensity_50_-s_density_diff-intensity_35.640406_circular_10"
#"s_color_diff-intensity_50_-s_density_diff-intensity_35.640406_circular_20"
#"s_color_diff-intensity_50_-s_density_diff-intensity_35.640406_inverted_5"
#"s_color_diff-intensity_50_-s_density_diff-intensity_35.640406_gaussian_5"
#"s_color_diff-intensity_25_-s_density_diff-intensity_35.640406_circular_10"
#"s_color_diff-intensity_25_-s_density_diff-intensity_35.640406_circular_20"
#"s_color_diff-intensity_25_-s_density_diff-intensity_35.640406_inverted_5"
#"s_color_diff-intensity_25_-s_density_diff-intensity_35.640406_gaussian_5"
#"s_color_diff-cmc1984_25_-s_density_diff-cmc1984_24.400485_circular_10"
#"s_color_diff-cmc1984_25_-s_density_diff-cmc1984_24.400485_circular_20"
#"s_color_diff-cmc1984_25_-s_density_diff-cmc1984_24.400485_gaussian_10"
#"s_color_diff-cmc1984_25_-s_density_diff-cmc1984_24.400485_inverted_10"
#"s_color_diff-cmc1984_40_-s_density_diff-cmc1984_24.400485_circular_10"
#"s_color_diff-cmc1984_40_-s_density_diff-cmc1984_24.400485_circular_20"
#"s_color_diff-cmc1984_40_-s_density_diff-cmc1984_24.400485_gaussian_10"
#"s_color_diff-cmc1984_40_-s_density_diff-cmc1984_24.400485_inverted_10"
#"s_color_diff-rgb_15_-s_density_diff-rgb_48.630549_circular_10"
#"s_color_diff-rgb_15_-s_density_diff-rgb_63.503281_circular_10"
#"s_color_diff-rgb_15_-s_density_diff-rgb_63.503281_gaussian_5"
#"s_color_diff-rgb_15_-s_density_diff-rgb_63.503281_inverted_5"
#"s_color_diff-rgb_40_-s_density_diff-rgb_48.630549_circular_10"
#"s_color_diff-rgb_40_-s_density_diff-rgb_63.503281_circular_10"
#"s_color_diff-rgb_40_-s_density_diff-rgb_63.503281_gaussian_5"
#"s_color_diff-rgb_40_-s_density_diff-rgb_63.503281_inverted_5"
#"s_color_diff-cie2000_15_-s_density_diff-cie2000_13.889990_circular_10"
#"s_color_diff-cie2000_15_-s_density_diff-cie2000_13.889990_circular_20"
#"s_color_diff-cie2000_15_-s_density_diff-cie2000_13.889990_inverted_10"
#"s_color_diff-cie2000_15_-s_density_diff-cie2000_13.889990_gaussian_10"
#"s_color_diff-cie2000_20_-s_density_diff-cie2000_13.889990_circular_10"
#"s_color_diff-cie2000_20_-s_density_diff-cie2000_13.889990_circular_20"
#"s_color_diff-cie2000_20_-s_density_diff-cie2000_13.889990_inverted_10"
#"s_color_diff-cie2000_20_-s_density_diff-cie2000_13.889990_gaussian_10"
#"s_color_diff-intensity_25_-s_entropy_diff-intensity_2_circular_10"
#"s_color_diff-intensity_25_-s_entropy_diff-intensity_8_circular_10"
#"s_color_diff-intensity_25_-s_entropy_diff-intensity_8_gaussian_10"
#"s_color_diff-intensity_25_-s_entropy_diff-intensity_8_inverted_10"
#"s_color_diff-intensity_50_-s_entropy_diff-intensity_2_circular_10"
#"s_color_diff-intensity_50_-s_entropy_diff-intensity_8_circular_10"
#"s_color_diff-intensity_50_-s_entropy_diff-intensity_8_gaussian_10"
#"s_color_diff-intensity_50_-s_entropy_diff-intensity_8_inverted_10"
#"s_color_diff-cie2000_15_-s_entropy_diff-cie1976_2_circular_10"
#"s_color_diff-cie2000_15_-s_entropy_diff-cie1976_4_circular_10"
#"s_color_diff-cie2000_15_-s_entropy_diff-cie1976_8_circular_10"
#"s_color_diff-cie2000_15_-s_entropy_diff-cie1976_8_gaussian_10"
#"s_color_diff-cie2000_15_-s_entropy_diff-cie1976_8_inverted_10"
#"s_color_diff-cie2000_20_-s_entropy_diff-cie1976_2_circular_10"
#"s_color_diff-cie2000_20_-s_entropy_diff-cie1976_4_circular_10"
#"s_color_diff-cie2000_20_-s_entropy_diff-cie1976_8_circular_10"
#"s_color_diff-cie2000_20_-s_entropy_diff-cie1976_8_gaussian_10"
#"s_color_diff-cie2000_20_-s_entropy_diff-cie1976_8_inverted_10"
#"s_color_diff-rgb_15_-s_entropy_diff-rgb_2_circular_10"
#"s_color_diff-rgb_15_-s_entropy_diff-rgb_8_circular_10"
#"s_color_diff-rgb_15_-s_entropy_diff-rgb_8_gaussian_10"
#"s_color_diff-rgb_15_-s_entropy_diff-rgb_8_inverted_10"
#"s_color_diff-rgb_40_-s_entropy_diff-rgb_2_circular_10"
#"s_color_diff-rgb_40_-s_entropy_diff-rgb_8_circular_10"
#"s_color_diff-rgb_40_-s_entropy_diff-rgb_8_gaussian_10"
#"s_color_diff-rgb_40_-s_entropy_diff-rgb_8_inverted_10"
#"s_color_diff-intensity_25_-s_mi_diff-intensity_2_circular_10"
#"s_color_diff-intensity_25_-s_mi_diff-intensity_8_circular_10"
#"s_color_diff-intensity_50_-s_mi_diff-intensity_2_circular_10"
#"s_color_diff-intensity_50_-s_mi_diff-intensity_8_circular_10"
#"s_color_diff-cie2000_15_-s_mi_diff-cie1976_2_circular_10"
#"s_color_diff-cie2000_15_-s_mi_diff-cie1976_8_circular_10"
#"s_color_diff-cie2000_20_-s_mi_diff-cie1976_2_circular_10"
#"s_color_diff-cie2000_20_-s_mi_diff-cie1976_8_circular_10"
#"s_color_diff-rgb_15_-s_mi_diff-rgb_2_circular_10"
#"s_color_diff-rgb_15_-s_mi_diff-rgb_8_circular_10"
#"s_color_diff-rgb_40_-s_mi_diff-rgb_2_circular_10"
#"s_color_diff-rgb_40_-s_mi_diff-rgb_8_circular_10"

"s_color_diff-intensity_9_-s_density_diff-intensity_9_circular_15"
"s_unscented_diff-intensity_9_-s_density_diff-intensity_9_circular_15"
"s_color_diff-intensity_9_-s_density_diff-intensity_9_circular_10"
"s_unscented_diff-intensity_9_-s_density_diff-intensity_9_circular_10"

"s_color_diff-intensity_10_-s_density_diff-intensity_10_circular_15"
"s_unscented_diff-intensity_10_-s_density_diff-intensity_10_circular_15"
"s_color_diff-intensity_10_-s_density_diff-intensity_10_circular_10"
"s_unscented_diff-intensity_10_-s_density_diff-intensity_10_circular_10"

"s_color_diff-intensity_11_-s_density_diff-intensity_11_circular_15"
"s_unscented_diff-intensity_11_-s_density_diff-intensity_11_circular_15"
"s_color_diff-intensity_11_-s_density_diff-intensity_11_circular_10"
"s_unscented_diff-intensity_11_-s_density_diff-intensity_11_circular_10"

"s_color_diff-cie1976_10_-s_density_diff-cie1976_10_circular_15"
"s_unscented_diff-cie1976_10_-s_density_diff-cie1976_10_circular_15"
"s_color_diff-cie1976_10_-s_density_diff-cie1976_10_circular_10"
"s_unscented_diff-cie1976_10_-s_density_diff-cie1976_10_circular_10"

"s_color_diff-cie1976_11_-s_density_diff-cie1976_11_circular_15"
"s_unscented_diff-cie1976_11_-s_density_diff-cie1976_11_circular_15"
"s_color_diff-cie1976_11_-s_density_diff-cie1976_11_circular_10"
"s_unscented_diff-cie1976_11_-s_density_diff-cie1976_11_circular_10"

"s_color_diff-cie1994_11_-s_density_diff-cie1994_11_circular_15"
"s_unscented_diff-cie1994_11_-s_density_diff-cie1994_11_circular_15"
"s_color_diff-cie1994_11_-s_density_diff-cie1994_11_circular_10"
"s_unscented_diff-cie1994_11_-s_density_diff-cie1994_11_circular_10"

"s_color_diff-cie1976_12_-s_density_diff-cie1976_12_circular_15"
"s_unscented_diff-cie1976_12_-s_density_diff-cie1976_12_circular_15"
"s_color_diff-cie1976_12_-s_density_diff-cie1976_12_circular_10"
"s_unscented_diff-cie1976_12_-s_density_diff-cie1976_12_circular_10"

"s_color_diff-cie2000_8_-s_density_diff-cie2000_8_circular_15"
"s_unscented_diff-cie2000_8_-s_density_diff-cie2000_8_circular_15"
"s_color_diff-cie2000_8_-s_density_diff-cie2000_8_circular_10"
"s_unscented_diff-cie2000_8_-s_density_diff-cie2000_8_circular_10"

"s_color_diff-cie2000_9_-s_density_diff-cie2000_9_circular_15"
"s_unscented_diff-cie2000_9_-s_density_diff-cie2000_9_circular_15"
"s_color_diff-cie2000_9_-s_density_diff-cie2000_9_circular_10"
"s_unscented_diff-cie2000_9_-s_density_diff-cie2000_9_circular_10"

"s_color_diff-cie2000_10_-s_density_diff-cie2000_10_circular_15"
"s_unscented_diff-cie2000_10_-s_density_diff-cie2000_10_circular_15"
"s_color_diff-cie2000_10_-s_density_diff-cie2000_10_circular_10"
"s_unscented_diff-cie2000_10_-s_density_diff-cie2000_10_circular_10"

"s_color_diff-cmc1984_9_-s_density_diff-cmc1984_9_circular_15"
"s_unscented_diff-cmc1984_9_-s_density_diff-cmc1984_9_circular_15"
"s_color_diff-cmc1984_9_-s_density_diff-cmc1984_9_circular_10"
"s_unscented_diff-cmc1984_9_-s_density_diff-cmc1984_9_circular_10"

"s_color_diff-cmc1984_10_-s_density_diff-cmc1984_10_circular_15"
"s_unscented_diff-cmc1984_10_-s_density_diff-cmc1984_10_circular_15"
"s_color_diff-cmc1984_10_-s_density_diff-cmc1984_10_circular_10"
"s_unscented_diff-cmc1984_10_-s_density_diff-cmc1984_10_circular_10"

"s_color_diff-cmc1984_11_-s_density_diff-cmc1984_11_circular_15"
"s_unscented_diff-cmc1984_11_-s_density_diff-cmc1984_11_circular_15"
"s_color_diff-cmc1984_11_-s_density_diff-cmc1984_11_circular_10"
"s_unscented_diff-cmc1984_11_-s_density_diff-cmc1984_11_circular_10"

"s_color_diff-rgb_12_-s_density_diff-rgb_12_circular_15"
"s_unscented_diff-rgb_12_-s_density_diff-rgb_12_circular_15"
"s_color_diff-rgb_12_-s_density_diff-rgb_12_circular_10"
"s_unscented_diff-rgb_12_-s_density_diff-rgb_12_circular_10"

"s_color_diff-rgb_13_-s_density_diff-rgb_13_circular_15"
"s_unscented_diff-rgb_13_-s_density_diff-rgb_13_circular_15"
"s_color_diff-rgb_13_-s_density_diff-rgb_13_circular_10"
"s_unscented_diff-rgb_13_-s_density_diff-rgb_13_circular_10"

"s_color_diff-cie1994_12_-s_density_diff-cie1994_12_circular_15"
"s_unscented_diff-cie1994_12_-s_density_diff-cie1994_12_circular_15"
"s_color_diff-cie1994_12_-s_density_diff-cie1994_12_circular_10"
"s_unscented_diff-cie1994_12_-s_density_diff-cie1994_12_circular_10"

)



#-e /home/mathias/Documentos/Datasets/ufrgs_google_longterm/adjusted/median_new -t /home/mathias/Documentos/Datasets/ufrgs_dronao_28ago/traj4_460_gt_mod.txt -s color diff-intensity 9 -s density diff-intensity 9 circular 15
#-e /home/mathias/Documentos/Datasets/ufrgs_google_longterm/adjusted/median_new -t /home/mathias/Documentos/Datasets/ufrgs_dronao_28ago/traj4_460_gt_mod.txt -s unscented diff-intensity 2


#for test in `seq 1 $numTests`
#do
for m in "${!Maps[@]}" #for m in "${Maps[@]}"
do
    map=${Maps[$m]}
    for c in "${!Configs[@]}"
    do
       config="${Configs[$c]}"
	   
       for traj in "${Trajs[@]}"
       do
            resultPath="Results/${MapsNames[$m]}/$traj/${Outputs[$c]}/"
#            echo "$resultPath"
            # Create dir for this test
            if [ ! -d "$resultPath" ] 
            then
                mkdir -p "$resultPath"
            fi 

			echo "-e $map -t $trajsPath$traj.txt $config -o "$resultPath" $count $(($count+$numTests-1))"

            # run test
           # echo "Running $traj $config"
            count=`ls "$resultPath" 2>/dev/null | wc -l`
			
     		./$execPath -quiet -e $map -t $trajsPath$traj.txt $config -o "$resultPath" $count $(($count+$numTests-1))
			
#            echo ./$execPath -quiet -e $map -t $trajsPath$traj.txt $config -o "$resultPath" $count $(($count+$numTests-1))
#            # Move log files to Results
#            ls -t Logs | head -n 1 | while read f
#            do
#                count=`ls "Results/${MapsNames[$m]}/$traj/$config $lim/" 2>/dev/null | wc -l`
#                mv "Logs/$f" "Results/${MapsNames[$m]}/$traj/$config $lim/$count.txt"
#            done
        done
    done
done 
#done   

