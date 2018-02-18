#!/bin/bash

sudo /home/jdg/Scripts/fan_high.sh

NUM_EXP=100
BIN_PATH="../build/Release/bin"

R2_STEER=0.3
R4_STEER=0.5
R8_STEER=0.9
R16_STEER=1.7

R2_TIME=3
R4_TIME=30
R8_TIME=150
R16_TIME=300

###Regular worlds###
#echo && echo && echo && echo "Regular-grid problems require using children.reserve() in RRT* constructor. Recompile BOTH OMPL and the tests." && echo && echo && echo
$BIN_PATH/bitstar_journal_regular -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_regular -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
#Do not use the reserve trick for R16, it hits swap.
$BIN_PATH/bitstar_journal_regular -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME

###Enclosure worlds###
#echo && echo && echo && echo "Regular-grid problems require using children.reserve() in RRT* constructor. Recompile BOTH OMPL and the tests." && echo && echo && echo
$BIN_PATH/bitstar_journal_double_enclosure -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_double_enclosure -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME
$BIN_PATH/bitstar_journal_double_enclosure -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
#Do not use the reserve trick for R16, it hits swap.
#####$BIN_PATH/bitstar_journal_double_enclosure -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME

###Random world###
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME
$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME

$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME
$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME

$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME
$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME

sudo /home/jdg/Scripts/fan_auto.sh
