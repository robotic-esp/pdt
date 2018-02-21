#!/bin/bash

sudo /home/jdg/Scripts/fan_high.sh

sleep 3

NUM_EXP=100
BIN_PATH="../build/Release/bin"

R2_STEER=0.3
R4_STEER=0.5
R8_STEER=0.9
R16_STEER=1.7

#planners x trials x time
#R2:  10*100*1 = 0.25 hours x 14 bin: 4 hours (0.1666 days)
#R4:  10*100*10 = 2.75 hours x 12 bin: 1.4 days
#R8:  10*100*30 = 8.5 hours x 12 bin: 4.16 days
#R16: 10*100*100 = 28 hours x 12 bin: 14 days

R2_TIME=1
R4_TIME=10
R8_TIME=30
R16_TIME=100

R2_LOG_INTERVAL=100
R4_LOG_INTERVAL=100
R8_LOG_INTERVAL=100
R16_LOG_INTERVAL=1000

###Response Problems###
# $BIN_PATH/bitstar_journal_rrtsharp_response -p 1 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
# $BIN_PATH/bitstar_journal_rrtsharp_response -p 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL

###Regular worlds###
# $BIN_PATH/bitstar_journal_regular -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
# $BIN_PATH/bitstar_journal_regular -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
$BIN_PATH/bitstar_journal_regular -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_regular -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL

###Enclosure worlds###
# $BIN_PATH/bitstar_journal_double_enclosure -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
# $BIN_PATH/bitstar_journal_double_enclosure -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
# $BIN_PATH/bitstar_journal_double_enclosure -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_double_enclosure -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL

###Random world###
# $BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 2 -s $R2_STEER -e $NUM_EXP -t $R2_TIME -i $R2_LOG_INTERVAL

# $BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 4 -s $R4_STEER -e $NUM_EXP -t $R4_TIME -i $R4_LOG_INTERVAL

$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 8 -s $R8_STEER -e $NUM_EXP -t $R8_TIME -i $R8_LOG_INTERVAL

#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL
#$BIN_PATH/bitstar_journal_random_single -r 16 -s $R16_STEER -e $NUM_EXP -t $R16_TIME -i $R16_LOG_INTERVAL

sudo /home/jdg/Scripts/fan_auto.sh
