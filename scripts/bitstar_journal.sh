#!/bin/bash

sudo /home/jdg/Scripts/fan_high.sh

R2_STEER=0.3
#R4_STEER=0.5
R8_STEER=0.9
R16_STEER=1.7

R2_TIME=3
#R4_TIME=30
R8_TIME=150
R16_TIME=300

###Target tolerance###
../build/Release_1_58_0/bin/bitstar_journal_targets -r 2 -s $R2_STEER -p TimeVTarget -v 1.05 1.04 1.03 1.02 1.01 1.009 1.008 1.007 1.006 1.005 1.004 1.003 1.002 1.001 1.0005 1.0001 -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_targets -r 8 -s $R8_STEER -p TimeVTarget -v 1.10 1.09 1.08 1.07 1.06 1.05 1.04 1.03 1.02 1.01 -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_targets -r 16 -s $R16_STEER -p TimeVTarget -v 1.10 1.09 1.08 1.07 1.06 1.05 1.04 1.03 1.02 1.01 -e 100 -t $R16_TIME

###Map size###
../build/Release_1_58_0/bin/bitstar_journal_targets -r 2 -s $R2_STEER -p TimeVMapSize -v 2.0 5.0 10.0 20.0 30.0 40.0 50.0 -g 1.01 -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_targets -r 8 -s $R8_STEER -p TimeVMapSize -v 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 -g 1.15 -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_targets -r 16 -s $R16_STEER -p TimeVMapSize -v 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 -g 1.15 -e 100 -t $R16_TIME

###Regular worlds###
../build/Release_1_58_0/bin/bitstar_journal_regular -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_regular -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_regular -r 16 -s $R16_STEER -e 100 -t $R16_TIME


###Random world###
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 2 -s $R2_STEER -e 100 -t $R2_TIME

../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 8 -s $R8_STEER -e 100 -t $R8_TIME

../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME
../build/Release_1_58_0/bin/bitstar_journal_random_single -r 16 -s $R16_STEER -e 100 -t $R16_TIME

sudo /home/jdg/Scripts/fan_auto.sh
