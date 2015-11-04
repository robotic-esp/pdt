#!/bin/bash

sudo /home/jdg/Scripts/fan_high.sh

R2_STEER=0.3
R4_STEER=0.5
R8_STEER=0.9

###Target tolerance###
../build/Release_1_58_0/bin/informed_journal_targets -r 2 -s $R2_STEER -p TimeVTarget -v 1.05 1.04 1.03 1.02 1.01 1.009 1.008 1.007 1.006 1.005 1.004 1.003 1.002 1.001 1.0005 1.0001 -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_targets -r 4 -s $R4_STEER -p TimeVTarget -v 1.10 1.09 1.08 1.07 1.06 1.05 1.04 1.03 1.02 1.01 1.005 -e 100 -t 60
../build/Release_1_58_0/bin/informed_journal_targets -r 8 -s $R8_STEER -p TimeVTarget -v 1.10 1.09 1.08 1.07 1.06 1.05 1.04 1.03 1.02 1.01 -e 100 -t 300

###Map size###
../build/Release_1_58_0/bin/informed_journal_targets -r 2 -s $R2_STEER -p TimeVMapSize -v 2.0 5.0 10.0 20.0 30.0 40.0 50.0 -g 1.01 -e 100 -t 3 #was -g 1.01
../build/Release_1_58_0/bin/informed_journal_targets -r 4 -s $R4_STEER -p TimeVMapSize -v 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 -g 1.05 -e 100 -t 60 #was -g 1.05
../build/Release_1_58_0/bin/informed_journal_targets -r 8 -s $R8_STEER -p TimeVMapSize -v 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 -g 1.10 -e 100 -t 300 #was -g 1.25

###Random world###
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3
../build/Release_1_58_0/bin/informed_journal_random_single -r 2 -s $R2_STEER -e 100 -t 3

../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30
../build/Release_1_58_0/bin/informed_journal_random_single -r 4 -s $R4_STEER -e 100 -t 30

../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150
../build/Release_1_58_0/bin/informed_journal_random_single -r 8 -s $R8_STEER -e 100 -t 150

sudo /home/jdg/Scripts/fan_auto.sh
