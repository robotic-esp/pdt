#!/bin/bash

STEER=0.4

###Convergence rates###
#../build/Release/bin/informed_journal_converge_rate -r 2 -s -1 -x -1 -t 10000 -i 1000
#../build/Release/bin/informed_journal_converge_rate -r 4 -s -1 -x -1 -t 10000 -i 1000
#../build/Release/bin/informed_journal_converge_rate -r 8 -s -1 -x -1 -t 10000 -i 1000

../build/Release/bin/informed_journal_converge_rate -r 2 -s $STEER -x -1 -t 10000 -i 5000
../build/Release/bin/informed_journal_converge_rate -r 4 -s $STEER -x -1 -t 10000 -i 5000
../build/Release/bin/informed_journal_converge_rate -r 8 -s $STEER -x -1 -t 10000 -i 5000

#../build/Release/bin/informed_journal_converge_rate -r 2 -s -1 -x 1.1 -t 10000 -i 5000
#../build/Release/bin/informed_journal_converge_rate -r 4 -s -1 -x 1.1 -t 10000 -i 5000
../build/Release/bin/informed_journal_converge_rate -r 8 -s -1 -x 1.1 -t 10000 -i 5000

../build/Release/bin/informed_journal_converge_rate -r 2 -s $STEER -x 1.1 -t 10000 -i 5000
../build/Release/bin/informed_journal_converge_rate -r 4 -s $STEER -x 1.1 -t 10000 -i 5000
../build/Release/bin/informed_journal_converge_rate -r 8 -s $STEER -x 1.1 -t 10000 -i 5000
