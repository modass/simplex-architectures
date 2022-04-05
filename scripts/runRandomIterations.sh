#!/bin/bash

binary=$1
iterations=200
seedOffset=0

# determine seed offset: each line in baseControllerInvocations has been used with the 0-indexed line number
if [ -f "baseControllerInvocations" ]; then
    seedOffset=$(wc -l < baseControllerInvocations)
    printf "Offset random seed by $seedOffset \n"
fi

for i in $(seq 1 $iterations);
do
    printf "Start iteration $i (press 'q' to quit)?\n"
    seed=$(($i+$seedOffset))
    # delete old storage, if present
    if [ -f "teststorage" ]; then
        rm teststorage
    fi
    #read -n 1 k <&1
    k="a"
    if [[ $k = q ]] ; then
        printf "\nQuitting from the program\n"
        break
    else
        $binary -m /home/stefan/tu/repositories/simplex-architectures/models/watertanks.model -i 600 --learn -s teststorage --random --seed $seed
        printf "Done iteration $i\n"
    fi
done
