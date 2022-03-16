# usage : ./benchmark.sh data_dir sol_dir model

echo Experimental Campaign: Traveling Salesman Problem
echo Data directory: $1
echo Output directory: $2
echo Model: $3

mkdir -p $2 # create the output directory if it does not already exist
echo `date` > $2/date.txt

for instance in `ls $1` ; do  # for each instance in directory $1
    echo Resolution of $instance
    cd build
    ./$3.out $1/$instance -nv >> ../$2/log_${instance}.txt   # writing console output to a log file
done

grep Result $2/*.txt >> $2/results.csv  # lines containing the word "Result" will be concatenated in the results.csv file