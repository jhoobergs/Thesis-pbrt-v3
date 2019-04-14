#!/bin/bash
USER=r0588750

#HOSTS=(ham lommel)
HOSTS=(aalst aarlen alken amay andenne ans antwerpen balen bastogne bergen beringen bevekom beveren bierbeek bilzen binche brugge charleroi chimay ciney couvin damme diest dilbeek dinant doornik dour durbuy eeklo eupen genk gent gouvy haacht halle ham hasselt heers heist herstal hoei hove ieper jemeppe knokke kortrijk laarne libin lier lint lommel luik maaseik marche mechelen moeskroen mol musson namen nijvel ninove ohey olen orval peer perwez pittem riemst rixensart ronse schoten seraing spa temse tienen torhout tremelo tubize turnhout verviers veurne vielsalm virton waver yvoir)

#echo ${#HOSTS[@]}

if [ $# -eq 0 ]
then
  echo "No arguments given"
  exit 0
fi

if [ $# -eq 2 ]
then
  echo "Using given hosts"
  HOSTS=( "$2" )
fi

if [ $# -lt 2 ]
then
  read -p "Do you really which to run on all hosts ?" yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) exit 1;;
        * ) exit 1;
    esac
fi

COMMAND=$1
if [ $COMMAND == 'stop' ]
then
  COMMAND="pkill -u "$USER
fi

echo "Running: "$COMMAND" on hosts: "${HOSTS[*]}

for host in ${HOSTS[*]}
do
  echo $host
  ssh $USER"@"$host $COMMAND
  #'echo "test" > "/home/r0588750/"$HOSTNAME".txt"'
done

