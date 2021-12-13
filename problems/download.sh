#!/bin/bash

cd "${0%/*}"

mkdir -p temp;

if [ "$1" = "fetch" ] ; then
  wget https://www.dropbox.com/sh/9fqe48lblts24b9/AABq8yYmhHcKpAg1GCBA95_ya?dl=0 -O temp/fetch.zip 
elif [ "$1" = "baxter" ] ; then
  wget https://www.dropbox.com/sh/4pubt8a7eqwm5t8/AABa5IM6sa3LgrN4q_fs6I5aa?dl=0 -O temp/baxter.zip 
elif [ "$1" = "panda" ] ; then
  wget https://www.dropbox.com/sh/zut1dy76py19sw5/AAB7OvU3PG9tJ_q9lSR7faOWa?dl=0 -O temp/panda.zip 
elif [ "$1" = "ur5" ] ; then
  wget https://www.dropbox.com/sh/y6o4j4c80uga9tu/AABMK_hyRgZJ6k-ijzDwHrEla?dl=0 -O temp/ur5.zip 
elif [ "$1" = "shadow" ] ; then
  wget https://www.dropbox.com/sh/imt4yxytjaoc876/AACyMB0rRPajjKpjI6XZxqoza?dl=0 -O temp/shadow.zip 
elif [ "$1" = "yumi" ] ; then
  wget https://www.dropbox.com/sh/mys8lz2amy17itt/AADewl_3ckaQOGJ2PsMht0pZa?dl=0 -O temp/yumi.zip 
elif [ "$1" = "all" ] ; then
  wget https://www.dropbox.com/sh/9fqe48lblts24b9/AABq8yYmhHcKpAg1GCBA95_ya?dl=0 -O temp/fetch.zip 
  wget https://www.dropbox.com/sh/4pubt8a7eqwm5t8/AABa5IM6sa3LgrN4q_fs6I5aa?dl=0 -O temp/baxter.zip 
  wget https://www.dropbox.com/sh/zut1dy76py19sw5/AAB7OvU3PG9tJ_q9lSR7faOWa?dl=0 -O temp/panda.zip 
  wget https://www.dropbox.com/sh/y6o4j4c80uga9tu/AABMK_hyRgZJ6k-ijzDwHrEla?dl=0 -O temp/ur5.zip 
  wget https://www.dropbox.com/sh/mys8lz2amy17itt/AADewl_3ckaQOGJ2PsMht0pZa?dl=0 -O temp/yumi.zip 
  wget https://www.dropbox.com/sh/imt4yxytjaoc876/AACyMB0rRPajjKpjI6XZxqoza?dl=0 -O temp/shadow.zip 
else
  echo "Unknown robot. Available commands {fetch, baxter, panda, ur5, yumi, shadow, all}"
  exit
fi


#Unzip the downloaded files

unzip 'temp/*.zip'
unzip '*.zip'
rm *.zip
rm -rf temp 
