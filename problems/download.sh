#!/bin/bash

cd "${0%/*}"

mkdir -p temp;

if [ "$1" = "fetch" ] ; then
  wget "https://www.dropbox.com/sh/9fqe48lblts24b9/AABq8yYmhHcKpAg1GCBA95_ya?dl=0" -O temp/fetch.zip 
elif [ "$1" = "fetch10k" ] ; then
  wget "https://www.dropbox.com/scl/fo/c86etqmhi01diyoa5wn4y/AE-o-wHV_ywmn2tk7ELVQlo?rlkey=umle77mwebx87she0c582iouy&st=2opcyc9f&dl=0" -O temp/fetch10k.zip
elif [ "$1" = "baxter" ] ; then
  wget "https://www.dropbox.com/sh/4pubt8a7eqwm5t8/AABa5IM6sa3LgrN4q_fs6I5aa?dl=0" -O temp/baxter.zip 
elif [ "$1" = "panda" ] ; then
  wget "https://www.dropbox.com/sh/zut1dy76py19sw5/AAB7OvU3PG9tJ_q9lSR7faOWa?dl=0" -O temp/panda.zip 
elif [ "$1" = "panda10k" ] ; then
  wget "https://www.dropbox.com/scl/fo/xt3miqfactcv25qeuasfu/AJ2JnPJ8IUEUUcU17kolU30?rlkey=z6lz814n0gsaxp6d38wazs5we&st=eowo7y7e&dl=0" -O temp/panda10k.zip
elif [ "$1" = "ur5" ] ; then
  wget "https://www.dropbox.com/sh/y6o4j4c80uga9tu/AABMK_hyRgZJ6k-ijzDwHrEla?dl=0" -O temp/ur5.zip 
elif [ "$1" = "ur510k" ] ; then
  wget "https://www.dropbox.com/scl/fo/gtsyuhcgqk1fmx5lv6kga/AA38g4mrVML97GbPCxt5CrA?rlkey=fz4yywbwabd8z0ncspbclb6fr&st=j3krcnea&dl=0" -O temp/ur510k.zip
elif [ "$1" = "shadow" ] ; then
  wget "https://www.dropbox.com/sh/imt4yxytjaoc876/AACyMB0rRPajjKpjI6XZxqoza?dl=0" -O temp/shadow.zip 
elif [ "$1" = "shadow10k" ] ; then
  wget "https://www.dropbox.com/scl/fo/4ufp52w1yztu76m29w99o/AFlIvoYKmj53pcRUaQFTLvE?rlkey=v196a440vot8rhs1llyi6r79g&st=0wg1cx1o&dl=0" -O temp/shadow10k.zip
elif [ "$1" = "yumi" ] ; then
  wget "https://www.dropbox.com/sh/mys8lz2amy17itt/AADewl_3ckaQOGJ2PsMht0pZa?dl=0" -O temp/yumi.zip 
elif [ "$1" = "all" ] ; then
  wget "https://www.dropbox.com/sh/9fqe48lblts24b9/AABq8yYmhHcKpAg1GCBA95_ya?dl=0" -O temp/fetch.zip 
  wget "https://www.dropbox.com/sh/4pubt8a7eqwm5t8/AABa5IM6sa3LgrN4q_fs6I5aa?dl=0" -O temp/baxter.zip 
  wget "https://www.dropbox.com/sh/zut1dy76py19sw5/AAB7OvU3PG9tJ_q9lSR7faOWa?dl=0" -O temp/panda.zip 
  wget "https://www.dropbox.com/sh/y6o4j4c80uga9tu/AABMK_hyRgZJ6k-ijzDwHrEla?dl=0" -O temp/ur5.zip 
  wget "https://www.dropbox.com/sh/mys8lz2amy17itt/AADewl_3ckaQOGJ2PsMht0pZa?dl=0" -O temp/yumi.zip 
  wget "https://www.dropbox.com/sh/imt4yxytjaoc876/AACyMB0rRPajjKpjI6XZxqoza?dl=0" -O temp/shadow.zip 
elif [ "$1" = "all10k" ] ; then
  wget "https://www.dropbox.com/scl/fo/c86etqmhi01diyoa5wn4y/AE-o-wHV_ywmn2tk7ELVQlo?rlkey=umle77mwebx87she0c582iouy&st=2opcyc9f&dl=0" -O temp/fetch10k.zip
  wget "https://www.dropbox.com/scl/fo/xt3miqfactcv25qeuasfu/AJ2JnPJ8IUEUUcU17kolU30?rlkey=z6lz814n0gsaxp6d38wazs5we&st=eowo7y7e&dl=0" -O temp/panda10k.zip
  wget "https://www.dropbox.com/scl/fo/gtsyuhcgqk1fmx5lv6kga/AA38g4mrVML97GbPCxt5CrA?rlkey=fz4yywbwabd8z0ncspbclb6fr&st=j3krcnea&dl=0" -O temp/ur510k.zip
  wget "https://www.dropbox.com/scl/fo/4ufp52w1yztu76m29w99o/AFlIvoYKmj53pcRUaQFTLvE?rlkey=v196a440vot8rhs1llyi6r79g&st=0wg1cx1o&dl=0" -O temp/shadow10k.zip
else
  echo "The sizes of the datasets are:
  - fetch: 310.0 MB 
  - fetch10k: 17.7 GB 
  - baxter: 218.0 MB
  - panda: 164.0 MB
  - panda10k: 6.9 GB
  - ur5: 178.1 MB 
  - ur510k: 17.9 GB
  - shadow: 177.3 MB
  - shadow10k: 776.MB
  - all (fetch, baxter, panda, ur5, yumi, shadow):a 1.1 GB 
  - all10k (fetch10k, panda10k, ur510k, shadown10k): 43.3GB"

  echo "Unknown robot. Available commands {fetch, fetch10k, baxter, panda, panda10k,\
  ur5, ur510k, yumi, shadow, shadow10k, all, all10k}"
  exit
fi

#Unzip the downloaded files

unzip 'temp/*.zip'
unzip '*.zip'
rm *.zip
rm -rf temp 
