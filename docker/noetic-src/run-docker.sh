#Run with gpu support

if [ "$1" = "gpu" ] ; then
  docker run --rm -it --gpus all -v /usr/share/glvnd/egl_vendor.d:/usr/share/glvnd/egl_vendor.d motion_bench_maker
elif [ "$1" = "no-gpu" ] ; then
  docker run --rm -it motion_bench_maker
else
  echo "You need to specify if you want gpu support or not. Syntax: ./run-docker.sh {gpu, no-gpu}"
  exit
fi

#Run without gpu support
