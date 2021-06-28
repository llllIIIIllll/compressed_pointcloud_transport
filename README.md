
使用ros2时 用rmw_cyclonedds_cpp 效果更好

5分钟单个激光数据大概为100M

启动脚本
```
#!/bin/bash


docker container stop key2 || true
docker container rm key2 || true

xhost +

docker run --rm -it \
  -d \
  --name key2 \
  --privileged=true \
  --network=host \
  --gpus all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=unix$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
  -e GDK_SCALE \
  -e GDK_DPI_SCALE \
  -e LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/opt/opencv/lib/:/opt/libtensorflow-1.14/lib:/usr/local/cuda/lib64:/opt/TensorRT-7.2.2.3/lib/ \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /dev:/dev \
  -v /opt/qomolo:/opt/qomolo \
  -v /etc/localtime:/etc/localtime \
  -v /home/work/driver/:/code/src/ \
  -v /opt/qomolo:/opt/qomolo \
  harbor.qomolo.com/ros2/foxy/foxy-deepstream /bin/bash
  #harbor.qomolo.com/ros2/foxy/foxy-deepstream /usr/bin/supervisord 


```

compress 

```
ros2 run compressed_pointcloud_transport compress 
```


decompress 

```
ros2 run compressed_pointcloud_transport decompress 
```



launch

```
export QOMOLO_ROBOT_ID=igv1

ros2 launch compressed_pointcloud_transport decompress.launch.py 
ros2 launch compressed_pointcloud_transport compress.launch.py 
```