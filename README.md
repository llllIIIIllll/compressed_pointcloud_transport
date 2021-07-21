
```
better use  rmw_cyclonedds_cpp 

rmw_fastrtps_cpp will cause a decompress error

```

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
