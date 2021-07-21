according to this 
[compressed_pointcloud_transport](https://docs.ros.org/en/electric/api/compressed_pointcloud_transport/html/files.html)


better use  rmw_cyclonedds_cpp 

rmw_fastrtps_cpp will cause a decompress error


compress 

```
ros2 run compressed_pointcloud_transport compress 
```


decompress 

```
ros2 run compressed_pointcloud_transport decompress 
```

