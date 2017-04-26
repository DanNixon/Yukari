# Documentation

Transform saved point clouds and save:
`--loglevel trace --cloudgrabber pcdfile(dir=./capture_1/raw_clouds,pattern=.*_cloud\.pcd) --imugrabber file(dir=./capture_1/imu2,pattern=.*_imu\.txt)  --process savecloud(out=transformed_clouds,transform=true)`
