# CLI Usage

Transform saved point clouds and save:
`Capture.exe --loglevel trace --cloudgrabber pcdfile(dir=./raw_clouds,pattern=.*_cloud\.pcd) --imugrabber file(dir=./imu,pattern=.*_imu\.txt)  --process savecloud(out=transformed_clouds,transform=true)`
