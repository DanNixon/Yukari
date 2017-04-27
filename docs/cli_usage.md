# CLI Usage

Capture point clouds from a Kinect and Teensy IMU every 5 seconds and save IMU frames, raw cloud and transformed clouds:
`Capture.exe --cloudgrabber openni2 --imugrabber teensy(port=COM5) --capturetrigger periodic(delay=5000) --process saveimu(out=raw_imu) --process savecloud(out=raw_clouds,transform=false) --process savecloud(out=transformed_clouds,transform=true)`

Transform saved point clouds and save:
`Capture.exe --loglevel trace --cloudgrabber pcdfile(dir=./raw_clouds,pattern=.*_cloud\.pcd) --imugrabber file(dir=./imu,pattern=.*_imu\.txt)  --process savecloud(out=transformed_clouds,transform=true)`

Incremental alignment:
`--cloudgrabber openni2 --imugrabber teensy(port=COM5) --capturetrigger periodic(delay=10000) --process saveimu(out=raw_imu) --process savecloud(out=raw_clouds,transform=false) --process savecloud(out=transformed_clouds,transform=true) --process ndtincremental(out=incremental,cloud=true)`

Test grabbing frames from a Kinect:
`CloudGrabberTest.exe --loglevel trace --grabber openni2`

Test grabbing frames from a Kinect transformed by a Teensy IMU:
`CloudGrabberTest.exe --loglevel trace --grabber openni2 --imugrabber teensy(port=COM5)`

Test grabbing IMU frames from a Teensy IMU:
`IMUGrabberTest.exe --grabber teensy(port=COM5)`
