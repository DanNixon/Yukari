# CLI Usage

Capture point clouds from a Kinect and Teensy IMU every 5 seconds and save IMU frames, raw cloud and transformed clouds:
`Capture.exe --cloudgrabber openni2 --imugrabber teensy(port=COM5) --capturetrigger periodic(delay=5000) --process saveimu(out=raw_imu) --process savecloud(out=raw_clouds,transform=false) --process savecloud(out=transformed_clouds,transform=true)`

Incremental alignment:
`Capture.exe --cloudgrabber openni2 --imugrabber teensy(port=COM4) --process ndtworld(out=world_aligned) --capturetrigger periodic(delay=10000)`

Transform saved point clouds and save:
`Capture.exe --loglevel trace --cloudgrabber pcdfile(dir=./raw_clouds,pattern=.*_cloud\.pcd) --imugrabber file(dir=./imu,pattern=.*_imu\.txt)  --process savecloud(out=transformed_clouds,transform=true)`

Transform a set of captured clouds by a set of IMU files:
`Capture.exe --cloudgrabber pcdfile(dir=raw_clouds,pattern=.*_cloud\.pcd) --imugrabber file(dir=imu,pattern=.*_imu\.txt) --process savecloud(out=transformed_clouds,transform=true)`

Downsample clouds:
`Capture.exe --cloudgrabber openni2 --capturetrigger periodic(delay=5000) --process savecloud(out=raw_clouds,transform=false) --process downsample(out=downsample,downsample=0.01)`

Test grabbing frames from a Kinect:
`CloudGrabberTest.exe --loglevel trace --grabber openni2`

Test grabbing frames from a Kinect transformed by a Teensy IMU:
`CloudGrabberTest.exe --loglevel trace --grabber openni2 --imugrabber teensy(port=COM5)`

Test grabbing IMU frames from a Teensy IMU:
`IMUGrabberTest.exe --grabber teensy(port=COM5)`

Test PCD file cloud grabber:
`CloudGrabberTest.exe --grabber pcdfile(dir=clouds,pattern=.*_cloud\.pcd,fps=0.1)`

Test IMU file grabber:
`IMUGrabberTest.exe --grabber file(dir=imu,pattern=.*_imu\.txt,delay=1000)`
