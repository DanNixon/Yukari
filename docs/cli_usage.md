# CLI Usage

Capture point clouds from a Kinect and Teensy IMU every 5 seconds and save IMU frames, raw cloud and transformed clouds:
```
Capture.exe
  --cloudgrabber openni2
  --imugrabber teensy(port=COM5)
  --capturetrigger periodic(delay=5000)
  --process saveimu(out=raw_imu)
  --process savecloud(out=raw_clouds,transform=false)
  --process savecloud(out=transformed_clouds,transform=true)
```

A similar thing with the ARM IMU:
```
Capture.exe
  --cloudgrabber openni2
  --imugrabber stm32(port=COM5,position=false)
  --process ndtworld(out=world_aligned) 
  --process savecloud(out=transformed_clouds,transform=true)
  --capturetrigger periodic(delay=5000)
```

Compare all world alignment methods:
```
Capture.exe
  --cloudgrabber openni2
  --imugrabber stm32(port=COM4,position=false)
  --process pairalign(out=world_aligned_pair,downsample=0.01)
  --process icpworld(out=world_aligned_icp)
  --process ndtworld(out=world_aligned_ndt)
  --process savecloud(out=transformed_clouds,transform=true)
  --capturetrigger periodic(delay=5000)
```

Incremental alignment:
```
Capture.exe
  --cloudgrabber openni2
  --imugrabber teensy(port=COM4)
  --process ndtworld(out=world_aligned)
  --capturetrigger periodic(delay=10000)
```

Transform saved point clouds and save:
```
Capture.exe
  --loglevel trace
  --cloudgrabber pcdfile(dir=./raw_clouds,pattern=.*_cloud\.pcd)
  --imugrabber file(dir=./imu,pattern=.*_imu\.txt)
  --process savecloud(out=transformed_clouds,transform=true)
```

Transform a set of captured clouds by a set of IMU files:
```
Capture.exe
  --cloudgrabber pcdfile(dir=raw_clouds,pattern=.*_cloud\.pcd)
  --imugrabber file(dir=imu,pattern=.*_imu\.txt)
  --process savecloud(out=transformed_clouds,transform=true)
```

Downsample clouds:
```
Capture.exe
  --cloudgrabber openni2
  --capturetrigger periodic(delay=5000)
  --process savecloud(out=raw_clouds,transform=false)
  --process downsample(out=downsample,downsample=0.01)
```

Feature based registration:
```
Capture.exe
  --cloudgrabber pcdfile(pattern=.*_cloud\.pcd,fps=1)
  --process featureincremental(out=inc,orenable=1,ormeank=50,orstddev=1.0,downsample=0.01,maxcorrdist=0.5,transepsilon=1e-6,efitepsilon=0.01,normradius=0.03,featureradius=0.05,corrrejectmaxiters=1000,corrrejinlierth=0.02,transform=false)
  --loglevel trace
```

Operate on a set of saved point clouds:
```
Capture.exe
  --cloudgrabber pcdfile(pattern=.*_cloud\.pcd,fps=1)
  --process savecloud(out=raw_clouds,transform=false)
```

Test grabbing frames from a Kinect:
```
CloudGrabberTest.exe
  --loglevel trace
  --grabber openni2
```

Test grabbing frames from a Kinect transformed by a Teensy IMU:
```
CloudGrabberTest.exe
  --loglevel trace
  --grabber openni2
  --imugrabber teensy(port=COM5)
```

Test grabbing IMU frames from a Teensy IMU:
```
IMUGrabberTest.exe
  --grabber teensy(port=COM5)
```

Test PCD file cloud grabber:
```
CloudGrabberTest.exe
  --grabber pcdfile(dir=clouds,pattern=.*_cloud\.pcd,fps=0.1)
```

Test IMU file grabber:
```
IMUGrabberTest.exe
  --grabber file(dir=imu,pattern=.*_imu\.txt,delay=1000)
```
