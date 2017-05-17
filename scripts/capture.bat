Capture.exe ^
  --cloudgrabber openni2 ^
  --imugrabber attitude(port=COM6) ^
  --capturetrigger periodic(delay=5000) ^
  --process savecloud(out=transformed_clouds,transform=true) ^
  --process savecloud(out=raw_clouds,transform=false) ^
  --process saveimu(out=raw_imu)