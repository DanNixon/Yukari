Capture.exe ^
  --cloudgrabber pcdfile(dir=raw_clouds,pattern=.*_cloud\.pcd,fps=1) ^
  --imugrabber file(dir=raw_imu,pattern=.*_imu\.txt) ^
  --process ndticpincremental(out=inc_ndticp,orenable=1,ormeank=50,orstddev=1.0,downsample=0.005,transform=false) ^
  --loglevel trace