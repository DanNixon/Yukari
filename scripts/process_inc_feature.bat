Capture.exe ^
  --cloudgrabber pcdfile(dir=raw_clouds,pattern=.*_cloud\.pcd,fps=1) ^
  --process featureincremental(out=inc_feature,orenable=1,ormeank=50,orstddev=1.0,downsample=0.005,maxcorrdist=4.0,transepsilon=1e-6,efitepsilon=0.01,normradius=0.08,featureradius=0.05,corrrejectmaxiters=1000,corrrejinlierth=0.05,transform=false) ^
  --loglevel trace