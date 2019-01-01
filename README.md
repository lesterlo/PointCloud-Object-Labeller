# 3D Image Object Labeller

This c++ program provide an interface for the user to label multiple objects on 3D point cloud.

## Prerequisite
Please install the following dependencies: 

1.  qt   >= 5.x
2.  vtk  >= 6.2
3.  PCL  >= 1.8

I have tested this program on the following OS:

| OS Version |
|---|
|Linux ubuntu 16.04.3|
|Linux ubuntu 18.04.1|
|MacOS 10.14.1|

Please visit the following website to compile a compatible version of PCL.

[PCL compilation Guide](https://wp.me/p7E4PF-m)


## Compilation
```
mkdir build
cd build/
cmake ..
make -j4
```

## Run
```
./PointCloud_Object_labeller
```
## Installation
The installation procedure is optional
```
make install 
```
