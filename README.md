# 3D Image Object Labeller

This c++ program provide a interface for the user to label multiple objects on 3D point cloud.

## Prerequisite
Please install the following dependencies: 

1.  qt   >= 5.x
2.  vtk  >= 6.2
3.  PCL  >= 1.8

I have tested the following installation procedure on ubuntu 16.04.3 & 18.04.1 with the following dependencies version:

|Library Name   | 16.04.3 Version        | 18.04.1 Version      |
|---------------|------------------------|----------------------|
|libboost-dev   | 1.58.0.1ubuntu1        | 1.65.1.0ubuntu1      |
|libeigen3-dev  | 3.3~beta1-2            | 3.3.4-4              |
|libflann-dev   | 1.8.4-4.1              | 1.9.1+dfsg-2         |
|libvtk6-dev    | 6.2.0+dfsg1-10build1   | 6.3.0+dfsg1-11build1 |
|libvtk6-qt-dev | 6.2.0+dfsg1-10build1   | 6.3.0+dfsg1-11build1 |
|libqhull-dev   | 2015.2-1               | 2015.2-4             |
|qtbase5-dev    | 5.5.1+dfsg-16ubuntu7.5 | 5.9.5+dfsg-0ubuntu1  |

#### please use apt to install the dependencies! 

## Installation
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

## License
MIT License

Copyright (c) 2018 Lester Lo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
