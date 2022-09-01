# Rtk in GSS
This project contains the open source codes for the paper "Global RTK Positioning in Graphical State Space"
This paper will be presented at ION GNSS+ 2022.
The library was built in Ubuntu 16.04. The results may be different for different OS. If any problem was found by you, please propose an issue or report to
4050627@qq.com. The rtk part was modified from RTKlib to C++ style.

## How to use this project
### 1)Required
- CMake --> sudo apt-get install cmake

- Boost   --> sudo apt-get install libboost-all-dev

- Eigen    --> sudo apt-get install libeigen3-dev

  ​					sudo cp -r /usr/local/include/eigen3 /usr/include

- Intel TBB (optional)->sudo apt-get install libtbb-dev

- [gtsam](https://github.com/borglab/gtsam)  --> git clone https://github.com/borglab/gtsam.git 

  and Install gtsam

- [IntelMkl](https://software.intel.com/content/www/us/en/develop/tools/oneapi/components/onemkl.html)(optional)

### 2）Clone repository to local machine
- RTKinGSS -->git clone https://github.com/shaolinbit/RTKinGSS

### 3)Build
​ cd RTKinGSS
 
 mkdir build && cd build

​ cmake ..

​ make

### 4)Test

 cd ..

 cd bin 

 ./Rtk_gtsam 

 The test data file is in the data file folder.
 
 There will be a test result named "graph_result_dd.txt" in the data/output file folder.
