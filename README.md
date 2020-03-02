## people_detect_pi

##### 1. Build and install OpenCV (3.3.1)

reference:[Optimizing OpenCV on the Raspberry Pi - PyImageSearch](https://www.pyimagesearch.com/2017/10/09/optimizing-opencv-on-the-raspberry-pi/)

```bash
# Build tools
sudo apt install build-essential cmake pkg-config
# Image read
sudo apt install libjpeg-dev libtiff5-dev libjasper-dev libpng-dev
# Video read
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt install libxvidcore-dev libx264-dev
# GUI
sudo apt install libgtk2.0-dev libgtk-3-dev
sudo apt install libcanberra-gtk*
# Matrix operation optimization
sudo apt install libatlas-base-dev gfortran
# Python header
sudo apt install python2.7-dev python3-dev pylint pylint3
# Increase the swap partition capacity
sudo nano /etc/dphys-swapfile
# Set CONF_SWAPSIZE to 1024
# Restart swap service
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start
# Build and install Ceres Solver
sudo apt install libeigen3-dev libgflags-dev libgoogle-glog-dev
sudo apt install libsuitesparse-dev
cd ~
# git clone https://ceres-solver.googlesource.com/ceres-solver
git clone https://github.com/ceres-solver/ceres-solver
git checkout <new version>
cd ceres-solver
mkdir build && cd build
cmake ..
make -j4 # several hours
make test # one hour
sudo make install
# Make opencv build dir
cd ~
mkdir opencv-3.3.1
cd opencv-3.3.1
# Clone opencv
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.3.1
cd ..
# Clone opencv_contrib
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.1
cd ..
# Configure cmake compilation parameters:

# BUILD_EXAMPLES
# BUILD_opencv_world BUILD_opencv_contrib_world
# BUILD_opencv_python2 BUILD_opencv_python3
# BUILD_TBB WITH_TBB
# ENABLE_NEON ENABLE_VFPV3
# INSTALL_CREATE_DISTRIB
# INSTALL_C_EXAMPLES
# INSTALL_TESTS
# OPENCV_EXTRA_MODULES_PATH
mkdir build
cd build
cmake \
    -D BUILD_EXAMPLES=ON \
    -D BUILD_opencv_world=ON \
    -D BUILD_opencv_contrib_world=ON \
    -D BUILD_opencv_python2=ON \
    -D BUILD_opencv_python3=ON \
    -D BUILD_TBB=ON \
    -D WITH_TBB=ON \
    -D ENABLE_NEON=ON \
    -D ENABLE_VFPV3=ON \
    -D INSTALL_CREATE_DISTRIB=ON \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_TESTS=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules  \
    ../opencv 

make -j2
sudo make install
# Restore swap partition size
sudo nano /etc/dphys-swapfile
# Set CONF_SWAPSIZE to 100
# Restart swap service
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start
# In order to prevent:
#   Error retrieving accessibility bus address: org.freedesktop.DBus.Error.ServiceUnknown: The name org.a11y.Bus was not provided by any .service files
# We need to install at-spi2-core
sudo apt install at-spi2-core
```
##### 2. Build and install raspicam

```bash
cd ~
git clone https://github.com/cedricve/raspicam
cd raspicam
mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```
##### 3. Build and install matio

```bash
# Install hdf5
sudo apt install libhdf5-dev
cd ~
git clone git://git.code.sf.net/p/matio/matio
cd matio
git submodule update --init  # for datasets used in unit tests
./autogen.sh
# Create a fake directory so matio can recognize hdf5
mkdir fake_hdf5_dir
cd fake_hdf5_dir
ln -s /usr/include/hdf5/serial/ ./include
ln -s /usr/lib/arm-linux-gnueabihf/hdf5/serial/ ./lib
cd ..
./configure --enable-mat73=yes --with-hdf5=/home/pi/matio/fake_hdf5_dir/ --with-default-api-version=v110 --with-default-file-ver=7.3
make -j4
make check  # test
sudo make install
sudo ldconfig
```

##### 4. Use checkinstall to package and copy to Raspberry Pi computing module and install libopencv libmatio libraspicam 

##### 5. Handle dependent dynamic link libraries

```bash
sudo apt install libtiff5 libjasper1 libavcodec57 libavformat57 libswscale4
sudo apt install libhdf5-100
sudo apt install libgtk-3-0
```

[note] this part has been done in the package, if you compile, follow these steps:

##### 6. Build source code

 - Copy the people-detect-pi-src.zip decompressed file to the Raspberry Pi home directory
 - Copy AcfHSMy18Detector.mat to the Raspberry Pi home directory

```bash
cd ~
cd people-detect-pi-src
mkdir people-detect-pi-build
cmake ../
make
```

##### 7. Run

 - Copy the ACF_HS_Detect file under people-detect-pi-src / people-detect-pi-build to the Raspberry Pi home directory
 - Copy human.sh to the Raspberry Pi desktop
 - run human.sh
