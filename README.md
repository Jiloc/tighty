# Building instruction
## MacOS
#### Prerequisites: 

Installing Homebrew is encouraged, for instruction see [Homebrew Home Page](https://brew.sh/)
* Install Xcode command line tool
* Install pkg-config 

    ```
    brew install pkg-config
    ```
* Install required 3d party libraries library
     ```
    brew install boost flann pcl eigen
    ```
* ~~Install librealsense sdk~~ A realsense build is included in src/3rdParty/librealsense/macos

* Make sure that PKG_CONFIG_PATH variable is set correctly, example: 
    ```
    pkg-config --cflags --libs flann
    ```
    or
    ```
    pkg-config --cflags --libs eigen3
    ```
    (brew package is called eigen but you must refer to it as eigen3)

    should print compiler include and library flags.
#### Configuration: 
* Create a local pkgconfig dir (es in your home):
    ``` 
    mkdir ~/pkgconfig
    ```
* Copy src/3rdParty/pkgconfig/*.pc into your previously create dir
    ```
    cp src/3rdParty/pkgconfig/*.pc ~/pkgconfig/
    ```
* Update realsense2 and realsense2-debug repodir variable and set it where you have cloned the repository
    ```
    repodir=<path_to_tighty_repo>
    ```
* Eventually update the prefix variable in those file, for example if boost is installed in a different location change the prefix variable in boost.pc

* Add src/3rdparty/pkgconfig to PKG_CONFIG_PATH
    ```
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:~/pkgconfig
    ```
    
#### Running:
When you start Qt Creator you must make sure that it's run with the right PKG_CONFIG_PATH variable set otherwise you'll get errors like
package [boost | realsense | etc..] not found. The best way is to set PKG_CONFIG_PATH in .bashrc ( or your shell startup script) and then
open a terminal and invoke Qt Creator.
There is an example script in src/3rdParty/tools, eventually copy it in your path and modify accordinglaly
```
cp src/3rdParty/tools/run_qt_creator ~/bin
```
modify variables in run_qt_creator and
```
chmod +x ~/bin/run_qt_creator
run_qt_creator
```