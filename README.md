libbldcm (Library to control BLDCM by using mBldcm)
===================================================

Overview
--------
This is a C++ library to control BLDCM by using mBldcm HW IP.

How to build
------------
```sh
$ cmake -S . -B build
$ cmake --build build
```

How to install
--------------
```sh
$ cmake --install build --prefix <Path to install>
```

Requirement
-----------

### Software
* cmake (only at building)
* make (only at building)
* drvfpgasoc
* libfpgasoc

### Hardware
* mBldcm

Limitation
----------
Software using this library must be built with option of C++17 or higher.

License
-------
Please see `LICENSE.md` file for details.

