### Deprecation notice (current repository: [STUSB4500](https://github.com/ardnew/STUSB4500))

This project has been replaced with the Arduino-based library [STUSB4500](https://github.com/ardnew/STUSB4500) and is no longer maintained. It is kept for reference and for compatibility with STM32 HAL-based projects. However, any bug fixes and changes will not be merged into this repository.

This project was a (mostly) successful attempt at developing the device driver compatible with both STM32 and Arduino (via a bunch of `#ifdef` macros) in pure C. However, providing support for both targets created unwarranted complexity (i.e. bugs). So for better general purpose support and compatibility, a conventional Arduino C++ port ([STUSB4500](https://github.com/ardnew/STUSB4500)) was selected as the final, sole target platform. Sorry STM32, still much ❤️ regardless.

----

# libstusb4500
STUSB4500 library for STM32 microcontrollers
