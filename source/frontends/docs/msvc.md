# Microsoft VisualStudio support

## Why

The original AppleWin project is built with Visual Studio. We now provide support for Visual Studio when building 'sa2', the SDL version of AppleWin.

## Status

`sa2` compiles under VisualStudio 2022.

## Building

Very **important** to install `vcpkg`. Refer to the Microsoft documentation.

Install using `vcpkg` the following packages:
```
vcpkg install sdl2 sdl2-image sdl2-ttf
vcpkg install zlib
vcpkg install boost
```
Then configure:
```
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=C:\PATH_TO_VCPKG\scripts\buildsystems\vcpkg.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DVCPKG_TARGET_TRIPLET=x64-windows -DBUILD_SA2=ON
```
Then open the generated `build\applewin.sln` file in Visual Studio.

## Running

`sa2` should be the startup project that runs when you press F5.
