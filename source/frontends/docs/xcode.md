# Apple Xcode support

## Status

Compiles cleanly under Xcode.

## Building

Configure:
Move to the top level directory of AppleWin
Create a `build` directory and configure (here to build the SDL version):
```
mkdir build
cmake -G Xcode -B build -DBUILD_SA2=ON
```
Then open the generated `build/applewin.xcodeproj` file in Xcode, or just type:
```
open build/applewin.xcodeproj
```

## Running

`sa2` should be inside build/Debug or build/Release depending on your choice.
