@echo off
setlocal enabledelayedexpansion

cd .box2d
git restore CMakeLists.txt
cd ..

set "V1=set_property(TARGET box2d PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$^<^$^<CONFIG:Debug^>:Debug^>")"
echo %V1% >> .\.box2d\CMakeLists.txt

cmake ./.box2d -B box2d_avx2 -DBOX2D_AVX2=ON -DUSE_MSVC_RUNTIME_LIBRARY_DLL=OFF -DBOX2D_SAMPLES=OFF
cmake --build box2d_avx2 --config Release
xcopy box2d_avx2\src\Release\box2d.lib binaries\box2d_windows_amd64_avx2.lib /E /C /H /R /Y
rmdir /s /q box2d_avx2

cmake ./.box2d -B box2d_sse2 -DBOX2D_AVX2=OFF -DUSE_MSVC_RUNTIME_LIBRARY_DLL=OFF -DBOX2D_SAMPLES=OFF
cmake --build box2d_sse2 --config Release
xcopy box2d_sse2\src\Release\box2d.lib binaries\box2d_windows_amd64_sse2.lib /E /C /H /R /Y
rmdir /s /q box2d_sse2

cd .box2d
git restore CMakeLists.txt
cd ..