@echo off
set SHELLOPTS=igncr
pushd "%~dp0"
bash ../Common/downloadCameraCalibrations %*
popd 
