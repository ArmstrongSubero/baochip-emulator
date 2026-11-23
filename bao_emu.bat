@echo off
rem bao_emu.bat: build a Dabao SDK program and run it on baoemu
rem
rem Usage:
rem   bao_emu [--sdk PATH] list
rem   bao_emu [--sdk PATH] run  ^<program^> [baoemu options]
rem   bao_emu [--sdk PATH] view ^<program^> [viewer options]
rem   bao_emu [--sdk PATH] test

python "%~dp0bao_emu.py" %*
