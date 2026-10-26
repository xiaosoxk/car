@echo off
chcp 65001 >nul
title 双轮自平衡机器人 - 一键推送到 GitHub
echo ============================================================
echo  一键推送（免 gh，用 Git 自带凭据管理器，浏览器登录一次即可）
echo ============================================================
echo  流程：① 你在网页上建一个空仓库  ② 本脚本自动推送
echo  推送时会弹出浏览器让你登录 GitHub，点 Authorize 就行。
echo.

set GIT="C:\Program Files\Git\cmd\git.exe"
if exist %GIT% goto :have_git
set GIT=git
where git >nul 2>nul
if errorlevel 1 (
    echo [错误] 没找到 git，请确认已安装 Git for Windows。
    pause & exit /b 1
)
:have_git

echo [预检] 测试能否连上 github.com ...
%GIT% ls-remote https://github.com/octocat/Hello-World.git HEAD >nul 2>nul
if errorlevel 1 (
    echo [错误] 当前网络连不上 github.com。
    echo        先关掉别的占带宽的下载，或打开你的代理/VPN，再重跑本脚本。
    pause & exit /b 1
)
echo [预检] 网络通畅。
echo.

set /p GH_USER=请输入你的 GitHub 用户名: 
set REPO=stm32-self-balancing-robot

echo.
echo [第 1 步] 即将打开浏览器新建仓库页面：
echo           仓库名填 %REPO%，选 Public 或 Private 都行，**不要勾选任何初始化选项**
echo           （不要 README / .gitignore / license，否则要先 pull 才能推）
echo           建好后回到这里按任意键继续...
start "" "https://github.com/new?name=%REPO%"
pause

cd /d "%~dp0"
%GIT% remote remove origin >nul 2>nul
%GIT% remote add origin https://github.com/%GH_USER%/%REPO%.git

echo [第 2 步] 推送中（首次会弹浏览器登录 GitHub，点 Authorize）...
%GIT% push -u origin main
if errorlevel 1 (
    echo [错误] 推送失败。常见原因：
    echo        1) 网页上建的仓库名和 %REPO% 不一致
    echo        2) 浏览器授权没完成 —— 重跑本脚本即可
    pause & exit /b 1
)

echo.
echo [完成] 仓库地址: https://github.com/%GH_USER%/%REPO%
start "" "https://github.com/%GH_USER%/%REPO%"
pause
