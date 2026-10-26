@echo off
chcp 65001 >nul
title 双轮自平衡机器人 - 一键推送到 GitHub
echo ============================================================
echo  一键推送脚本（本目录已 git init 并完成首次提交）
echo ============================================================
echo  第一次使用需要登录 GitHub，跟着提示走即可：
echo    1. 会弹浏览器打开 github.com/login/device
echo    2. 输入终端里显示的 8 位配对码
echo    3. 点 Authorize 授权，脚本会自动完成建仓和推送
echo.
pause
cd /d "%~dp0"

where gh >nul 2>nul
if errorlevel 1 (
    echo [错误] 没找到 gh 命令，请先安装 GitHub CLI：
    echo        winget install GitHub.cli
    echo        装完重开本脚本。
    pause
    exit /b 1
)

echo [1/3] 检查登录状态...
gh auth status >nul 2>nul
if errorlevel 1 (
    echo 需要登录，按提示在浏览器完成配对：
    gh auth login --hostname github.com --git-protocol https --web
    if errorlevel 1 (
        echo [错误] 登录失败，重跑本脚本再试。
        pause
        exit /b 1
    )
)

echo [2/3] 创建仓库并推送（默认公开仓，名字 stm32-self-balancing-robot）...
gh repo create stm32-self-balancing-robot --public --source . --remote origin --push

echo [3/3] 完成！仓库地址：
gh repo view --web 2>nul
echo.
echo 如果想把仓库改为私有：
echo    gh repo edit --visibility private --accept-visibility-change-consequences
pause
