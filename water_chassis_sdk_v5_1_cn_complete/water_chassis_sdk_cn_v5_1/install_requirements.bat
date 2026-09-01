@echo off
chcp 65001 >nul
echo 正在安装 WATER SDK Python 依赖...
python -m pip install -r requirements.txt
if errorlevel 1 (
    echo.
    echo 安装失败，请检查 Python / pip 环境。
    pause
    exit /b 1
)
echo.
echo 依赖安装完成。
pause
