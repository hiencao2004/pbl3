@echo off
:: ===============================
:: Auto Git Commit & Push Script
:: ===============================
cd /d %~dp0

:: Lấy thời gian hiện tại để đặt tên commit
for /f "tokens=1-4 delims=/ " %%a in ('date /t') do (
    set day=%%a
    set month=%%b
    set year=%%c
)
for /f "tokens=1-2 delims=: " %%a in ('time /t') do (
    set hour=%%a
    set min=%%b
)
set commitmsg=Auto commit: %year%-%month%-%day% %hour%h%min%m

echo 🔄 Adding changes...
git add .

echo 📝 Committing changes with message:
echo %commitmsg%
git commit -m "%commitmsg%"

echo 🚀 Pushing to GitHub...
git push

echo ✅ Done!
pause
