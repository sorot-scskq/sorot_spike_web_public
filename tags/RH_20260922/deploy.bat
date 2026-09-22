
@echo off
chcp 65001 >nul
echo Git commit and push to GitHub Pages...

REM 現在のディレクトリを確認
cd /d "%~dp0"

REM Gitの状態を確認
echo Git status check...
git status

REM すべての変更をステージング
echo Adding files...
git add .

REM コミットメッセージを設定（固定メッセージ）
set commit_msg=Update build files

REM コミット実行
echo Committing...
git commit -m "%commit_msg%"

REM プッシュ実行
echo Pushing to GitHub...
git push origin main

echo.
echo Deploy completed!
echo GitHub Pages URL: https://sorot-scskq.github.io/sorot_spike_web_public/
pause