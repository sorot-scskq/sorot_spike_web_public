#!/usr/bin/env bash
# GitHub Pages 向け: 変更をコミットして origin main へ push（deploy.bat のシェル版）
set -euo pipefail

cd "$(dirname "$0")"

echo "Git commit and push to GitHub Pages..."

echo "Git status check..."
git status

echo "Adding files..."
git add .

commit_msg="Update build files"

if git diff --cached --quiet; then
  echo "No changes to commit; skip commit."
else
  echo "Committing..."
  git commit -m "$commit_msg"
fi

echo "Pushing to GitHub..."
git push origin main

echo ""
echo "Deploy completed!"
echo "GitHub Pages URL: https://sorot-scskq.github.io/sorot_spike_web_public/"
