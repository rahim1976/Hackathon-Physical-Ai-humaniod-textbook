@echo off
REM Deployment batch script for AI/Spec-Driven Book with Embedded RAG Chatbot

echo Starting deployment process...

REM Build the Docusaurus site
echo Building Docusaurus site...
npm run build

if %errorlevel% neq 0 (
    echo Build failed. Aborting deployment.
    exit /b 1
)

echo Build completed successfully.

REM Deploy to GitHub Pages
echo Deploying to GitHub Pages...
npx docusaurus deploy

if %errorlevel% neq 0 (
    echo Deployment failed.
    exit /b 1
)

echo Deployment completed successfully!
echo Your site should be available at: https://%%GITHUB_REPOSITORY%%.github.io/%%REPOSITORY_NAME%%/

echo Note: For local deployment to work, you need to:
echo 1. Have gh-pages branch set up in your repository
echo 2. Have proper GitHub Pages settings configured in your repository settings
echo 3. Or use the GitHub Actions workflow for automated deployment

pause