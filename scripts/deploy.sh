#!/bin/bash

# Deployment script for AI/Spec-Driven Book with Embedded RAG Chatbot

set -e  # Exit on any error

echo "Starting deployment process..."

# Build the Docusaurus site
echo "Building Docusaurus site..."
npm run build

if [ $? -ne 0 ]; then
    echo "Build failed. Aborting deployment."
    exit 1
fi

echo "Build completed successfully."

# Check if we're in the correct branch
CURRENT_BRANCH=$(git branch --show-current)
if [ "$CURRENT_BRANCH" != "main" ]; then
    echo "Warning: You are not on the main branch. Deployment should typically happen from main."
    read -p "Do you want to continue? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Deployment cancelled."
        exit 0
    fi
fi

# Deploy to GitHub Pages
echo "Deploying to GitHub Pages..."
npx docusaurus deploy

echo "Deployment completed successfully!"
echo "Your site should be available at: https://$(echo $GITHUB_REPOSITORY | cut -d'/' -f1).github.io/$(echo $GITHUB_REPOSITORY | cut -d'/' -f2)/"

# If this is being run locally, provide additional instructions
if [ -z "$GITHUB_ACTIONS" ]; then
    echo ""
    echo "Note: For local deployment to work, you need to:"
    echo "1. Have gh-pages branch set up in your repository"
    echo "2. Have proper GitHub Pages settings configured in your repository settings"
    echo "3. Or use the GitHub Actions workflow for automated deployment"
fi