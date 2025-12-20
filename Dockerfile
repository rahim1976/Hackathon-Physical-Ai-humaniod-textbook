# Multi-stage Dockerfile for AI/Spec-Driven Book with Embedded RAG Chatbot

# Build stage for Python backend
FROM python:3.11-slim as backend

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy backend source code
COPY src/ ./src/
COPY .env.example ./

# Expose port for the backend
EXPOSE 8000

# Backend service command
CMD ["python", "-m", "src.backend.main", "--host", "0.0.0.0", "--port", "8000"]

# Build stage for Docusaurus frontend
FROM node:18-alpine as frontend

# Set working directory
WORKDIR /app

# Copy package files from frontendofbook directory
COPY frontendofbook/package.json ./
COPY frontendofbook/yarn.lock ./

# Install dependencies
RUN npm install

# Copy Docusaurus source from frontendofbook directory
COPY frontendofbook/ .

# Build the Docusaurus site
RUN npm run build

# Production stage - serve the built site
FROM nginx:alpine as production

# Copy built Docusaurus site to nginx
COPY --from=frontend /app/build /usr/share/nginx/html

# Copy nginx configuration
COPY nginx.conf /etc/nginx/nginx.conf

# Expose port
EXPOSE 80

# Start nginx
CMD ["nginx", "-g", "daemon off;"]