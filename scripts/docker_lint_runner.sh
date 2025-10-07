#!/bin/bash

# Simple script to run linting fixes using Docker Compose

echo "=========================================="
echo "  Docker Lint Fix Runner                  "
echo "=========================================="
echo ""

# Check if docker-compose is available
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed or not in PATH"
    exit 1
fi

if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Error: Docker Compose is not installed or not in PATH"
    exit 1
fi

# Determine docker compose command
if docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

echo "Building development image with linting tools..."

# Check if user wants to force rebuild
if [ "$1" = "--rebuild" ] || [ "$1" = "--no-cache" ]; then
    echo "Force rebuilding image (this may take a few minutes)..."
    $COMPOSE_CMD build --no-cache development
else
    echo "Using cached image (use --rebuild to force rebuild)..."
    $COMPOSE_CMD build development
fi

echo ""
echo "Running linting fixes..."
$COMPOSE_CMD --profile lint-fix up lint-fix

echo ""
echo "Linting fixes completed!"
echo ""
echo "Next steps:"
echo "1. Review changes: git diff"
echo "2. Test build: docker compose run --rm development bash -c 'cd packages && colcon build'"
echo "3. Enter dev environment: docker compose --profile development up -d development && docker compose exec development bash"