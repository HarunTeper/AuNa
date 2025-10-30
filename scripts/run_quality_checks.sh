#!/usr/bin/env bash
set -e
IFS=$'\n\t'

# Combined Quality Check Script
# Runs copyright headers, linting, and tests in sequence
# Usage: ./scripts/run_quality_checks.sh [AUTHOR_NAME] [LICENSE]
#
# Examples:
#   ./scripts/run_quality_checks.sh
#   ./scripts/run_quality_checks.sh "Harun Teper" "mit"

WORKSPACE="${WORKSPACE:-$(pwd)}"
AUTHOR="${1:-Harun Teper}"
LICENSE="${2:-mit}"

echo "=========================================="
echo "  AuNa Quality Checks Pipeline           "
echo "=========================================="
echo "Workspace: $WORKSPACE"
echo "Author: $AUTHOR"
echo "License: $LICENSE"
echo "=========================================="
echo ""

cd "$WORKSPACE"

# Determine docker compose command (use array to handle spaces)
if docker compose version &> /dev/null 2>&1; then
    COMPOSE_CMD=(docker compose)
else
    COMPOSE_CMD=(docker-compose)
fi

#------------------------------------------------------------------------------
# Step 1: Add Copyright Headers
#------------------------------------------------------------------------------
echo "=========================================="
echo "Step 1/3: Adding Copyright Headers"
echo "=========================================="
echo ""

./scripts/add_copyright.sh "$AUTHOR" "$LICENSE"

if [ $? -ne 0 ]; then
    echo "❌ Copyright header addition failed!"
    exit 1
fi

echo ""
echo "✅ Copyright headers added successfully"
echo ""

#------------------------------------------------------------------------------
# Step 2: Run Linting Fixes
#------------------------------------------------------------------------------
echo "=========================================="
echo "Step 2/3: Running Linting Fixes"
echo "=========================================="
echo ""

echo "Building development image if needed..."
"${COMPOSE_CMD[@]}" build development

if [ $? -ne 0 ]; then
    echo "❌ Development image build failed!"
    exit 1
fi

echo ""
echo "Running linting fixes in Docker container..."
"${COMPOSE_CMD[@]}" run --rm development bash -c "/home/ubuntu/workspace/scripts/fix_all_linting.sh"

if [ $? -ne 0 ]; then
    echo "❌ Linting fixes failed!"
    exit 1
fi

echo ""
echo "✅ Linting fixes completed successfully"
echo ""

#------------------------------------------------------------------------------
# Step 3: Run Tests
#------------------------------------------------------------------------------
echo "=========================================="
echo "Step 3/3: Running Build and Tests"
echo "=========================================="
echo ""

echo "Running build and tests via Docker Compose..."
"${COMPOSE_CMD[@]}" --profile test up test

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ Tests failed!"
    echo ""
    echo "To view detailed test results, run:"
    echo "  docker compose run --rm development bash -c 'cd packages && colcon test-result --verbose'"
    exit 1
fi

echo ""
echo "✅ All tests passed successfully"
echo ""

#------------------------------------------------------------------------------
# Summary
#------------------------------------------------------------------------------
echo "=========================================="
echo "  Quality Checks Complete! ✅             "
echo "=========================================="
echo ""
echo "Summary:"
echo "  ✅ Copyright headers added"
echo "  ✅ Linting fixes applied"
echo "  ✅ All tests passed"
echo ""
echo "Next steps:"
echo "  1. Review changes: git diff"
echo "  2. Verify changes: git status"
echo "  3. Commit changes: git add -A && git commit -m 'chore: apply quality checks'"
echo "  4. Push to repository: git push"
echo ""
