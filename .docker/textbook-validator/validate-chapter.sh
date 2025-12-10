#!/bin/bash
set -e

CHAPTER_FILE=$1

if [ -z "$CHAPTER_FILE" ]; then
    echo "Usage: validate-chapter.sh <chapter-file.md>"
    exit 1
fi

if [ ! -f "$CHAPTER_FILE" ]; then
    echo "❌ Error: Chapter file not found: $CHAPTER_FILE"
    exit 1
fi

echo "🔍 Validating chapter: $CHAPTER_FILE"
echo "========================================"

# Extract Python code blocks
echo "📝 Extracting Python code blocks..."
grep -Pzo '```python\n\K(.|\n)*?(?=\n```)' "$CHAPTER_FILE" > code.py 2>/dev/null || {
    echo "⚠️  No Python code blocks found in chapter"
    exit 0
}

if [ ! -s code.py ]; then
    echo "⚠️  No Python code blocks found in chapter"
    exit 0
fi

echo "✅ Code extracted successfully"

# Format check (don't modify, just verify)
echo "🎨 Checking PEP 8 formatting with black..."
black --check code.py
if [ $? -ne 0 ]; then
    echo "❌ Code formatting failed PEP 8 (black)"
    echo "   Run: black code.py"
    exit 1
fi
echo "✅ PEP 8 formatting passed"

# Lint check
echo "🔍 Linting code with flake8..."
flake8 code.py
if [ $? -ne 0 ]; then
    echo "❌ Code linting failed (flake8)"
    exit 1
fi
echo "✅ Linting passed"

# Execute code with ROS 2 environment
echo "🚀 Executing code with ROS 2 Humble..."
source /opt/ros/humble/setup.bash
timeout 30s python3 code.py > output.txt 2>&1 || {
    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 124 ]; then
        echo "❌ Code execution timed out (>30 seconds)"
    else
        echo "❌ Code execution failed with exit code: $EXIT_CODE"
    fi
    echo "Output:"
    cat output.txt
    exit $EXIT_CODE
}

echo "✅ Code executed successfully"
echo "Output:"
cat output.txt

echo ""
echo "========================================"
echo "✅ Chapter validation PASSED"
