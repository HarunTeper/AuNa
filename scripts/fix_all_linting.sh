#!/usr/bin/env bash
set -e pipefail
IFS=$'\n\t'

# Updated comprehensive lint/fix script for ROS2 repos
# - runs black, isort, ruff, autoflake, docformatter for python
# - runs clang-format (with SortIncludes) and uncrustify for C++
# - attempts to auto-fix cpplint "Add #include <...> for ..." messages
# - fixes header guard style to PKG__FILE_HPP_ for headers under include/<pkg>/
#
# Usage: run from workspace root
#   ./scripts/fix_lint_all.sh

WORKSPACE="${WORKSPACE:-$(pwd)}"
echo "Workspace: $WORKSPACE"
cd "$WORKSPACE"

echo "Sourcing ROS (if present)..."
if [ -f /opt/ros/humble/setup.bash ]; then
  # pick a sensible default, do not fail if not present
  source /opt/ros/humble/setup.bash || true
fi

echo "Installing / updating Python tools (if pip available)..."
if command -v pip3 >/dev/null 2>&1; then
  pip3 install --upgrade black ruff isort autoflake docformatter >/dev/null || true
else
  echo "pip3 not found: skip pip installs (CI should provide these tools)."
fi

# Try apt installs for system tools if available (non-fatal)
install_apt_if_missing() {
  cmd="$1"; pkg="$2"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    if command -v apt-get >/dev/null 2>&1; then
      echo "Installing $pkg via apt-get..."
      sudo apt-get update -qq
      sudo apt-get install -y -qq "$pkg" || true
    fi
  fi
}

install_apt_if_missing uncrustify uncrustify
install_apt_if_missing clang-format clang-format
install_apt_if_missing clang-tidy clang-tidy

echo
echo "1) Python: isort -> black -> ruff -> autoflake -> docformatter"
PY_FILES=$(find packages/src -name "*.py" -type f || true)
if [ -n "$PY_FILES" ]; then
  # isort
  if command -v isort >/dev/null 2>&1; then
    echo "Running isort..."
    isort --profile black --line-length 99 $(echo $PY_FILES) || true
  fi

  # black
  if command -v black >/dev/null 2>&1; then
    echo "Running black --line-length=99..."
    black --line-length 99 $(echo $PY_FILES) || true
  fi

  # ruff fix
  if command -v ruff >/dev/null 2>&1; then
    echo "Running ruff --fix..."
    ruff --fix --no-cache $(echo $PY_FILES) || true
  fi

  # autoflake to remove unused imports/variables
  for f in $PY_FILES; do
    if command -v autoflake >/dev/null 2>&1; then
      autoflake --in-place --remove-all-unused-imports --remove-unused-variables "$f" || true
    fi
  done

  # docformatter to normalize docstrings (wraps, removes blank lines in docstrings)
  if command -v docformatter >/dev/null 2>&1; then
    echo "Running docformatter (wrap summaries/descriptions to 99)..."
    docformatter -i --wrap-summaries 99 --wrap-descriptions 99 $(echo $PY_FILES) || true
  fi

  # targeted fixer: remove one blank line that appears immediately after a function docstring
  echo "Removing stray blank line after function docstrings (pep257 D202) where safe..."
  python3 - <<'PY'
import io,sys,re,os
from pathlib import Path
paths = [p for p in Path('packages/src').rglob('*.py')]
pattern = re.compile(r'^(def .+?:\n)(\s+?"""[\s\S]*?""")(\n)\n', flags=re.M)
for p in paths:
    txt = p.read_text(encoding='utf8')
    new = pattern.sub(lambda m: m.group(1)+m.group(2)+m.group(3), txt)
    if new != txt:
        p.write_text(new, encoding='utf8')
        print("Fixed docstring blank-line in", p)
PY
else
  echo "No Python files found"
fi

echo
echo "2) C++: clang-format (sort includes) -> uncrustify -> header-guard & include heuristics"
CPP_FILES=$(find packages/src -name "*.cpp" -o -name "*.hpp" -o -name "*.cxx" -o -name "*.hh" -type f || true)
if [ -n "$CPP_FILES" ]; then
  # clang-format in-place (will sort includes if .clang-format sets SortIncludes: true)
  if command -v clang-format >/dev/null 2>&1; then
    echo "Running clang-format -i on C/C++ files..."
    for f in $CPP_FILES; do
      clang-format -i --style=file "$f" || true
    done
  else
    echo "clang-format not installed, skipping"
  fi

  # Run uncrustify if available
  # prefer ROS ament config if present
  UNCRUSTIFY_CONFIG="$WORKSPACE/scripts/uncrustify.cfg"
  if [ -f "$UNCRUSTIFY_CONFIG" ] && command -v uncrustify >/dev/null 2>&1; then
    echo "Running uncrustify with config: $UNCRUSTIFY_CONFIG"
    for f in $CPP_FILES; do
      uncrustify -c "$UNCRUSTIFY_CONFIG" --no-backup --replace "$f" || true
    done
  else
    echo "uncrustify config missing or uncrustify not installed; skipping uncrustify"
  fi

  # --- header guard fixer: for files under include/<pkg>/**.hpp/.h/.hh
  echo "Fixing header guards for headers under include/<pkg>/..."
  python3 - <<'PY'
import re,sys
from pathlib import Path
for p in Path('packages/src').rglob('include/*/*'):
    if p.suffix.lower() in ['.h','.hpp','.hh','.hxx']:
        parts = p.parts
        # find index of 'include'
        try:
            idx = parts.index('include')
        except ValueError:
            continue
        rel = parts[idx+1:]  # pkg/..
        if len(rel) >= 2:
            pkg = rel[0]
            rest = rel[1:]
            # construct guard name: PKGNAME__PATH_FILE_HPP_
            fname = '_'.join([s.upper().replace('.', '_') for s in rest])
            guard = f"{pkg.upper()}__{fname}_".replace('__HPP_','_HPP_')  # basic normalization
            text = p.read_text(encoding='utf8')
            # find existing ifndef/define and endif
            new_text = text
            # replace #ifndef ... and #define ... at top if present
            new_text = re.sub(r'(#ifndef\s+)[A-Z0-9_]+', r'\1'+guard, new_text, count=1)
            new_text = re.sub(r'(#define\s+)[A-Z0-9_]+', r'\1'+guard, new_text, count=1)
            # ensure #endif trailing comment
            # This regex now only matches the #endif line itself, without the newline.
            new_text = re.sub(r'#endif\s*(//.*)?$', '#endif  // '+guard, new_text.rstrip()) + '\n'
            if new_text != text.rstrip() + '\n':
                p.write_text(new_text, encoding='utf8')
                print("Header guard fixed:", p)
PY

  # --- run cpplint to capture "Add #include <...> for ..." messages and auto-insert includes
  echo "Running cpplint (ament_cpplint) to collect missing-include hints..."
  if command -v ament_cpplint >/dev/null 2>&1; then
    TMP_CPPLINT=$(mktemp)
    # run with no fail, capture stderr (cpplint prints to stderr)
    ament_cpplint --quiet 2> "$TMP_CPPLINT" || true
    # parse lines like: Add #include <string> for string
    awk '/Add #include/ { print $0 }' "$TMP_CPPLINT" | while read -r line; do
      # extract filename (last token before colon) and include
      # format example from CI logs: /path/to/file.hpp:192:  Add #include <memory> for shared_ptr<>
      file=$(echo "$line" | awk -F: '{print $1}')
      inc=$(echo "$line" | sed -n 's/.*Add #include \(<[^>]*>\).*/\1/p')
      if [ -n "$file" ] && [ -f "$file" ] && [ -n "$inc" ]; then
        echo "Inserting $inc into $file"
        # find the last system include in the top block, otherwise after header guard
        awk -v inc="$inc" 'BEGIN{printed=0} 
          { print $0; if(!printed && /^#include/){ lastline=NR } }
          END{ if(lastline==0) print "#include " inc }' "$file" > "$file.tmp" || true
        # safer: try a python insertion that inserts after the existing #includes block
        python3 - <<PYCODE
from pathlib import Path
p = Path("$file")
txt = p.read_text(encoding='utf8')
lines = txt.splitlines()
# find insertion point: after last #include in first 40 lines, else after header guard define
ins = None
for i,l in enumerate(lines[:200]):
    if l.strip().startswith("#include"):
        ins = i
if ins is not None:
    # insert after last include line
    j = ins
    # move j to last consecutive include
    while j+1 < len(lines) and lines[j+1].strip().startswith("#include"):
        j += 1
    lines.insert(j+1, f'#include {repr("$inc").strip(\"\\'\")}'.replace("'", "").replace('"', ''))
else:
    # find #define guard and insert after it
    for i,l in enumerate(lines[:60]):
        if l.strip().startswith("#define "):
            lines.insert(i+1, f'#include {repr("$inc").strip(\"\\'\")}'.replace("'", "").replace('"', ''))
            break
p.write_text("\n".join(lines)+"\n", encoding='utf8')
print("Inserted", "$inc", "into", "$file")
PYCODE
      fi
    done
    rm -f "$TMP_CPPLINT"
  else
    echo "ament_cpplint not found; cannot auto-insert includes from cpplint hints"
  fi

else
  echo "No C/C++ files found"
fi

echo
echo "3) CMake/CMakeLists: trailing whitespace removal and EOF newline (kept from old script)"
CMAKE_FILES=$(find packages/src -name "CMakeLists.txt" -type f || true)
if [ -n "$CMAKE_FILES" ]; then
  for f in $CMAKE_FILES; do
    sed -i 's/[[:space:]]*$//' "$f" || true
    echo "Fixed trailing whitespace: $f"
  done
fi

echo
echo "4) Ensure files end with newline (kept)"
ALL_SOURCE_FILES=$(find packages/src -type f \( -name "*.py" -o -name "CMakeLists.txt" \) || true)
if [ -n "$ALL_SOURCE_FILES" ]; then
  for f in $ALL_SOURCE_FILES; do
    if [ -s "$f" ] && [ "$(tail -c1 "$f" | wc -l)" -eq 0 ]; then
      echo >> "$f"
      echo "Added newline to: $f"
    fi
  done
fi

echo
echo "5) Final validation: list remaining issues for manual review"
echo "Run these locally or in CI:"
echo " - colcon build"
echo " - colcon test"
echo " - ament_cpplint, ament_pep257 (will show remaining problems)"
echo "Done."
