#!/usr/bin/env bash
set -euo pipefail

TAG="v2.4.1"
DOCS_DIR="docs"
THEME_DIR="$DOCS_DIR/doxygen-awesome"
HTML_OUTPUT_DIR="$DOCS_DIR/html"  # Directory where Doxygen generates HTML

# Delete the old docs/html folder before rebuild (if it exists)
if [ -d "$HTML_OUTPUT_DIR" ]; then
    echo "Deleting old docs/html directory..."
    rm -rf "$HTML_OUTPUT_DIR"
fi
# Delete the old docs/html folder before rebuild (if it exists)
if [ -d "$THEME_DIR" ]; then
    echo "Deleting old theme directory"
    rm -rf "$THEME_DIR"
fi

# Clone doxygen-awesome-css directly into docs/ (gitignored)
if [ ! -d "$THEME_DIR" ]; then
    git clone \
      --depth 1 \
      --branch "$TAG" \
      https://github.com/jothepro/doxygen-awesome-css.git \
      "$THEME_DIR"
else
    echo "Theme already exists at $THEME_DIR, skipping clone"
fi

# Update Doxyfile paths to use theme in docs/
sed -i "s|^HTML_EXTRA_STYLESHEET.*$|HTML_EXTRA_STYLESHEET = $THEME_DIR/doxygen-awesome.css|" Doxyfile
sed -i "s|^HTML_HEADER.*$|HTML_HEADER = $THEME_DIR/doxygen-custom/header.html|" Doxyfile

# Add dark mode toggle JS as an extra file
sed -i "/^\s*HTML_EXTRA_FILES/ s|^#*\s*HTML_EXTRA_FILES.*$|HTML_EXTRA_FILES = $THEME_DIR/doxygen-awesome-darkmode-toggle.js|" Doxyfile

# Insert dark mode toggle initialization into header.html
sed -i "/<\/head>/i \
        <script type=\"text/javascript\" src=\"doxygen-awesome-darkmode-toggle.js\"></script>\\
        <script type=\"text/javascript\">\\
            DoxygenAwesomeDarkModeToggle.init();\\
        </script>" "$THEME_DIR/doxygen-custom/header.html"

# Run Doxygen
doxygen Doxyfile