#!/usr/bin/env bash
set -euo pipefail

OWNER="Ridrik"
REPO="DroneLab-Demo"

DIST_DIR="$(cd "$(dirname "$0")"/.. && pwd)/dist"

mkdir -p "$DIST_DIR"

API="https://api.github.com/repos/$OWNER/$REPO/releases"

echo "Fetching latest release..."

URL=$(
    curl -fsSL "$API" \
        | grep '"browser_download_url"' \
        | grep 'Linux' \
        | grep 'tar\.gz' \
        | head -n1 \
        | cut -d '"' -f4
)

if [ -z "$URL" ]; then
    echo "Could not find Linux SDK."
    exit 1
fi

echo "Downloading:"
echo "$URL"

TMP=$(mktemp)

curl -fL "$URL" -o "$TMP"

tar -xzf "$TMP" -C "$DIST_DIR" --strip-components=1

rm "$TMP"

echo "Installed SDK into:"
echo "$DIST_DIR"