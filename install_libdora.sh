#!/usr/bin/env bash
set -e

PREFIX="/usr/local"

detect_platform() {
  local os="$(uname -s)"
  local arch="$(uname -m)"

  case "$os" in
    Darwin)
      # macOS
      if [ "$arch" = "x86_64" ]; then
        # macOS Intel
        echo "x86_64-apple-darwin"
      elif [ "$arch" = "arm64" ] || [ "$arch" = "aarch64" ]; then
        # macOS Apple Silicon
        echo "aarch64-apple-darwin"
      else
        echo "Unsupported architecture on macOS: $arch" >&2
        exit 1
      fi
      ;;
    Linux)
      # Linux
      case "$arch" in
        x86_64)
          echo "x86_64-unknown-linux-gnu"
          ;;
        i686)
          echo "i686-unknown-linux-gnu"
          ;;
        armv7l)
          echo "armv7-unknown-linux-musleabihf"
          ;;
        aarch64|arm64)
          echo "aarch64-unknown-linux-gnu"
          ;;
        *)
          echo "Unsupported architecture on Linux: $arch" >&2
          exit 1
          ;;
      esac
      ;;
    *)
      echo "Unsupported OS: $os" >&2
      exit 1
      ;;
  esac
}

if ! command -v curl >/dev/null 2>&1; then
  echo "Error: 'curl' not found!" >&2
  exit 1
fi

if ! command -v jq >/dev/null 2>&1; then
  echo "Error: 'jq' not found! Please install it (e.g. 'brew install jq' or 'apt install jq')" >&2
  exit 1
fi

echo "==> Determining local platform..."
PLATFORM="$(detect_platform)"
echo "    => PLATFORM: $PLATFORM"

echo "==> Fetching latest Dora release info from GitHub..."
LATEST_JSON=$(curl -s https://api.github.com/repos/dora-rs/dora/releases/latest)

if [ -z "$LATEST_JSON" ]; then
  echo "Error: failed to fetch release info from GitHub!"
  exit 1
fi

TAG=$(echo "$LATEST_JSON" | jq -r '.tag_name')
if [ -z "$TAG" ] || [ "$TAG" = "null" ]; then
  echo "Error: 'tag_name' not found in GitHub API JSON!"
  echo "$LATEST_JSON"
  exit 1
fi

VERSION="${TAG#v}"
echo "==> Latest release: $TAG => version: $VERSION"

echo "==> Searching for matching asset in release assets..."

ASSET_INFO=$(echo "$LATEST_JSON" | jq -r \
  --arg platform "$PLATFORM" \
  '.assets[] | select(.name | contains($platform)) | {name: .name, url: .browser_download_url} | @base64')

ASSET_COUNT=0
CHOSEN_ASSET_NAME=""
CHOSEN_ASSET_URL=""
for item in $ASSET_INFO; do
  dec=$(echo "$item" | base64 --decode)
  asset_name=$(echo "$dec" | jq -r '.name')
  asset_url=$(echo "$dec" | jq -r '.url')
  ASSET_COUNT=$((ASSET_COUNT + 1))

  CHOSEN_ASSET_NAME="$asset_name"
  CHOSEN_ASSET_URL="$asset_url"
done

if [ "$ASSET_COUNT" -eq 0 ]; then
  echo "Error: no asset found for platform '$PLATFORM' in release $TAG"
  exit 1
elif [ "$ASSET_COUNT" -gt 1 ]; then
  echo "Warning: multiple assets matched. Will use the last one found:"
  echo "    name: $CHOSEN_ASSET_NAME"
fi

echo "==> Downloading asset: $CHOSEN_ASSET_NAME"
echo "    from URL: $CHOSEN_ASSET_URL"

curl -L -o "$CHOSEN_ASSET_NAME" "$CHOSEN_ASSET_URL"

if [ ! -f "$CHOSEN_ASSET_NAME" ]; then
  echo "Error: download failed or no file found: $CHOSEN_ASSET_NAME"
  exit 1
fi

echo "==> Extracting $CHOSEN_ASSET_NAME ..."
tar -zxvf "$CHOSEN_ASSET_NAME"

FILE_TO_FIX="include/operator.h"

if [ -f "$FILE_TO_FIX" ]; then
  echo "==> Fixing include paths in $FILE_TO_FIX ..."
  if sed --version 2>/dev/null | grep -q "GNU"; then
    sed -i 's|../operator-rust-api/operator.h|operator.h|g' "$FILE_TO_FIX"
    sed -i 's|#include "../../../apis/c/operator/operator_api.h"|#include "operator_api.h"|' "$FILE_TO_FIX"
    sed -i 's|#include "../build/dora-operator-api.h"|#include "dora-operator-api.h"|' "$FILE_TO_FIX"
  else
    sed -i '' 's|../operator-rust-api/operator.h|operator.h|g' "$FILE_TO_FIX"
    sed -i '' 's|#include "../../../apis/c/operator/operator_api.h"|#include "operator_api.h"|' "$FILE_TO_FIX"
    sed -i '' 's|#include "../build/dora-operator-api.h"|#include "dora-operator-api.h"|' "$FILE_TO_FIX"
  fi
fi

echo "==> Installing headers to: $PREFIX/include/dora"
mkdir -p "$PREFIX/include/dora"
cp -r include/* "$PREFIX/include/dora"

echo "==> Installing libraries to: $PREFIX/lib"
mkdir -p "$PREFIX/lib"
cp -r lib/* "$PREFIX/lib"

echo "==> Cleaning up..."
rm -rf include lib
rm -f "$CHOSEN_ASSET_NAME"

echo
echo "Installation complete!"
echo "Dora version: $VERSION"
echo "Headers installed at: $PREFIX/include/dora"
echo "Libraries installed at: $PREFIX/lib"
echo "All temp files removed."
