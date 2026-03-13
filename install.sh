#!/usr/bin/env sh
# Install the Adora CLI binary from GitHub Releases.
# Usage: curl -fsSL https://github.com/dora-rs/adora/releases/latest/download/adora-cli-installer.sh | sh

set -eu

REPO="dora-rs/adora"
DEST="$HOME/.adora/bin"

# Resolve latest tag
tag=$(curl -fsSL "https://api.github.com/repos/$REPO/releases/latest" | grep '"tag_name"' | cut -d'"' -f4)

# Detect target triple
case "$(uname -m)-$(uname -s)" in
  aarch64-Linux)  target=aarch64-unknown-linux-gnu ;;
  arm64-Darwin)   target=aarch64-apple-darwin ;;
  x86_64-Darwin)  target=x86_64-apple-darwin ;;
  x86_64-Linux)   target=x86_64-unknown-linux-gnu ;;
  *) echo "Unsupported platform: $(uname -m)-$(uname -s)" >&2; exit 1 ;;
esac

archive="https://github.com/$REPO/releases/download/$tag/adora-$target.tar.gz"
echo "Installing adora $tag ($target) to $DEST"

# Download, extract, install
mkdir -p "$DEST"
curl -fsSL "$archive" | tar -xz -C "$DEST" adora
chmod 755 "$DEST/adora"

# Add to PATH if not already there
add_to_rc() {
  rc="$1"
  if [ -f "$rc" ] && grep -q "$DEST" "$rc"; then
    return
  fi
  echo "export PATH=\"\$PATH:$DEST\"" >> "$rc"
  echo "Added $DEST to PATH in $rc (restart your shell or run: source $rc)"
}

case "${SHELL:-}" in
  */bash) add_to_rc "$HOME/.bashrc" ;;
  */zsh)  add_to_rc "$HOME/.zshrc" ;;
  *)
    echo "Add this to your shell profile:"
    echo "  export PATH=\"\$PATH:$DEST\""
    ;;
esac

echo "Done! Run 'adora --version' to verify."
