
#!/usr/bin/env sh

set -eu

if [ -n "${GITHUB_ACTIONS-}" ]; then
  set -x
fi

# Check pipefail support in a subshell, ignore if unsupported
# shellcheck disable=SC3040
(set -o pipefail 2> /dev/null) && set -o pipefail

help() {
  cat <<'EOF'
Install a binary released on GitHub

USAGE:
    install.sh [options]

FLAGS:
    -h, --help      Display this message
    -f, --force     Force overwriting an existing binary

OPTIONS:
    --repo REPO     Github Repository to install the binary from  [default: dora-rs/dora]
    --bin BIN       Name of the binary to install  [default: dora-cli]
    --tag TAG       Tag (version) of the bin to install, defaults to latest release
    --to LOCATION   Where to install the binary [default: ~/.dora/bin]
    --target TARGET
EOF
}


say() {
  echo "install: $*" >&2
}

err() {
  if [ -n "${td-}" ]; then
    rm -rf "$td"
  fi

  say "error: $*"
  exit 1
}

need() {
  if ! command -v "$1" > /dev/null 2>&1; then
    err "need $1 (command not found)"
  fi
}


download() {
  url="$1"
  output="$2"

  if command -v curl > /dev/null 2>&1; then
    if [ "$output" = "-" ]; then
      curl --proto '=https' --tlsv1.2 -sSfL "$url"
    else
      curl --proto '=https' --tlsv1.2 -sSfL "$url" -o "$output"
    fi
  else
    if [ "$output" = "-" ]; then
      wget --https-only --secure-protocol=TLSv1_2 --quiet "$url" -O -
    else
      wget --https-only --secure-protocol=TLSv1_2 --quiet "$url" -O "$output"
    fi
  fi
}

force=false
while test $# -gt 0; do
  case $1 in
    --help | -h)
      help
      exit 0
      ;;
    --repo)
      repo=$2
      shift
      ;;
    --bin)
      bin=$2
      shift
      ;;
    --tag)
      tag=$2
      shift
      ;;
    --target)
      target=$2
      shift
      ;;
    --to)
      dest=$2
      shift
      ;;
    *)
      say "error: unrecognized argument '$1'. Usage:"
      help
      exit 1
      ;;
  esac
  shift
done

if [ -z "${repo-}" ]; then
  repo="dora-rs/dora"
fi

if [ -z "${bin-}" ]; then
  bin="dora-cli"
fi

url=https://github.com/$repo
releases=$url/releases

command -v curl > /dev/null 2>&1 ||
  command -v wget > /dev/null 2>&1 ||
  err "need wget or curl (command not found)"

need mkdir
need mktemp

if [ -z "${tag-}" ]; then
  need grep
  need cut
fi

if [ -z "${target-}" ]; then
  need cut
fi

if [ -z "${dest-}" ]; then
  dest="$HOME/.dora/bin"
fi


if [ -z "${tag-}" ]; then
  tag=$(
    download https://api.github.com/repos/$repo/releases/latest - |
    grep tag_name |
    cut -d'"' -f4
  )
fi


if [ -z "${target-}" ]; then
  # bash compiled with MINGW (e.g. git-bash, used in github windows runners),
  # unhelpfully includes a version suffix in `uname -s` output, so handle that.
  # e.g. MINGW64_NT-10-0.19044
  kernel=$(uname -s | cut -d- -f1)
  uname_target="$(uname -m)-$kernel"

  case $uname_target in
    aarch64-Linux) target=aarch64-unknown-linux-gnu;;
    arm64-Darwin) target=aarch64-apple-darwin;;
    armv7l-Linux) target=armv7-unknown-linux-musleabihf;;
    x86_64-Darwin) target=x86_64-apple-darwin;;
    x86_64-Linux) target=x86_64-unknown-linux-gnu;;
    *)
      # shellcheck disable=SC2016
      err 'Could not determine target from output of `uname -m`-`uname -s`, please use `--target`:' "$uname_target"
    ;;
  esac
fi

case $target in
  *-windows-*) extension=zip; need unzip;;
  *) extension=tar.gz; need tar;;
esac

archive="$releases/download/$tag/$bin-$target.$extension"
say "Repository:  $url"
say "Bin:         $bin"
say "Tag:         $tag"
say "Target:      $target"
say "Destination: $dest"
say "Archive:     $archive"

# Check for existing dora installations
check_existing_installations() {
  existing=""

  # Find all dora binaries in PATH
  if command -v dora > /dev/null 2>&1; then
    # Get all locations of dora
    dora_locations=$(command -v -a dora 2>/dev/null || which -a dora 2>/dev/null || true)

    for dora_path in $dora_locations; do
      [ -z "$dora_path" ] && continue

      # Skip the destination we're installing to
      case "$dora_path" in
        "$dest"*) continue ;;
      esac

      # Skip nix symlinks
      if [ -L "$dora_path" ]; then
        link_target=$(readlink "$dora_path" 2>/dev/null || true)
        case "$link_target" in
          /nix/*) continue ;;
        esac
      fi

      existing="$existing $dora_path"
    done
  fi

  # Check common installation locations
  for check_path in \
    "$HOME/.cargo/bin/dora" \
    "/usr/local/bin/dora" \
    "/usr/bin/dora" \
    "$HOME/.local/bin/dora"
  do
    [ -e "$check_path" ] || continue

    # Skip the destination we're installing to
    case "$check_path" in
      "$dest"*) continue ;;
    esac

    # Skip if already in list
    case "$existing" in
      *"$check_path"*) continue ;;
    esac

    existing="$existing $check_path"
  done

  # Check for pip-installed dora-rs-cli
  if command -v pip > /dev/null 2>&1; then
    if pip show dora-rs-cli > /dev/null 2>&1; then
      existing="$existing pip:dora-rs-cli"
    fi
  fi

  echo "$existing"
}

existing_installations=$(check_existing_installations)

if [ -n "$existing_installations" ]; then
  echo ""
  echo "WARNING: Found existing dora installation(s):"
  for inst in $existing_installations; do
    if [ "$inst" = "pip:dora-rs-cli" ]; then
      echo "  - dora-rs-cli (installed via pip)"
    else
      version=$("$inst" --version 2>/dev/null | head -1 || echo "unknown version")
      echo "  - $inst ($version)"
    fi
  done
  echo ""
  echo "Having multiple installations may cause conflicts."
  echo "Recommended: Keep only one installation at $dest"
  echo ""

  # Only prompt if running interactively (not in CI)
  if [ -z "${GITHUB_ACTIONS-}" ] && [ -t 0 ]; then
    printf "Do you want to remove existing installations? [y/N] "
    read -r response
    case "$response" in
      [yY]|[yY][eE][sS])
        for inst in $existing_installations; do
          if [ "$inst" = "pip:dora-rs-cli" ]; then
            echo "Removing pip-installed dora-rs-cli..."
            pip uninstall -y dora-rs-cli || echo "Failed to uninstall pip package"
          else
            echo "Removing $inst..."
            rm -f "$inst" 2>/dev/null && echo "Removed $inst" || echo "Failed to remove $inst (may need sudo)"
          fi
        done
        echo ""
        ;;
      *)
        echo "Keeping existing installations. Proceeding with install..."
        echo ""
        ;;
    esac
  else
    echo "Running in non-interactive mode, skipping removal prompt."
    echo "To remove manually, delete the above paths before running this script."
    echo ""
  fi
fi

td=$(mktemp -d || mktemp -d -t tmp)

if [ "$extension" = "zip" ]; then
  download "$archive" "$td/$bin.zip"
  unzip -d "$td" "$td/$bin.zip"
else
  download "$archive" - | tar -C "$td" -xz
fi

echo "Placing dora-rs cli in $dest"

# Binary is named 'dora' inside subdirectory '$bin-$target/'
src_bin="$td/$bin-$target/dora"
dest_bin="$dest/dora"

if [ -e "$dest_bin" ] && [ "$force" = false ]; then
  echo " Replacing \`$dest_bin\` with downloaded version"
  cp "$src_bin" "$dest_bin"
  chmod 755 "$dest_bin"
else
  mkdir -p "$dest"
  cp "$src_bin" "$dest_bin"
  chmod 755 "$dest_bin"
  echo ""
fi

if [ "$SHELL" = "/bin/bash" ]; then
    if ! grep -q "$dest" ~/.bashrc; then
        echo "Adding $dest to PATH in ~/.bashrc"
        echo "export PATH=\$PATH:$dest" >> ~/.bashrc
        echo "Path added to ~/.bashrc."
        echo "Please reload with:"
        echo "  source ~/.bashrc"
    else
        echo "$dest is already in the PATH in ~/.bashrc"
    fi
elif [ "$SHELL" = "/bin/zsh" ]; then
    if ! grep -q "$dest" ~/.zshrc; then
        echo "Adding $dest to PATH in ~/.zshrc"
        echo "export PATH=\$PATH:$dest" >> ~/.zshrc
        echo "Path added to ~/.zshrc."
        echo "Please reload with:"
        echo "  source ~/.zshrc"
    else
        echo "$dest is already in the PATH in ~/.zshrc"
    fi
else
    echo "Unsupported shell: $SHELL"
    echo "Please add the following to your shell's configuration file manually:"
    echo "    export PATH=\$PATH:$dest"
fi

# Check dora-rs pip package version compatibility
check_pip_dora_rs() {
  if ! command -v pip > /dev/null 2>&1; then
    return
  fi

  # Check if dora-rs is installed
  pip_version=$(pip show dora-rs 2>/dev/null | grep "^Version:" | cut -d' ' -f2 || true)
  if [ -z "$pip_version" ]; then
    return
  fi

  # Extract major.minor from installed pip version (e.g., 0.4.0 -> 0.4)
  pip_major_minor=$(echo "$pip_version" | cut -d'.' -f1,2)

  # Extract major.minor from tag (e.g., v0.4.0 -> 0.4)
  # Remove 'v' prefix if present
  tag_version=$(echo "$tag" | sed 's/^v//')
  tag_major_minor=$(echo "$tag_version" | cut -d'.' -f1,2)

  echo ""
  echo "Found dora-rs Python package version $pip_version"

  if [ "$pip_major_minor" = "$tag_major_minor" ]; then
    echo "dora-rs pip package ($pip_version) matches CLI version ($tag_version) - OK"
  else
    echo "WARNING: dora-rs pip package ($pip_version) does not match CLI version ($tag_version)"
    echo "Mismatched versions may cause compatibility issues."
    echo ""

    # Only prompt if running interactively (not in CI)
    if [ -z "${GITHUB_ACTIONS-}" ] && [ -t 0 ]; then
      printf "Do you want to upgrade dora-rs pip package to %s? [Y/n] " "$tag_version"
      read -r response
      case "$response" in
        [nN]|[nN][oO])
          echo "Skipping dora-rs pip package upgrade."
          ;;
        *)
          echo "Upgrading dora-rs pip package to $tag_version..."
          if pip install "dora-rs==$tag_version" 2>/dev/null; then
            echo "dora-rs pip package upgraded to $tag_version"
          else
            echo "Failed to upgrade dora-rs. You may need to run: pip install dora-rs==$tag_version"
          fi
          ;;
      esac
    else
      echo "Running in non-interactive mode."
      echo "To sync versions, run: pip install dora-rs==$tag_version"
    fi
  fi
}

check_pip_dora_rs

# Check Cargo.toml dora-node-api version compatibility (recursive for workspaces)
check_cargo_toml_version() {
  # Check if any Cargo.toml files exist
  if ! find . -name "Cargo.toml" -not -path "*/target/*" -print -quit 2>/dev/null | grep -q .; then
    return
  fi

  # Extract version from tag (remove 'v' prefix if present)
  tag_version=$(echo "$tag" | sed 's/^v//')
  tag_major_minor=$(echo "$tag_version" | cut -d'.' -f1,2)

  mismatched_files=""
  mismatched_list=""
  matched_version=""

  # Use find with -exec to properly handle file paths
  find . -name "Cargo.toml" -not -path "*/target/*" 2>/dev/null | while IFS= read -r cargo_toml; do
    # Extract dora-node-api version from Cargo.toml
    # Matches patterns like: dora-node-api = { version = "0.4.1" or dora-node-api = "0.4.1"
    cargo_version=$(grep -E 'dora-node-api *=' "$cargo_toml" 2>/dev/null | head -1 | sed -n 's/.*version *= *"\([^"]*\)".*/\1/p' || true)
    if [ -z "$cargo_version" ]; then
      # Try simple format: dora-node-api = "0.4.1"
      cargo_version=$(grep -E 'dora-node-api *=' "$cargo_toml" 2>/dev/null | head -1 | sed -n 's/.*= *"\([^"]*\)".*/\1/p' || true)
    fi

    # Check for git-based dependency with tag: dora-node-api = { git = "...", tag = "v0.3.12" }
    cargo_git_tag=$(grep -E 'dora-node-api *=' "$cargo_toml" 2>/dev/null | head -1 | sed -n 's/.*tag *= *"\([^"]*\)".*/\1/p' || true)

    if [ -z "$cargo_version" ] && [ -z "$cargo_git_tag" ]; then
      continue
    fi

    # For git tags, remove 'v' prefix for comparison
    if [ -n "$cargo_git_tag" ]; then
      cargo_version_for_compare=$(echo "$cargo_git_tag" | sed 's/^v//')
      cargo_display="git tag $cargo_git_tag"
    else
      cargo_version_for_compare="$cargo_version"
      cargo_display="$cargo_version"
    fi

    # Extract major.minor from cargo version
    cargo_major_minor=$(echo "$cargo_version_for_compare" | cut -d'.' -f1,2)

    if [ "$cargo_major_minor" != "$tag_major_minor" ]; then
      # Write to temp file to persist across subshell
      echo "$cargo_toml" >> "${td}/mismatched_cargo_files"
      echo "  - $cargo_toml (dora-node-api $cargo_display)" >> "${td}/mismatched_display"
    else
      echo "$cargo_version_for_compare" > "${td}/matched_version"
    fi
  done

  # Read results from temp files
  if [ -f "${td}/matched_version" ]; then
    matched_version=$(cat "${td}/matched_version")
  fi
  if [ -f "${td}/mismatched_display" ]; then
    mismatched_files=$(cat "${td}/mismatched_display")
  fi
  if [ -f "${td}/mismatched_cargo_files" ]; then
    mismatched_list=$(cat "${td}/mismatched_cargo_files" | tr '\n' ' ')
  fi

  if [ -n "$matched_version" ] && [ -z "$mismatched_files" ]; then
    echo ""
    echo "Found Cargo.toml with dora-node-api version $matched_version - matches CLI version ($tag_version) - OK"
  elif [ -n "$mismatched_files" ]; then
    echo ""
    echo "WARNING: Found Cargo.toml file(s) with dora-node-api version that does not match CLI version ($tag_version):"
    echo "$mismatched_files"
    echo ""
    echo "This may cause compatibility issues when building your project."

    # Only prompt if running interactively (not in CI)
    if [ -z "${GITHUB_ACTIONS-}" ] && [ -t 0 ]; then
      printf "Do you want to update these Cargo.toml files to use dora-node-api %s? [y/N] " "$tag_version"
      read -r response
      case "$response" in
        [yY]|[yY][eE][sS])
          for cargo_toml in $mismatched_list; do
            echo "Updating $cargo_toml..."
            # Update git tag format: { git = "...", tag = "vX.Y.Z" }
            if grep -qE 'dora-node-api *= *\{[^}]*tag *=' "$cargo_toml" 2>/dev/null; then
              sed -i.bak -E "s/(dora-node-api *= *\{[^}]*tag *= *\")[^\"]*(\")/\1$tag\2/" "$cargo_toml" && rm -f "$cargo_toml.bak"
            # Update version in table format: { version = "X.Y.Z" }
            elif grep -qE 'dora-node-api *= *\{[^}]*version *=' "$cargo_toml" 2>/dev/null; then
              sed -i.bak -E "s/(dora-node-api *= *\{[^}]*version *= *\")[^\"]*(\")/\1$tag_version\2/" "$cargo_toml" && rm -f "$cargo_toml.bak"
            # Update version in simple format: "X.Y.Z"
            elif grep -qE 'dora-node-api *= *"' "$cargo_toml" 2>/dev/null; then
              sed -i.bak -E "s/(dora-node-api *= *\")[^\"]*(\")/\1$tag_version\2/" "$cargo_toml" && rm -f "$cargo_toml.bak"
            fi
          done
          echo "Cargo.toml files updated to dora-node-api $tag_version"
          ;;
        *)
          echo "Skipping Cargo.toml updates."
          echo "Consider updating manually or install a CLI version that matches your project."
          ;;
      esac
    else
      echo "Running in non-interactive mode."
      echo "Consider updating your Cargo.toml dependencies to version $tag_version"
      echo "or install a CLI version that matches your project."
    fi
  fi
}

check_cargo_toml_version

# Check pyproject.toml dora-rs version compatibility
check_pyproject_toml_version() {
  # Check if any pyproject.toml files exist
  if ! find . -name "pyproject.toml" -not -path "*/.*" -print -quit 2>/dev/null | grep -q .; then
    return
  fi

  # Extract version from tag (remove 'v' prefix if present)
  tag_version=$(echo "$tag" | sed 's/^v//')
  tag_major_minor=$(echo "$tag_version" | cut -d'.' -f1,2)

  # Use find to locate pyproject.toml files
  find . -name "pyproject.toml" -not -path "*/.*" 2>/dev/null | while IFS= read -r pyproject; do
    # Extract dora-rs version from pyproject.toml
    # Matches patterns like: dora-rs==0.4.0, dora-rs>=0.4.0, dora-rs = "^0.4.0", "dora-rs==0.4.0"
    pyproject_version=$(grep -E 'dora-rs' "$pyproject" 2>/dev/null | head -1 | sed -n 's/.*dora-rs[^0-9]*\([0-9][0-9]*\.[0-9][0-9]*\.[0-9][0-9]*\).*/\1/p' || true)

    if [ -z "$pyproject_version" ]; then
      continue
    fi

    # Extract major.minor from pyproject version
    pyproject_major_minor=$(echo "$pyproject_version" | cut -d'.' -f1,2)

    if [ "$pyproject_major_minor" != "$tag_major_minor" ]; then
      echo "$pyproject" >> "${td}/mismatched_pyproject_files"
      echo "  - $pyproject (dora-rs $pyproject_version)" >> "${td}/mismatched_pyproject_display"
    else
      echo "$pyproject_version" > "${td}/matched_pyproject_version"
    fi
  done

  # Read results from temp files
  matched_pyproject_version=""
  mismatched_pyproject_files=""
  mismatched_pyproject_list=""

  if [ -f "${td}/matched_pyproject_version" ]; then
    matched_pyproject_version=$(cat "${td}/matched_pyproject_version")
  fi
  if [ -f "${td}/mismatched_pyproject_display" ]; then
    mismatched_pyproject_files=$(cat "${td}/mismatched_pyproject_display")
  fi
  if [ -f "${td}/mismatched_pyproject_files" ]; then
    mismatched_pyproject_list=$(cat "${td}/mismatched_pyproject_files" | tr '\n' ' ')
  fi

  if [ -n "$matched_pyproject_version" ] && [ -z "$mismatched_pyproject_files" ]; then
    echo ""
    echo "Found pyproject.toml with dora-rs version $matched_pyproject_version - matches CLI version ($tag_version) - OK"
  elif [ -n "$mismatched_pyproject_files" ]; then
    echo ""
    echo "WARNING: Found pyproject.toml file(s) with dora-rs version that does not match CLI version ($tag_version):"
    echo "$mismatched_pyproject_files"
    echo ""
    echo "This may cause compatibility issues when running your project."

    # Only prompt if running interactively (not in CI)
    if [ -z "${GITHUB_ACTIONS-}" ] && [ -t 0 ]; then
      printf "Do you want to update these pyproject.toml files to use dora-rs %s? [y/N] " "$tag_version"
      read -r response
      case "$response" in
        [yY]|[yY][eE][sS])
          for pyproject in $mismatched_pyproject_list; do
            echo "Updating $pyproject..."
            # Update version patterns like dora-rs==X.Y.Z or dora-rs>=X.Y.Z
            sed -i.bak -E "s/(dora-rs[><=]*)[0-9]+\.[0-9]+\.[0-9]+/\1$tag_version/g" "$pyproject" && rm -f "$pyproject.bak"
          done
          echo "pyproject.toml files updated to dora-rs $tag_version"
          ;;
        *)
          echo "Skipping pyproject.toml updates."
          echo "Consider updating manually or install a CLI version that matches your project."
          ;;
      esac
    else
      echo "Running in non-interactive mode."
      echo "Consider updating your pyproject.toml dependencies to version $tag_version"
      echo "or install a CLI version that matches your project."
    fi
  fi
}

check_pyproject_toml_version

# Check requirements.txt dora-rs version compatibility
check_requirements_txt_version() {
  # Check if any requirements.txt files exist
  if ! find . -name "requirements*.txt" -not -path "*/.*" -print -quit 2>/dev/null | grep -q .; then
    return
  fi

  # Extract version from tag (remove 'v' prefix if present)
  tag_version=$(echo "$tag" | sed 's/^v//')
  tag_major_minor=$(echo "$tag_version" | cut -d'.' -f1,2)

  # Use find to locate requirements.txt files (including requirements-dev.txt, etc.)
  find . -name "requirements*.txt" -not -path "*/.*" 2>/dev/null | while IFS= read -r reqfile; do
    # Extract dora-rs version from requirements.txt
    # Matches patterns like: dora-rs==0.4.0, dora-rs>=0.4.0, dora-rs~=0.4.0
    req_version=$(grep -E '^dora-rs' "$reqfile" 2>/dev/null | head -1 | sed -n 's/.*dora-rs[^0-9]*\([0-9][0-9]*\.[0-9][0-9]*\.[0-9][0-9]*\).*/\1/p' || true)

    if [ -z "$req_version" ]; then
      continue
    fi

    # Extract major.minor from requirements version
    req_major_minor=$(echo "$req_version" | cut -d'.' -f1,2)

    if [ "$req_major_minor" != "$tag_major_minor" ]; then
      echo "$reqfile" >> "${td}/mismatched_req_files"
      echo "  - $reqfile (dora-rs $req_version)" >> "${td}/mismatched_req_display"
    else
      echo "$req_version" > "${td}/matched_req_version"
    fi
  done

  # Read results from temp files
  matched_req_version=""
  mismatched_req_files=""
  mismatched_req_list=""

  if [ -f "${td}/matched_req_version" ]; then
    matched_req_version=$(cat "${td}/matched_req_version")
  fi
  if [ -f "${td}/mismatched_req_display" ]; then
    mismatched_req_files=$(cat "${td}/mismatched_req_display")
  fi
  if [ -f "${td}/mismatched_req_files" ]; then
    mismatched_req_list=$(cat "${td}/mismatched_req_files" | tr '\n' ' ')
  fi

  if [ -n "$matched_req_version" ] && [ -z "$mismatched_req_files" ]; then
    echo ""
    echo "Found requirements.txt with dora-rs version $matched_req_version - matches CLI version ($tag_version) - OK"
  elif [ -n "$mismatched_req_files" ]; then
    echo ""
    echo "WARNING: Found requirements.txt file(s) with dora-rs version that does not match CLI version ($tag_version):"
    echo "$mismatched_req_files"
    echo ""
    echo "This may cause compatibility issues when running your project."

    # Only prompt if running interactively (not in CI)
    if [ -z "${GITHUB_ACTIONS-}" ] && [ -t 0 ]; then
      printf "Do you want to update these requirements.txt files to use dora-rs %s? [y/N] " "$tag_version"
      read -r response
      case "$response" in
        [yY]|[yY][eE][sS])
          for reqfile in $mismatched_req_list; do
            echo "Updating $reqfile..."
            # Update version patterns like dora-rs==X.Y.Z or dora-rs>=X.Y.Z
            sed -i.bak -E "s/^(dora-rs[><=~]*)[0-9]+\.[0-9]+\.[0-9]+/\1$tag_version/" "$reqfile" && rm -f "$reqfile.bak"
          done
          echo "requirements.txt files updated to dora-rs $tag_version"
          ;;
        *)
          echo "Skipping requirements.txt updates."
          echo "Consider updating manually or install a CLI version that matches your project."
          ;;
      esac
    else
      echo "Running in non-interactive mode."
      echo "Consider updating your requirements.txt dependencies to version $tag_version"
      echo "or install a CLI version that matches your project."
    fi
  fi
}

check_requirements_txt_version

rm -rf "$td"
