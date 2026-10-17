# Install the Dora CLI binary from GitHub Releases.
# Usage: powershell -ExecutionPolicy ByPass -c "irm https://github.com/dora-rs/dora/releases/latest/download/dora-cli-installer.ps1 | iex"

$ErrorActionPreference = "Stop"

$repo = "dora-rs/dora"
$dest = "$HOME\.dora\bin"

# Resolve latest tag
$release = Invoke-RestMethod "https://api.github.com/repos/$repo/releases/latest"
$tag = $release.tag_name

$target = "x86_64-pc-windows-msvc"
$archive = "https://github.com/$repo/releases/download/$tag/dora-cli-$target.zip"
Write-Host "Installing dora $tag ($target) to $dest"

# Download and extract
$tmp = Join-Path ([System.IO.Path]::GetTempPath()) ([System.Guid]::NewGuid())
New-Item -ItemType Directory -Path $tmp | Out-Null
$zip = "$tmp\dora.zip"
Invoke-WebRequest -Uri $archive -OutFile $zip

New-Item -ItemType Directory -Path $dest -Force | Out-Null
# The zip is flat: the binaries sit at the archive root (unlike the unix
# tarballs, which nest them in a dora-cli-<target>/ directory).
Expand-Archive -Path $zip -DestinationPath $tmp -Force
Move-Item -Path "$tmp\dora.exe" -Destination "$dest\dora.exe" -Force
# `dora-record-node` and `dora-replay-node` are `publish = false`, so the
# archive is where `dora record`/`dora replay` get them.
Move-Item -Path "$tmp\dora-record-node.exe" -Destination "$dest\dora-record-node.exe" -Force
Move-Item -Path "$tmp\dora-replay-node.exe" -Destination "$dest\dora-replay-node.exe" -Force
Remove-Item -Path $tmp -Recurse -Force

# Add to PATH if not already there
$userPath = [System.Environment]::GetEnvironmentVariable("PATH", [System.EnvironmentVariableTarget]::User)
if ($userPath -notlike "*$dest*") {
    [System.Environment]::SetEnvironmentVariable("PATH", "$userPath;$dest", [System.EnvironmentVariableTarget]::User)
    Write-Host "Added $dest to user PATH (restart your terminal to apply)."
} else {
    Write-Host "$dest is already in PATH."
}

Write-Host "Done! Run 'dora --version' to verify."
