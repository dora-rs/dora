# Install the Adora CLI binary from GitHub Releases.
# Usage: powershell -ExecutionPolicy ByPass -c "irm https://github.com/dora-rs/adora/releases/latest/download/adora-cli-installer.ps1 | iex"

$ErrorActionPreference = "Stop"

$repo = "dora-rs/adora"
$dest = "$HOME\.adora\bin"

# Resolve latest tag
$release = Invoke-RestMethod "https://api.github.com/repos/$repo/releases/latest"
$tag = $release.tag_name

$target = "x86_64-pc-windows-msvc"
$archive = "https://github.com/$repo/releases/download/$tag/adora-$target.zip"
Write-Host "Installing adora $tag ($target) to $dest"

# Download and extract
$tmp = Join-Path ([System.IO.Path]::GetTempPath()) ([System.Guid]::NewGuid())
New-Item -ItemType Directory -Path $tmp | Out-Null
$zip = "$tmp\adora.zip"
Invoke-WebRequest -Uri $archive -OutFile $zip

New-Item -ItemType Directory -Path $dest -Force | Out-Null
Expand-Archive -Path $zip -DestinationPath $dest -Force
Remove-Item -Path $tmp -Recurse -Force

# Add to PATH if not already there
$userPath = [System.Environment]::GetEnvironmentVariable("PATH", [System.EnvironmentVariableTarget]::User)
if ($userPath -notlike "*$dest*") {
    [System.Environment]::SetEnvironmentVariable("PATH", "$userPath;$dest", [System.EnvironmentVariableTarget]::User)
    Write-Host "Added $dest to user PATH (restart your terminal to apply)."
} else {
    Write-Host "$dest is already in PATH."
}

Write-Host "Done! Run 'adora --version' to verify."
