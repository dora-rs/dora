param (
    [string]$repo = "dora-rs",
    [string]$bin = "dora",
    [string]$tag,
    [string]$target,
    [string]$to = "$HOME\.dora\bin",
    [switch]$force
)

$ErrorActionPreference = "Stop"

if ($env:GITHUB_ACTIONS) {
    $VerbosePreference = "Continue"
    Write-Verbose "Verbose mode enabled because we're running in GitHub Actions"
}

try {
    # Simulate pipefail-like behavior using ErrorAction and traps
    $ErrorActionPreference = "Stop"
    $null = (Get-Command "NonExistentCommand" -ErrorAction Stop | Out-Null)
} catch {
    Write-Verbose "Pipefail-like support enabled"
}

function Show-Help {
    @"
Install a binary released on GitHub

USAGE:
    install.ps1 [options]

FLAGS:
    -h, --help      Display this message
    -f, --force     Force overwriting an existing binary

OPTIONS:
    --repo REPO     Github Repository to install the binary from  [default: dora-rs]
    --bin BIN       Name of the binary to install  [default: dora]
    --tag TAG       Tag (version) of the bin to install, defaults to latest release
    --to LOCATION   Where to install the binary [default: $HOME\.dora\bin]
    --target TARGET
"@
}

function Handle-Error {
    param (
        [string]$Message
    )
    Write-Error "install: $Message"
    exit 1
}

function Need-Command {
    param (
        [string]$command
    )
    if (-not (Get-Command $command -ErrorAction SilentlyContinue)) {
        Handle-Error "$command (command not found)"
    }
}

function New-Temp-Dir() {
  [CmdletBinding(SupportsShouldProcess)]
  param()
  $parent = [System.IO.Path]::GetTempPath()
  [string] $name = [System.Guid]::NewGuid()
  New-Item -ItemType Directory -Path (Join-Path $parent $name)
}

function Download-File {
    param (
        [string]$url,
        [string]$output
    )

    try {
        $outputDir = Split-Path -Path $output -Parent
        if (-not (Test-Path -Path $outputDir)) {
            New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
        }

        Invoke-WebRequest -Uri $url -OutFile $output
    } catch {
        Write-Error "Failed to download the file. Error: $_"
    }
}

Need-Command mkdir
Need-Command Expand-Archive

$url="https://api.github.com/repos/$repo/$bin"
$releases="$url/releases"

$tmp = New-Temp-Dir

if (-not $tag) {
    Download-File "$releases/latest" "$tmp/$bin-version.txt"
    $json = (Get-Content -Path "$tmp/$bin-version.txt")
    $tag = $json | ConvertFrom-Json | Select-Object -ExpandProperty tag_name
}

$target = "x86_64-pc-windows-msvc"
$archive = "https://github.com/$repo/$bin/releases/download/$tag/$bin-$tag-$target.zip"

Write-Host "Repository https://github.com/$repo"
Write-Host "Binary $bin"
Write-Host "Tag $tag"
Write-Host "Target $target"
Write-Host "Destination $to"
Write-Host "Archive $archive"

$zip = "$tmp\$bin-$tag-$target.zip"
Download-File $archive $zip

Write-Host "Placing dora-rs cli in $dest"

Expand-Archive -Path $zip -DestinationPath $to -Force
Remove-Item -Path $tmp -Recurse -Force

function Add-PathToEnv {
    param (
        [string]$newPath
    )

    $currentPath = [System.Environment]::GetEnvironmentVariable("PATH", [System.EnvironmentVariableTarget]::User)

    if ($currentPath -like "*$newPath*") {
        Write-Host "`$newPath` is already in the PATH variable."
    } else {
        $updatedPath = "$currentPath;$newPath"

        [System.Environment]::SetEnvironmentVariable("PATH", $updatedPath, [System.EnvironmentVariableTarget]::User)

        Write-Host "Path added to PATH variable and reloaded."
    }
}

function Confirm-AddToPath {
    param (
        [string]$newPath
    )

    $response = Read-Host "Do you want to add `$newPath` to your PATH automatically? (y/n): "
    if ($response -eq "Y" -or $response -eq "y" -or $response -eq "") {
        Add-PathToEnv -newPath $newPath
    } else {
        Write-Host "You chose not to add `$newPath` to your PATH."
        Write-Host "To run dora CLI without adding to PATH, use: '~/.dora/bin/dora'"
    }
}

Confirm-AddToPath -newPath $to
