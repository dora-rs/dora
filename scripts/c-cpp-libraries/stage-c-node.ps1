# Stage C node library files from OUT_DIR to PREFIX directory
# Usage: stage-c-node.ps1 -BuildDir <BUILD_DIR> -Prefix <PREFIX>
# BUILD_DIR: cargo build output directory (e.g., target\release or target\x86_64-pc-windows-msvc\release)
# PREFIX: staging output directory (e.g., dora-c-libraries-x86_64-pc-windows-msvc)

param(
    [string]$BuildDir = "target/release",
    [string]$Prefix = "dora-c-libraries"
)

$ErrorActionPreference = "Stop"

Write-Host "Staging C node library files..."
Write-Host "  BUILD_DIR: $BuildDir"
Write-Host "  PREFIX:    $Prefix"

# Find OUT_DIR for dora-node-api-c
$OutDirs = Get-ChildItem -Path "$BuildDir/build" -Recurse -Directory -Filter "out" -ErrorAction SilentlyContinue |
    Where-Object { $_.FullName -like "*dora-node-api-c*" }

$OutDir = $OutDirs | Select-Object -First 1

if (-not $OutDir) {
    Write-Error "ERROR: Could not find OUT_DIR for dora-node-api-c in $BuildDir/build"
    exit 1
}

Write-Host "  OUT_DIR:   $OutDir"

# Create PREFIX directory structure
New-Item -ItemType Directory -Force -Path "$Prefix/lib/cmake" | Out-Null
New-Item -ItemType Directory -Force -Path "$Prefix/include" | Out-Null

# Copy cmake config files from OUT_DIR
Copy-Item -Recurse -Force "$OutDir/lib/cmake/dora-node-api-c" "$Prefix/lib/cmake/"

# Copy header from OUT_DIR
Copy-Item -Force "$OutDir/include/node_api.h" "$Prefix/include/"

# Copy library from BUILD_DIR
if (Test-Path "$BuildDir/dora_node_api_c.lib") {
    Copy-Item -Force "$BuildDir/dora_node_api_c.lib" "$Prefix/lib/"
} else {
    Write-Warning "WARNING: Library file not found at $BuildDir/dora_node_api_c.lib"
}

Write-Host "Staged files:"
Get-ChildItem "$Prefix/lib/"
Get-ChildItem "$Prefix/include/"
Get-ChildItem "$Prefix/lib/cmake/dora-node-api-c/"