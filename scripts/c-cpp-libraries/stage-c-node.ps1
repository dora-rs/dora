# Stage C node library files from TARGET_DIR to PREFIX directory
# Usage: stage-c-node.ps1 -TargetDir <TARGET_DIR> -Prefix <PREFIX>
# TARGET_DIR: cargo build output directory (e.g., target\release or target\x86_64-pc-windows-msvc\release)
# PREFIX:     staging output directory (e.g., dora-c-libraries-x86_64-pc-windows-msvc)

param(
    [string]$TargetDir = "target/release",
    [string]$Prefix = "dora-c-libraries"
)

$ErrorActionPreference = "Stop"

Write-Host "Staging C node library files..."
Write-Host "  TARGET_DIR: $TargetDir"
Write-Host "  PREFIX:     $Prefix"

# Create PREFIX directory structure
New-Item -ItemType Directory -Force -Path "$Prefix/lib/cmake" | Out-Null
New-Item -ItemType Directory -Force -Path "$Prefix/include" | Out-Null

# Copy cmake config files from TARGET_DIR
Copy-Item -Recurse -Force "$TargetDir/lib/cmake/dora-node-api-c" "$Prefix/lib/cmake/"

# Copy header from TARGET_DIR
Copy-Item -Force "$TargetDir/include/node_api.h" "$Prefix/include/"

# Copy library from TARGET_DIR
$LibName = "dora_node_api_c.lib"
if (Test-Path "$TargetDir/$LibName") {
    Copy-Item -Force "$TargetDir/$LibName" "$Prefix/lib/"
} else {
    Write-Warning "WARNING: Library file not found at $TargetDir/$LibName"
}

Write-Host "Staged files:"
Get-ChildItem "$Prefix/lib/"
Get-ChildItem "$Prefix/include/"
Get-ChildItem "$Prefix/lib/cmake/dora-node-api-c/"