$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$dataDir = Join-Path $root "data\tuning"
New-Item -ItemType Directory -Force -Path $dataDir | Out-Null

$datasets = @(
    @{
        Name = "email-Eu-core"
        Url = "https://snap.stanford.edu/data/email-Eu-core.txt.gz"
        Gz = "email-Eu-core.txt.gz"
        Txt = "email-Eu-core.txt"
    },
    @{
        Name = "p2p-Gnutella08"
        Url = "https://snap.stanford.edu/data/p2p-Gnutella08.txt.gz"
        Gz = "p2p-Gnutella08.txt.gz"
        Txt = "p2p-Gnutella08.txt"
    }
)

foreach ($dataset in $datasets) {
    $gzPath = Join-Path $dataDir $dataset.Gz
    $txtPath = Join-Path $dataDir $dataset.Txt

    if (-not (Test-Path $gzPath) -and -not (Test-Path $txtPath)) {
        Write-Host "Downloading $($dataset.Name) from $($dataset.Url)"
        Invoke-WebRequest -Uri $dataset.Url -OutFile $gzPath
    }

    if (-not (Test-Path $txtPath)) {
        Write-Host "Expanding $($dataset.Gz)"
        $inputStream = [System.IO.File]::OpenRead($gzPath)
        try {
            $gzipStream = [System.IO.Compression.GZipStream]::new(
                $inputStream,
                [System.IO.Compression.CompressionMode]::Decompress
            )
            try {
                $outputStream = [System.IO.File]::Create($txtPath)
                try {
                    $gzipStream.CopyTo($outputStream)
                } finally {
                    $outputStream.Dispose()
                }
            } finally {
                $gzipStream.Dispose()
            }
        } finally {
            $inputStream.Dispose()
        }
    }
}

Write-Host "Datasets ready in $dataDir"
