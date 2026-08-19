param(
    [Parameter(Mandatory = $true)]
    [string]$Size,
    [Parameter(Mandatory = $true)]
    [string]$InputPath,
    [Parameter(Mandatory = $true)]
    [string]$OutputDir,
    [string]$ConfigPath = "configs\gurobi_benchmark_support75.json",
    [string]$ReportPath = "output\gurobi_size_benchmark.csv"
)

$pythonPath = Join-Path (Get-Location) ".venv\Scripts\python.exe"
$logPath = "$OutputDir.log"
$outputPath = Join-Path (Get-Location) $OutputDir
$report = Join-Path (Get-Location) $ReportPath
$reportParent = Split-Path -Parent $report
New-Item -ItemType Directory -Force -Path $reportParent | Out-Null
New-Item -ItemType Directory -Force -Path $outputPath | Out-Null

$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
$process = Start-Process -FilePath $pythonPath -WorkingDirectory (Get-Location) -PassThru -RedirectStandardOutput $logPath -RedirectStandardError "$OutputDir.err.log" -ArgumentList @(
    "gurobi_pallet_solver.py",
    "--input", $InputPath,
    "--config", $ConfigPath,
    "--output-dir", $OutputDir
)
$sampledCpuSeconds = 0.0
while (-not $process.HasExited) {
    $process.Refresh()
    $sampledCpuSeconds = [math]::Max($sampledCpuSeconds, $process.TotalProcessorTime.TotalSeconds)
    Start-Sleep -Milliseconds 500
}
$process.WaitForExit()
$stopwatch.Stop()
$process.Refresh()
$sampledCpuSeconds = [math]::Max($sampledCpuSeconds, $process.TotalProcessorTime.TotalSeconds)
$cpuSeconds = [math]::Round($sampledCpuSeconds, 3)
$wallSeconds = [math]::Round($stopwatch.Elapsed.TotalSeconds, 3)
$exitCode = if ($process.HasExited) { $process.ExitCode } else { -1 }
$solutionCount = 0
$minGap = $null
$summary = Join-Path $outputPath "pareto_front.csv"
$status = if (Test-Path $summary) { "completed" } else { "failed" }
if (Test-Path $summary) {
    $rows = @(Import-Csv $summary)
    $solutionCount = $rows.Count
    if ($rows.Count -gt 0) {
        $minGap = ($rows | ForEach-Object { [double]$_.mip_gap } | Measure-Object -Minimum).Minimum
    }
}

$record = [pscustomobject]@{
    boxes = [int]$Size
    input = $InputPath
    status = $status
    exit_code = $exitCode
    solution_count = $solutionCount
    minimum_mip_gap = $minGap
    wall_seconds = $wallSeconds
    cpu_seconds = $cpuSeconds
}
if (Test-Path $report) {
    $record | Export-Csv -Path $report -NoTypeInformation -Append
} else {
    $record | Export-Csv -Path $report -NoTypeInformation
}
$record | Format-List
