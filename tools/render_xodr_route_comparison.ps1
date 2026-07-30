param(
    [Parameter(Mandatory = $true)]
    [string]$BaselineXodr,
    [Parameter(Mandatory = $true)]
    [string]$FixedXodr,
    [Parameter(Mandatory = $true)]
    [string]$ScenarioIr,
    [string]$ValidationJson = "",
    [string]$HeaderTitle = "scene-0061 Ego control-corridor diagnostic",
    [string]$BaselineTitle = "Baseline: local lane strips",
    [string]$FixedTitle = "Ego corridor only (not the map)",
    [Parameter(Mandatory = $true)]
    [string]$Output
)

Add-Type -AssemblyName System.Drawing

function Read-XodrGeometry([string]$Path) {
    [xml]$document = Get-Content -LiteralPath $Path -Raw
    $rows = @()
    foreach ($road in @($document.OpenDRIVE.road)) {
        foreach ($geometry in @($road.planView.geometry)) {
            $rows += [pscustomobject]@{
                RoadId = [string]$road.id
                X = [double]$geometry.x
                Y = [double]$geometry.y
                Heading = [double]$geometry.hdg
                Length = [double]$geometry.length
            }
        }
    }
    return $rows
}

function Read-Trajectory([string]$Path) {
    $ir = Get-Content -LiteralPath $Path -Raw | ConvertFrom-Json
    return @($ir.ego.reference_trajectory | ForEach-Object {
        [pscustomobject]@{ X = [double]$_.x; Y = [double]$_.y }
    })
}

function End-Point($geometry) {
    return [pscustomobject]@{
        X = $geometry.X + $geometry.Length * [math]::Cos($geometry.Heading)
        Y = $geometry.Y + $geometry.Length * [math]::Sin($geometry.Heading)
    }
}

function Expand-GeometryPoints($geometries) {
    $points = @()
    foreach ($geometry in @($geometries)) {
        $points += [pscustomobject]@{ X = $geometry.X; Y = $geometry.Y }
        $points += End-Point $geometry
    }
    return $points
}

function New-Transform($bounds, [double]$x0, [double]$y0, [double]$width, [double]$height) {
    $padding = 48.0
    $rangeX = [math]::Max(1.0, $bounds.MaxX - $bounds.MinX)
    $rangeY = [math]::Max(1.0, $bounds.MaxY - $bounds.MinY)
    $scale = [math]::Min(($width - 2.0 * $padding) / $rangeX, ($height - 2.0 * $padding) / $rangeY)
    return [pscustomobject]@{
        X0 = $x0
        Y0 = $y0
        Width = $width
        Height = $height
        Scale = $scale
        MinX = $bounds.MinX
        MaxY = $bounds.MaxY
        Padding = $padding
    }
}

function Convert-Point($transform, [double]$x, [double]$y) {
    return New-Object System.Drawing.PointF(
        ($transform.X0 + $transform.Padding + ($x - $transform.MinX) * $transform.Scale),
        ($transform.Y0 + $transform.Height - $transform.Padding - ($y - ($transform.MaxY - ($transform.Height - 2.0 * $transform.Padding) / $transform.Scale)) * $transform.Scale)
    )
}

function Get-Bounds($groups) {
    $points = @($groups | ForEach-Object { $_ })
    return [pscustomobject]@{
        MinX = ($points | Measure-Object X -Minimum).Minimum
        MaxX = ($points | Measure-Object X -Maximum).Maximum
        MinY = ($points | Measure-Object Y -Minimum).Minimum
        MaxY = ($points | Measure-Object Y -Maximum).Maximum
    }
}

function Draw-Panel($graphics, $geometries, $trajectory, $transform, [string]$title, $roadPen, $titleBrush, [string]$detail) {
    $panel = New-Object System.Drawing.RectangleF($transform.X0, $transform.Y0, $transform.Width, $transform.Height)
    $graphics.FillRectangle((New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(18, 25, 34))), $panel)
    $graphics.DrawRectangle((New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(70, 84, 102), 1)), $panel.X, $panel.Y, $panel.Width, $panel.Height)

    foreach ($geometry in @($geometries)) {
        $start = Convert-Point $transform $geometry.X $geometry.Y
        $endPoint = End-Point $geometry
        $end = Convert-Point $transform $endPoint.X $endPoint.Y
        $graphics.DrawLine($roadPen, $start, $end)
    }

    $trajectoryPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(80, 178, 255), 3)
    $trajectoryPoints = @($trajectory | ForEach-Object { Convert-Point $transform $_.X $_.Y })
    if ($trajectoryPoints.Count -gt 1) {
        $graphics.DrawLines($trajectoryPen, $trajectoryPoints)
    }
    if ($trajectoryPoints.Count -gt 0) {
        $graphics.FillEllipse((New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 235, 120))), $trajectoryPoints[0].X - 5, $trajectoryPoints[0].Y - 5, 10, 10)
        $last = $trajectoryPoints[$trajectoryPoints.Count - 1]
        $graphics.FillEllipse((New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(255, 100, 100))), $last.X - 5, $last.Y - 5, 10, 10)
    }
    $graphics.DrawString($title, (New-Object System.Drawing.Font("Segoe UI", 18, [System.Drawing.FontStyle]::Bold)), $titleBrush, $transform.X0 + 18, $transform.Y0 + 16)
    $graphics.DrawString($detail, (New-Object System.Drawing.Font("Segoe UI", 11)), $titleBrush, $transform.X0 + 18, $transform.Y0 + 48)
}

$baseline = Read-XodrGeometry $BaselineXodr
$fixed = Read-XodrGeometry $FixedXodr
$trajectory = Read-Trajectory $ScenarioIr
$baselineRoadCount = @($baseline | Select-Object -ExpandProperty RoadId -Unique).Count
$fixedRoadCount = @($fixed | Select-Object -ExpandProperty RoadId -Unique).Count
$validationPath = $ValidationJson
if ([string]::IsNullOrWhiteSpace($validationPath)) {
    $candidateValidationPath = Join-Path (Split-Path -Parent $Output) "xodr_route_static_validation.json"
    if (Test-Path -LiteralPath $candidateValidationPath) {
        $validationPath = $candidateValidationPath
    }
}
$validation = $null
if (-not [string]::IsNullOrWhiteSpace($validationPath) -and (Test-Path -LiteralPath $validationPath)) {
    $validation = Get-Content -LiteralPath $validationPath -Raw | ConvertFrom-Json
}
$baselineJunctionCount = if ($null -ne $validation) { [int]$validation.baseline.junction_count } else { 0 }
$fixedJunctionCount = if ($null -ne $validation) { [int]$validation.fixed.junction_count } else { 0 }
$baselineDetail = "{0} roads | {1} junctions | local static metrics unavailable" -f $baselineRoadCount, $baselineJunctionCount
$fixedDetail = "{0} roads | {1} junctions | local static metrics unavailable" -f $fixedRoadCount, $fixedJunctionCount
if ($null -ne $validation) {
    $baselineDetail = "{0} roads | {1} junctions | local static: {2:P1} inside lane | centerline P95 {3:F2} m" -f `
        $baselineRoadCount, $baselineJunctionCount, [double]$validation.baseline.inside_lane_fraction, [double]$validation.baseline.centerline_distance_m.p95
    $fixedDetail = "{0} roads | {1} junctions | local static: {2:P1} inside lane | centerline P95 {3:F2} m" -f `
        $fixedRoadCount, $fixedJunctionCount, [double]$validation.fixed.inside_lane_fraction, [double]$validation.fixed.centerline_distance_m.p95
}
$allPoints = @(Expand-GeometryPoints $baseline) + @(Expand-GeometryPoints $fixed) + $trajectory
$bounds = Get-Bounds $allPoints
$bitmap = New-Object System.Drawing.Bitmap(1800, 980)
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
$graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$graphics.Clear([System.Drawing.Color]::FromArgb(10, 14, 20))
$titleBrush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(238, 242, 247))
$graphics.DrawString($HeaderTitle, (New-Object System.Drawing.Font("Segoe UI", 24, [System.Drawing.FontStyle]::Bold)), $titleBrush, 42, 24)
$graphics.DrawString("Blue = Scenario IR ego trajectory | Yellow = start | Red = end", (New-Object System.Drawing.Font("Segoe UI", 12)), $titleBrush, 44, 62)

$left = New-Transform $bounds 30 100 850 770
$right = New-Transform $bounds 920 100 850 770
Draw-Panel $graphics $baseline $trajectory $left $BaselineTitle (New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(220, 116, 96), 2)) $titleBrush $baselineDetail
Draw-Panel $graphics $fixed $trajectory $right $FixedTitle (New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(96, 220, 142), 3)) $titleBrush $fixedDetail

$graphics.DrawString("Metrics are local static geometry checks, not CARLA runtime acceptance. Remote CARLA waypoint, physics, and collision validation remain required.", (New-Object System.Drawing.Font("Segoe UI", 12)), $titleBrush, 44, 914)
New-Item -ItemType Directory -Force (Split-Path -Parent $Output) | Out-Null
$bitmap.Save($Output, [System.Drawing.Imaging.ImageFormat]::Png)
$graphics.Dispose()
$bitmap.Dispose()
