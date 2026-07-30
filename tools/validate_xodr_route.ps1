param(
    [Parameter(Mandatory = $true)]
    [string]$BaselineXodr,
    [Parameter(Mandatory = $true)]
    [string]$FixedXodr,
    [Parameter(Mandatory = $true)]
    [string]$ScenarioIr,
    [Parameter(Mandatory = $true)]
    [string]$Output,
    [ValidateSet("all", "map", "network", "corridor")]
    [string]$RoadScope = "network"
)

function Read-Xodr([string]$Path, [string]$Scope) {
    [xml]$document = Get-Content -LiteralPath $Path -Raw
    $segments = @()
    $laneWidths = @()
    $junctions = @($document.OpenDRIVE.junction | Where-Object { $_ -ne $null -and $_ -ne "" })
    $allRoads = @($document.OpenDRIVE.road)
    $roads = @($allRoads | Where-Object {
        $road = $_
        switch ($Scope) {
            "map" { $road.name -like "nuscenes_lane_*" }
            "network" { $road.name -ne "ego_route_corridor" }
            "corridor" {
                $road.name -eq "ego_route_corridor" -or
                $road.name -eq "route_aligned_ego_corridor" -or
                $allRoads.Count -eq 1
            }
            default { $true }
        }
    })
    foreach ($road in $roads) {
        $lane = $road.lanes.laneSection.right.lane | Select-Object -First 1
        $roadLaneWidth = 3.5
        if ($lane.width.a) {
            $roadLaneWidth = [double]$lane.width.a
        }
        $laneWidths += $roadLaneWidth
        foreach ($geometry in @($road.planView.geometry)) {
            $heading = [double]$geometry.hdg
            $length = [double]$geometry.length
            $segments += [pscustomobject]@{
                X1 = [double]$geometry.x
                Y1 = [double]$geometry.y
                X2 = [double]$geometry.x + $length * [math]::Cos($heading)
                Y2 = [double]$geometry.y + $length * [math]::Sin($heading)
                HeadingDeg = $heading * 180.0 / [math]::PI
                LaneWidthM = $roadLaneWidth
            }
        }
    }
    $representativeLaneWidth = if ($laneWidths.Count -gt 0) {
        ($laneWidths | Measure-Object -Average).Average
    } else {
        3.5
    }
    return [pscustomobject]@{
        Segments = $segments
        LaneWidthM = $representativeLaneWidth
        RoadCount = $roads.Count
        TotalRoadCount = $allRoads.Count
        MapLaneRoadCount = @($allRoads | Where-Object { $_.name -like "nuscenes_lane_*" }).Count
        ConnectorRoadCount = @($allRoads | Where-Object { $_.name -like "inferred_connector_*" }).Count
        RouteInferenceRoadCount = @($allRoads | Where-Object { $_.name -like "inferred_route_*" }).Count
        EgoCorridorRoadCount = @($allRoads | Where-Object { $_.name -eq "ego_route_corridor" }).Count
        JunctionCount = $junctions.Count
        RoadScope = $Scope
    }
}

function Point-SegmentDistance($point, $segment) {
    $dx = $segment.X2 - $segment.X1
    $dy = $segment.Y2 - $segment.Y1
    $denominator = $dx * $dx + $dy * $dy
    if ($denominator -le 1e-12) {
        return [pscustomobject]@{ Distance = [math]::Sqrt(($point.X - $segment.X1) * ($point.X - $segment.X1) + ($point.Y - $segment.Y1) * ($point.Y - $segment.Y1)); HeadingDeg = $segment.HeadingDeg; LaneWidthM = $segment.LaneWidthM }
    }
    $ratio = (($point.X - $segment.X1) * $dx + ($point.Y - $segment.Y1) * $dy) / $denominator
    $ratio = [math]::Max(0.0, [math]::Min(1.0, $ratio))
    $x = $segment.X1 + $ratio * $dx
    $y = $segment.Y1 + $ratio * $dy
    return [pscustomobject]@{ Distance = [math]::Sqrt(($point.X - $x) * ($point.X - $x) + ($point.Y - $y) * ($point.Y - $y)); HeadingDeg = $segment.HeadingDeg; LaneWidthM = $segment.LaneWidthM }
}

function Angle-Difference([double]$left, [double]$right) {
    $difference = ($left - $right) % 360.0
    if ($difference -gt 180.0) { $difference -= 360.0 }
    if ($difference -lt -180.0) { $difference += 360.0 }
    return [math]::Abs($difference)
}

function Percentile($values, [double]$percent) {
    $sorted = @($values | Sort-Object)
    if ($sorted.Count -eq 0) { return $null }
    $index = [int][math]::Round(($sorted.Count - 1) * $percent / 100.0)
    return [double]$sorted[$index]
}

function Measure-Route([string]$Path, $trajectory) {
    $xodr = Read-Xodr $Path $RoadScope
    if ($xodr.Segments.Count -eq 0) {
        throw "no XODR road geometry matches road scope '$RoadScope': $Path"
    }
    $distances = @()
    $headings = @()
    $inside = 0
    foreach ($state in $trajectory) {
        $point = [pscustomobject]@{ X = [double]$state.x; Y = [double]$state.y }
        $nearest = $xodr.Segments | ForEach-Object { Point-SegmentDistance $point $_ } | Sort-Object Distance | Select-Object -First 1
        $distances += $nearest.Distance
        $headings += Angle-Difference ([double]$state.yaw) $nearest.HeadingDeg
        if ($nearest.Distance -le $nearest.LaneWidthM / 2.0) { $inside++ }
    }
    return [ordered]@{
        path = (Resolve-Path $Path).Path
        road_scope = $xodr.RoadScope
        road_count = $xodr.RoadCount
        total_road_count = $xodr.TotalRoadCount
        map_lane_road_count = $xodr.MapLaneRoadCount
        connector_road_count = $xodr.ConnectorRoadCount
        route_inference_road_count = $xodr.RouteInferenceRoadCount
        ego_corridor_road_count = $xodr.EgoCorridorRoadCount
        junction_count = $xodr.JunctionCount
        lane_width_m = $xodr.LaneWidthM
        pose_count = $trajectory.Count
        inside_lane_fraction = $inside / [double]$trajectory.Count
        centerline_distance_m = [ordered]@{ p50 = Percentile $distances 50; p95 = Percentile $distances 95; max = ($distances | Measure-Object -Maximum).Maximum }
        heading_error_deg = [ordered]@{ p50 = Percentile $headings 50; p95 = Percentile $headings 95; max = ($headings | Measure-Object -Maximum).Maximum }
    }
}

$ir = Get-Content -LiteralPath $ScenarioIr -Raw | ConvertFrom-Json
$trajectory = @($ir.ego.reference_trajectory)
$report = [ordered]@{
    schema_version = "xodr_route_static_validation.v1"
    road_scope = $RoadScope
    baseline = Measure-Route $BaselineXodr $trajectory
    fixed = Measure-Route $FixedXodr $trajectory
    runtime_validation = "required_remote_carla"
}
New-Item -ItemType Directory -Force (Split-Path -Parent $Output) | Out-Null
$report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $Output -Encoding utf8
$report | ConvertTo-Json -Depth 8
