param(
    [Parameter(Mandatory = $true)]
    [string]$Xodr,
    [Parameter(Mandatory = $true)]
    [string]$Output,
    [double]$MaxRoadLinkGapM = 1.0,
    [switch]$RequireMapTopology,
    [switch]$RequireJunctionTopology,
    [switch]$RequireBoundaryAudit
)

[xml]$document = Get-Content -LiteralPath $Xodr -Raw
if ($document.OpenDRIVE.LocalName -ne "OpenDRIVE") {
    throw "not an OpenDRIVE document: $Xodr"
}

$roads = @($document.OpenDRIVE.SelectNodes("./road"))
$junctions = @($document.OpenDRIVE.SelectNodes("./junction"))
$roadIds = @{}
$junctionIds = @{}
$roadEndpoints = @{}
$errors = @()

foreach ($road in $roads) {
    $roadId = [string]$road.id
    if ([string]::IsNullOrWhiteSpace($roadId)) {
        $errors += "road without id"
    } elseif ($roadIds.ContainsKey($roadId)) {
        $errors += "duplicate road id: $roadId"
    } else {
        $roadIds[$roadId] = $true
    }
    $geometries = @($road.SelectNodes("./planView/geometry"))
    if ($geometries.Count -gt 0) {
        $first = $geometries[0]
        $last = $geometries[$geometries.Count - 1]
        $lastHeading = [double]$last.hdg
        $lastLength = [double]$last.length
        $roadEndpoints[$roadId] = [pscustomobject]@{
            startX = [double]$first.x
            startY = [double]$first.y
            endX = [double]$last.x + $lastLength * [math]::Cos($lastHeading)
            endY = [double]$last.y + $lastLength * [math]::Sin($lastHeading)
        }
    } else {
        $errors += "road $roadId has no planView geometry"
    }
}
foreach ($junction in $junctions) {
    $junctionId = [string]$junction.id
    if ([string]::IsNullOrWhiteSpace($junctionId)) {
        $errors += "junction without id"
    } elseif ($junctionIds.ContainsKey($junctionId)) {
        $errors += "duplicate junction id: $junctionId"
    } else {
        $junctionIds[$junctionId] = $true
    }
}

$roadLinkCount = 0
$laneLinkCount = 0
$roadLinkEndpointGaps = @()
$mapRoads = @($roads | Where-Object { $_.name -like "nuscenes_lane_*" })
$connectorRoads = @($roads | Where-Object { $_.name -like "inferred_connector_*" })
$isolatedMapRoads = @(
    $mapRoads | Where-Object {
        @($_.SelectNodes("./link/*")).Count -eq 0
    }
)
$isolatedMapRoadIds = @($isolatedMapRoads | ForEach-Object { [string]$_.id })
$unclassifiedBoundaryRoads = @(
    $isolatedMapRoads | Where-Object {
        $boundary = $_.SelectSingleNode("./userData/property[@name='topology_boundary']")
        $null -eq $boundary -or [string]$boundary.value -ne "true"
    }
)
if ($RequireBoundaryAudit -and $unclassifiedBoundaryRoads.Count -gt 0) {
    $errors += "boundary topology audit found $($unclassifiedBoundaryRoads.Count) isolated map lane road(s) without source-boundary metadata"
}
if ($RequireMapTopology -and $mapRoads.Count -lt 2) {
    $errors += "map topology requires at least two nuscenes_lane_* roads; found $($mapRoads.Count)"
}
foreach ($road in $roads) {
    $roadId = [string]$road.id
    $roadJunction = [string]$road.junction
    if ($roadJunction -ne "-1" -and -not $junctionIds.ContainsKey($roadJunction)) {
        $errors += "road $roadId references missing junction $roadJunction"
    }
    $links = @($road.SelectNodes("./link"))
    foreach ($link in $links) {
        $references = @()
        $references += @($link.SelectNodes("./predecessor"))
        $references += @($link.SelectNodes("./successor"))
        foreach ($reference in $references) {
            $roadLinkCount++
            $elementType = [string]$reference.elementType
            if ($elementType -eq "junction") {
                if (-not $junctionIds.ContainsKey([string]$reference.elementId)) {
                    $errors += "road $roadId references missing junction $($reference.elementId)"
                }
                continue
            }
            if ($elementType -ne "road") {
                $errors += "road $roadId has unsupported link elementType $elementType"
                continue
            }
            if (-not $roadIds.ContainsKey([string]$reference.elementId)) {
                $errors += "road $roadId references missing road $($reference.elementId)"
                continue
            }
            $targetRoad = $roads | Where-Object { [string]$_.id -eq [string]$reference.elementId } | Select-Object -First 1
            $reverseName = if ([string]$reference.LocalName -eq "predecessor") { "successor" } else { "predecessor" }
            $reverse = $targetRoad.SelectSingleNode("./link/$reverseName[@elementId='$roadId']")
            $isJunctionConnector = [string]$road.name -like "inferred_connector_*"
            if ($null -eq $reverse -and -not $isJunctionConnector) {
                $errors += "road $roadId $($reference.LocalName) link to $($reference.elementId) has no reciprocal $reverseName link"
            }
            $targetEndpoint = $roadEndpoints[[string]$reference.elementId]
            $sourceEndpoint = $roadEndpoints[$roadId]
            if ($null -ne $sourceEndpoint -and $null -ne $targetEndpoint) {
                if ([string]$reference.LocalName -eq "predecessor") {
                    $dx = $sourceEndpoint.startX - $targetEndpoint.endX
                    $dy = $sourceEndpoint.startY - $targetEndpoint.endY
                } else {
                    $dx = $sourceEndpoint.endX - $targetEndpoint.startX
                    $dy = $sourceEndpoint.endY - $targetEndpoint.startY
                }
                $gap = [math]::Sqrt($dx * $dx + $dy * $dy)
                $roadLinkEndpointGaps += $gap
                if ($gap -gt $MaxRoadLinkGapM) {
                    $errors += "road $roadId $($reference.LocalName) endpoint gap exceeds $MaxRoadLinkGapM m: $gap"
                }
            }
        }
    }
    $laneLinks = @($road.SelectNodes("./lanes/laneSection/*/lane/link"))
    foreach ($laneLink in $laneLinks) {
        $laneLinkCount++
        $references = @()
        $references += @($laneLink.SelectNodes("./predecessor"))
        $references += @($laneLink.SelectNodes("./successor"))
        foreach ($reference in $references) {
            if ([string]$reference.id -ne "-1") {
                $errors += "road $roadId contains unsupported lane link id $($reference.id)"
            }
        }
    }
}

$junctionConnectionCount = 0
$junctionEndpointGaps = @()
foreach ($junction in $junctions) {
    foreach ($connection in @($junction.SelectNodes("./connection"))) {
        $junctionConnectionCount++
        $incoming = [string]$connection.incomingRoad
        $connecting = [string]$connection.connectingRoad
        if (-not $roadIds.ContainsKey($incoming)) {
            $errors += "junction $($junction.id) references missing incoming road $incoming"
        }
        if (-not $roadIds.ContainsKey($connecting)) {
            $errors += "junction $($junction.id) references missing connecting road $connecting"
        }
        $incomingRoadNode = $roads | Where-Object { [string]$_.id -eq $incoming } | Select-Object -First 1
        $connectingRoadNode = $roads | Where-Object { [string]$_.id -eq $connecting } | Select-Object -First 1
        if ($null -ne $connectingRoadNode -and [string]$connectingRoadNode.junction -ne [string]$junction.id) {
            $errors += "junction $($junction.id) connecting road $connecting has junction $($connectingRoadNode.junction)"
        }
        $incomingEndpoint = $roadEndpoints[$incoming]
        $connectingEndpoint = $roadEndpoints[$connecting]
        if ($null -ne $incomingEndpoint -and $null -ne $connectingEndpoint) {
            $gap = [math]::Sqrt(
                (($incomingEndpoint.endX - $connectingEndpoint.startX) * ($incomingEndpoint.endX - $connectingEndpoint.startX)) +
                (($incomingEndpoint.endY - $connectingEndpoint.startY) * ($incomingEndpoint.endY - $connectingEndpoint.startY))
            )
            $junctionEndpointGaps += $gap
            if ($gap -gt 12.0) {
                $errors += "junction $($junction.id) connection $($connection.id) endpoint gap exceeds 12 m: $gap"
            }
        }
        if (@($connection.SelectNodes("./laneLink")).Count -eq 0) {
            $errors += "junction $($junction.id) connection $($connection.id) has no laneLink"
        }
        if ($null -ne $incomingRoadNode) {
            $incomingSuccessor = $incomingRoadNode.SelectSingleNode("./link/successor")
            if ($null -eq $incomingSuccessor -or
                [string]$incomingSuccessor.elementType -ne "junction" -or
                [string]$incomingSuccessor.elementId -ne [string]$junction.id) {
                $errors += "junction $($junction.id) incoming road $incoming does not link to the junction"
            }
        }
        if ($null -ne $connectingRoadNode) {
            $connectorPredecessor = $connectingRoadNode.SelectSingleNode("./link/predecessor")
            $connectorSuccessor = $connectingRoadNode.SelectSingleNode("./link/successor")
            if ($null -eq $connectorPredecessor -or
                [string]$connectorPredecessor.elementType -ne "road" -or
                [string]$connectorPredecessor.elementId -ne $incoming) {
                $errors += "junction $($junction.id) connector $connecting does not link back to incoming road $incoming"
            }
            if ($null -eq $connectorSuccessor -or
                [string]$connectorSuccessor.elementType -ne "road" -or
                -not $roadIds.ContainsKey([string]$connectorSuccessor.elementId)) {
                $errors += "junction $($junction.id) connector $connecting does not link to an outgoing road"
            } else {
                $outgoingRoadNode = $roads | Where-Object { [string]$_.id -eq [string]$connectorSuccessor.elementId } | Select-Object -First 1
                $outgoingPredecessor = $outgoingRoadNode.SelectSingleNode("./link/predecessor")
                if ($null -eq $outgoingPredecessor -or
                    [string]$outgoingPredecessor.elementType -ne "junction" -or
                    [string]$outgoingPredecessor.elementId -ne [string]$junction.id) {
                    $errors += "junction $($junction.id) outgoing road $($connectorSuccessor.elementId) does not link to the junction"
                }
                $outgoingEndpoint = $roadEndpoints[[string]$connectorSuccessor.elementId]
                $connectorEndpoint = $roadEndpoints[$connecting]
                if ($null -ne $outgoingEndpoint -and $null -ne $connectorEndpoint) {
                    $outgoingDx = $connectorEndpoint.endX - $outgoingEndpoint.startX
                    $outgoingDy = $connectorEndpoint.endY - $outgoingEndpoint.startY
                    $outgoingGap = [math]::Sqrt($outgoingDx * $outgoingDx + $outgoingDy * $outgoingDy)
                    $junctionEndpointGaps += $outgoingGap
                    if ($outgoingGap -gt 12.0) {
                        $errors += "junction $($junction.id) connector $connecting endpoint gap to outgoing road exceeds 12 m: $outgoingGap"
                    }
                }
            }
        }
    }
}
if ($RequireJunctionTopology) {
    if ($junctions.Count -lt 1) {
        $errors += "junction topology requires at least one junction"
    }
    if ($connectorRoads.Count -lt 1) {
        $errors += "junction topology requires at least one inferred_connector_* road"
    }
    if ($junctionConnectionCount -lt 1) {
        $errors += "junction topology requires at least one junction connection"
    }
}

$junctionEndpointGapMax = if ($junctionEndpointGaps.Count -gt 0) {
    ($junctionEndpointGaps | Measure-Object -Maximum).Maximum
} else {
    0.0
}
$roadLinkEndpointGapMax = if ($roadLinkEndpointGaps.Count -gt 0) {
    ($roadLinkEndpointGaps | Measure-Object -Maximum).Maximum
} else {
    0.0
}

$result = [ordered]@{
    schema_version = "xodr_topology_validation.v1"
    path = (Resolve-Path $Xodr).Path
    status = if ($errors.Count -eq 0) { "passed" } else { "failed" }
    road_count = $roads.Count
    junction_count = $junctions.Count
    road_link_count = $roadLinkCount
    road_link_endpoint_gap_count = $roadLinkEndpointGaps.Count
    road_link_endpoint_gap_max_m = $roadLinkEndpointGapMax
    max_road_link_gap_m = $MaxRoadLinkGapM
    lane_link_count = $laneLinkCount
    junction_connection_count = $junctionConnectionCount
    junction_endpoint_gap_count = $junctionEndpointGaps.Count
    junction_endpoint_gap_max_m = $junctionEndpointGapMax
    map_topology_required = [bool]$RequireMapTopology
    junction_topology_required = [bool]$RequireJunctionTopology
    map_lane_road_count = $mapRoads.Count
    connector_road_count = $connectorRoads.Count
    route_inference_road_count = @($roads | Where-Object { $_.name -like "inferred_route_*" }).Count
    ego_corridor_road_count = @($roads | Where-Object { $_.name -eq "ego_route_corridor" }).Count
    isolated_map_lane_road_ids = $isolatedMapRoadIds
    isolated_map_lane_boundary_count = $isolatedMapRoads.Count - $unclassifiedBoundaryRoads.Count
    isolated_map_lane_unclassified_count = $unclassifiedBoundaryRoads.Count
    isolated_map_lane_boundary_status = if ($unclassifiedBoundaryRoads.Count -eq 0) { "passed" } else { "failed" }
    errors = $errors
}
New-Item -ItemType Directory -Force (Split-Path -Parent $Output) | Out-Null
$result | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $Output -Encoding utf8
$result | ConvertTo-Json -Depth 8
if ($errors.Count -gt 0) {
    exit 1
}
