# OpenSCENARIO Export MVP

ClosedLoopBench exports OpenSCENARIO as a portable exchange artifact, not as the primary runtime path.

## Requirements

- Build a valid XML document from Scenario IR using only the Python standard library.
- Include FileHeader, RoadNetwork, Entities, Init, Storyboard, and StopTrigger.
- Represent ego and actors with initial TeleportAction and speed actions.
- Do not require esmini for unit tests.
- Add optional esmini smoke tests later.
