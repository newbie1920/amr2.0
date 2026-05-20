# App Protocol Layer

App-side protocol documentation/adapters live here.

In the vehicle-brain architecture, this layer should focus on:

- WebSocket command schemas sent to firmware.
- Telemetry/map/path/status schemas received from firmware.
- Backward-compatible adapters for existing UI stores.

The app should not be described as the navigation authority for the physical robot.
