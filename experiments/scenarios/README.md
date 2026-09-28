# Scenario presets

YAML files for `avatar run|compare|export-viz --scenario-file <file>`. Keys:

```yaml
scenario: harbor_fleet        # registered scenario builder
duration_s: 600               # default mission length (CLI --duration overrides)
args:                         # keyword arguments of the builder
  acoustic: m64               # channel profile (avatar.comm.channel.CHANNEL_PROFILES)
  n_uuv: 2
  paths:                      # per-agent trajectory overrides (avatar.sim.trajectories)
    uuv_0: {kind: figure8, start: [30, 0], z: -4.0, length: 40, width: 16, speed_mps: 0.5}
```

`avatar trajectories` lists the kinds and their parameters. The same keys work
on the command line: `--scenario-arg paths.uuv_0.kind=circle --scenario-arg paths.uuv_0.radius=8`.

| File | What it shows |
|---|---|
| `fleet_default.yaml` | Reference fleet, default paths |
| `fleet_complex_turns.yaml` | Figure-8 and zig-zag paths (gyro scale error, heading drift) |
| `fleet_heights.yaml` | UAV helix 2 → 7 m, BlueROV2 yo-yo 8 → 1 m depth |
| `fleet_surfacing.yaml` | BlueROV2 surfaces and uses Wi-Fi while surfaced (surfacing windows) |
| `fleet_exploration.yaml` | Expanding spiral: no revisits, no self loop closures |
