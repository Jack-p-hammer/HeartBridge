# HeartBridge

Software, electronics, mechanical parts and simulation for HeartBridge, an automated CPR machine. The project started as MIT 2.009 Green Team's 2025 class project and has continued since.

## Repository layout

| Folder | What's in it |
|---|---|
| [`Raspberry Pi code/`](Raspberry%20Pi%20code/) | The Python controller that runs on the Raspberry Pi: state machine, sensing, motor control (moteus), screen and audio prompts, tests and bench scripts. See its [README](Raspberry%20Pi%20code/README.md) for setup. |
| `hardware/pcb/main-board/` | KiCad project for the main CPR machine board, plus its fab outputs in `production/` |
| `hardware/pcb/mcu-board/` | KiCad project for the MCU board |
| `hardware/pcb/contactor-board/` | KiCad project for the contactor board |
| `hardware/pcb/single-pager-legacy/` | Earlier single-board design, kept for reference |
| `hardware/mechanical/` | 3D-print sources (`.step`, `.3mf`) for the enclosure, brackets and mounts |
| `simulation/` | Simulink model (`Heartbridge.slx`) and supporting MATLAB scripts. `legacy-m4/` holds the force sensor calibration from the class project. |
| `docs/` | Project documentation |

## Getting started

- **Controller code:** follow [`Raspberry Pi code/README.md`](Raspberry%20Pi%20code/README.md).
- **PCBs:** open the `.kicad_pro` file in a board's folder with KiCad. Custom symbol and footprint libraries live inside each board folder and are referenced relative to it.
- **Simulation:** in MATLAB, run `simulinkSetup.m` then open `Heartbridge.slx`.

## History

The repository was reorganized out of its class-project layout (`M3`–`M6`, `CPR Machine`, etc.) in October 2026. Older work is kept in tags rather than folders:

- `pre-restructure`: the last commit before the reorganization
- `archive/final`, `archive/integration`, `archive/zeroing`, `archive/hmi_v2`: class-era branches

To look at one, run `git checkout <tag>`. To keep working from one, create a branch: `git switch -c <name> <tag>`.
