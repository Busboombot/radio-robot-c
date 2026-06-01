# mbdeploy

A standalone command-line tool for building and deploying micro:bit firmware to
one or more devices via pyOCD.

## Installation

```bash
pipx install --editable ./mbdeploy
```

Re-install after editing source:

```bash
pipx install --editable --force ./mbdeploy
```

## Subcommands

| Subcommand | Description |
|------------|-------------|
| `build`    | Compile the micro:bit firmware. |
| `deploy`   | Flash firmware to one or more micro:bit devices. |
| `list`     | List all detected micro:bit devices. |
| `probe`    | Probe a device and report its state. |

Run `mbdeploy --help` or `mbdeploy <subcommand> --help` for full usage.

## Top-level flags

| Flag        | Description |
|-------------|-------------|
| `--version` | Print the installed mbdeploy version and exit. |
| `--agent`   | Print the detailed agent manual (usage, recipes, device model) and exit. |

`mbdeploy --agent` prints a complete manual aimed at AI coding agents and
power users driving the tool non-interactively.
