# Build and Run Instructions

The simulation has **two run modes**:
- **Terminal mode** — pure CLI, no graphics. Works without SFML at runtime.
- **GUI mode** — startup input dialog + live dashboard. Requires SFML 3.x.

The binary still **links against SFML for both modes** (so you need SFML
installed to compile), but only the GUI mode actually opens windows.

---

## Install SFML 3.x

### macOS (Homebrew)

```bash
brew install sfml
```

The Makefile automatically detects Homebrew on `/opt/homebrew` (Apple Silicon)
or `/usr/local` (Intel Mac).

### Linux (Ubuntu / Debian)

Ubuntu 24.04+ ships SFML 2.6 by default. For SFML 3 you'll need to either build
from source or use a third-party PPA. If you already have SFML 2 installed and
want to keep it, see the "Downgrading for SFML 2" section below.

```bash
sudo apt update
sudo apt install libsfml-dev build-essential
```

### Windows

1. Download **SFML 3.0** from <https://www.sfml-dev.org/download.php>
2. Extract to `C:\SFML`
3. Add `C:\SFML\bin` to your PATH
4. Use `g++` (MinGW) or Visual Studio to build

---

## Build

From the repo root:

```bash
make
```

This produces a `simulation` executable.

---

## Run

The Makefile provides three targets:

```bash
make run         # Default - launches GUI mode
make gui         # Same as 'make run' (explicit)
make terminal    # Runs in terminal mode
```

You can also call the binary directly:

```bash
./simulation                # GUI mode
./simulation --gui          # GUI mode
./simulation --terminal     # Terminal mode
./simulation -t             # Short form
./simulation --help         # Usage info
```

---

## Terminal Mode

Pure command-line. Prompts for parameters one at a time:

```
================================================================
  Sensor Fusion System - Terminal Mode
================================================================
Enter simulation parameters (press Enter to keep default):

  Vehicle speed [5 m/s]:
  Turn rate [10 deg/s]:
  Duration (0 = infinite, will stop at 60s for terminal mode) [30 s]:
  GPS noise sigma [1.5 m]:
  Vehicle mass [1000 kg]:
```

Press Enter on any prompt to accept the default. Invalid input is rejected
with a clear message and the prompt is re-asked.

When the simulation runs, it prints one line per timestep with the current
position, velocity, heading, and temperature.

---

## GUI Mode

Opens a 560×540 startup dialog with five text fields. Type values, press
Enter or click **START**. Invalid fields are highlighted in red with an
error tooltip.

After clicking START, the main 1400×800 dashboard opens with:

| Region              | Contents                                              |
|---------------------|-------------------------------------------------------|
| Left (map)          | Grid + axes + vehicle + trajectory trail              |
| Right (top panels)  | Live IMU, GPS, Temperature, Wheel Encoder readouts    |
| Right (bottom)      | Highlighted FUSED STATE ESTIMATE panel                |
| Bottom of map       | START / PAUSE / RESET buttons                         |
| Title bar           | Status indicator (READY / RUNNING / PAUSED)           |

### GUI Controls

| Key / Button | Action                              |
|--------------|--------------------------------------|
| **START**    | Begin / resume simulation           |
| **PAUSE**    | Pause                               |
| **RESET**    | Rebuild from t=0                    |
| `SPACE`      | Toggle pause                        |
| `R`          | Reset                               |
| `ESC`        | Quit                                |

### Input Dialog Controls

| Key      | Action                         |
|----------|--------------------------------|
| `TAB`    | Next field                     |
| `↑` / `↓` | Move between fields            |
| `ENTER`  | Confirm and start              |
| `ESC`    | Cancel                         |

---

## Verifying your SFML version

```bash
pkg-config --modversion sfml-graphics
```

Or on Mac:
```bash
brew info sfml
```

The code is written for **SFML 3.x**. If you have SFML 2.x, see below.

---

## Downgrading for SFML 2

If you must use SFML 2 (common on Linux), the main differences are in
`Visualizer.cpp` and `InputDialog.cpp`. Replace these patterns:

| SFML 3                                          | SFML 2                                |
|-------------------------------------------------|---------------------------------------|
| `font.openFromFile(path)`                       | `font.loadFromFile(path)`             |
| `sf::VideoMode({W, H})`                         | `sf::VideoMode(W, H)`                 |
| `while (const auto event = window.pollEvent())` | `sf::Event e; while (window.pollEvent(e))` |
| `event->is<sf::Event::Closed>()`                | `e.type == sf::Event::Closed`         |
| `event->getIf<sf::Event::KeyPressed>()->code`   | `e.key.code`                          |
| `sf::Keyboard::Key::Escape`                     | `sf::Keyboard::Escape`                |
| `setPosition({x, y})`                           | `setPosition(x, y)`                   |
| `sf::Vertex{pos, color}`                        | `sf::Vertex(pos, color)`              |
| `sf::PrimitiveType::Lines`                      | `sf::Lines`                           |
| `sf::degrees(angle)`                            | `angle` (plain float)                 |
| `sf::Text text(font, str, size);`               | `sf::Text text; text.setFont(font); text.setString(str); text.setCharacterSize(size);` |
| `sf::FloatRect({x, y}, {w, h})`                 | `sf::FloatRect(x, y, w, h)`           |
| `rect.position.x`                               | `rect.left`                           |
| `rect.size.x`                                   | `rect.width`                          |

---

## Troubleshooting

**`SFML/Graphics.hpp not found`** — SFML isn't installed or the include path
isn't set. Run `brew install sfml` (Mac) or install the dev package on Linux.

**Errors about `loadFromFile` / `pollEvent(Event&)`** — you have SFML 2
installed but the code expects SFML 3. Either upgrade SFML or apply the
downgrade patches above.

**`make: command not found`** (Mac) — install Xcode command-line tools:
```bash
xcode-select --install
```

**Window opens but no text is visible** — the font loader couldn't find
Helvetica or Arial. Check the terminal for `[Visualizer] Warning: no font
found` and add your system's font path to the `fontPaths[]` array in
`Visualizer.cpp` and `InputDialog.cpp`.
