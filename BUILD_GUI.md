# GUI Build Instructions

The simulation uses **SFML 3.x** for its graphical interface. You need to install SFML before building. Choose the instructions for your operating system.

---

## macOS (Homebrew)

```bash
brew install sfml
```

Then from the repo root:
```bash
make run
```

The Makefile automatically detects Homebrew's SFML at `/opt/homebrew` (Apple Silicon) or `/usr/local` (Intel).

If Homebrew's SFML is at a different path, you may need to set it manually:
```bash
g++ -std=c++17 src/*.cpp -o simulation \
    -I/opt/homebrew/include -L/opt/homebrew/lib \
    -lsfml-graphics -lsfml-window -lsfml-system
./simulation
```

---

## Linux (Ubuntu / Debian / WSL)

Ubuntu 24.04+ ships with SFML 2.6 by default. If you have SFML 2.6, you'll need to either install SFML 3 manually or downgrade the Visualizer code. For SFML 3 on Linux, the easiest path is building from source or using a PPA.

```bash
sudo apt update
sudo apt install libsfml-dev build-essential
make run
```

Note: if you get compile errors about `openFromFile` or `pollEvent() returning std::optional`, you have SFML 2.x installed. See the "Downgrading for SFML 2" section below.

---

## Windows

1. Download **SFML 3.0** from <https://www.sfml-dev.org/download.php>
2. Extract to `C:\SFML`
3. Add `C:\SFML\bin` to your PATH
4. Build with:
   ```bash
   g++ -std=c++17 src/*.cpp -o simulation.exe ^
       -IC:\SFML\include -LC:\SFML\lib ^
       -lsfml-graphics -lsfml-window -lsfml-system
   simulation.exe
   ```

---

## Verifying your SFML version

```bash
pkg-config --modversion sfml-graphics
```

Or on Mac:
```bash
brew info sfml
```

- **3.x** — the code works as-is
- **2.x** — use the downgrade notes below

---

## Downgrading for SFML 2

If you must use SFML 2 (common on Linux), replace in `Visualizer.cpp`:

| SFML 3                                          | SFML 2                             |
|-------------------------------------------------|------------------------------------|
| `font.openFromFile(path)`                       | `font.loadFromFile(path)`          |
| `sf::VideoMode({W, H})`                         | `sf::VideoMode(W, H)`              |
| `while (const auto event = window.pollEvent())` | `sf::Event event; while (window.pollEvent(event))` |
| `event->is<sf::Event::Closed>()`                | `event.type == sf::Event::Closed`  |
| `event->getIf<sf::Event::KeyPressed>()->code`   | `event.key.code`                   |
| `sf::Keyboard::Key::Escape`                     | `sf::Keyboard::Escape`             |
| `setPosition({x, y})`                           | `setPosition(x, y)`                |
| `sf::Vertex{pos, color}`                        | `sf::Vertex(pos, color)`           |
| `sf::PrimitiveType::Lines`                      | `sf::Lines`                        |
| `sf::degrees(angle)`                            | `angle` (plain float)              |
| `sf::Text text(font, str, size);`               | `sf::Text text; text.setFont(font); text.setString(str); text.setCharacterSize(size);` |

---

## Controls

| Key     | Action                     |
|---------|----------------------------|
| `SPACE` | Pause / resume simulation  |
| `R`     | Clear the trajectory trail |
| `ESC`   | Close window and quit      |

