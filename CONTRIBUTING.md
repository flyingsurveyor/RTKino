# Contributing to RTKino

Thank you for your interest in RTKino. This project exists to make professional GNSS surveying accessible to everyone, and contributions from the community help make that possible.

## Before You Start

Please read the [Ethical Use Notice](ETHICAL_USE.md). By contributing to RTKino, you agree that your contributions will be licensed under the AGPL-3.0 license and that you share the project's commitment to ethical use.

## How to Contribute

### Reporting Bugs

If you find a bug, open an issue on GitHub with:

- **Hardware setup**: board model, ZED-F9P module, antenna, wiring
- **Firmware version**: git commit hash or release tag
- **Steps to reproduce**: what you did, what you expected, what happened
- **Serial log output**: run `pio device monitor -b 115200` and paste the relevant lines
- **Screenshots**: of the web UI or OLED display, if relevant

Field-related bugs are especially valuable — if something fails during actual survey work, that information helps improve reliability for everyone.

### Suggesting Features

Open an issue with the "feature request" label. Describe:

- What you need and why
- Your use case (rover, base, logging, survey, stakeout, etc.)
- Whether you are willing to implement it yourself

Feature suggestions grounded in real fieldwork experience are prioritized.

### Submitting Code

1. **Fork** the repository
2. **Create a feature branch**: `git checkout -b feature/your-feature-name`
3. **Make your changes** following the guidelines below
4. **Test on real hardware** — do not submit untested code
5. **Commit** with clear messages: `git commit -m "Add RTCM1124 (BeiDou MSM4) output option"`
6. **Push**: `git push origin feature/your-feature-name`
7. **Open a Pull Request** with a clear description of what changed and why

### Translating

The web UI and OLED display strings are in English. If you want to help translate to other languages, open an issue to discuss the approach before starting work — we want to avoid maintaining multiple hardcoded string sets in the firmware.

## Code Guidelines

### General Principles

- **Field reliability first.** RTKino runs on battery-powered hardware in outdoor conditions. Every feature must work reliably when it matters — during an actual survey, not just on a bench.
- **Memory is precious.** The ESP32-S3 has 8 MB PSRAM but heap fragmentation kills embedded systems. Prefer stack allocation, fixed buffers, and pre-allocated structures over dynamic allocation.
- **Concurrency is real.** RTKino uses FreeRTOS tasks, queues, and mutexes. Any shared resource (SD card, GNSS state, settings) must be protected. Use RAII locking patterns.
- **Fail gracefully.** Network connections drop, SD cards get removed, GNSS signals are lost. Handle every failure case without crashing.

### Code Style

- Use the existing code style — look at the surrounding code and match it
- All new libraries go in `lib/` with their own directory, `.h` and `.cpp` files
- Keep `main.cpp` as an orchestrator — business logic belongs in libraries
- Use `namespace` (not classes with only static methods) for singleton modules
- Comment non-obvious decisions, especially GNSS protocol details and timing constraints

### SD Card Access

**Critical:** All SD card access must be guarded by the global `sdMutex` semaphore. Failing to do so causes system-wide deadlocks. Use the RAII pattern established in the codebase:

```cpp
if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
    // SD operations here
    xSemaphoreGive(sdMutex);
}
```

### ZED-F9P Configuration

Use the `UbxValset` library for all ZED-F9P configuration. Add new key IDs to `UbxVal::Keys` with their correct types and register addresses from the u-blox receiver description document. Always specify the layer mask (RAM by default for volatile configuration).

### Web UI

The web UI is served via chunked HTTP from `lib/WebUI/WebUI.cpp`. HTML is built as C string literals sent via `sendChunk()`. This is not elegant, but it works reliably on constrained hardware without a filesystem-based template engine.

When modifying the web UI:

- Test on both desktop and mobile browsers
- Keep JavaScript minimal — no frameworks, no build tools
- Ensure all API calls handle network errors gracefully
- Use the existing CSS class conventions

### Hardware Abstraction

All pin definitions live in `include/config.h` behind `#ifdef BOARD_*` guards. If you add support for a new board:

1. Add a new `#ifdef BOARD_YOUR_BOARD` block in `config.h`
2. Add a new environment in `platformio.ini`
3. Document the pin mapping in this file and in the README
4. Test on the actual board

## What We Need Help With

If you are looking for something to work on, here are areas where contributions would be particularly valuable:

- **Support for additional GNSS receivers** (e.g., Septentrio Mosaic-X5, Unicore UM980) — RTKino currently targets the ZED-F9P (firmware 1.51 at the moment), but the architecture is modular enough to support other receivers
- **Multi-language support** for the web UI
- **Testing and documentation** — field reports, setup guides, antenna comparisons, accuracy benchmarks
- **PPK workflow integration** — tighter connection with post-processing tools
- **Platform ports** — adapting to other ESP32-S3 boards, documenting wiring for different GNSS modules

## Code of Conduct

Be respectful. This project is built by people who care about making technology accessible. Disagreements about implementation are welcome; personal attacks are not.

We are surveyors, farmers, engineers, hobbyists, and curious people from all over the world. Treat each other accordingly.

## License

By submitting a contribution, you agree that your work will be licensed under the [AGPL-3.0](LICENSE), the same license as the rest of the project.
