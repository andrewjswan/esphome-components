# Project overview
Custom components for ESPHome

## Project Overview & Purpose

*   **Primary Goal:** ESPHome is a system to configure microcontrollers (like ESP32, ESP8266, RP2040, and LibreTiny-based chips) using simple yet powerful YAML configuration files. It generates C++ firmware that can be compiled and flashed to these devices, allowing users to control them remotely through home automation systems.
*   **Business Domain:** Internet of Things (IoT), Home Automation.

## Architectural Patterns

*   **Overall Architecture:** The project follows a code-generation architecture. The Python code parses user-defined YAML configuration files and generates C++ source code. This C++ code is then compiled and flashed to the target microcontroller using PlatformIO.

*   **Directory Structure Philosophy:**
    *   `/components`: Contains the individual components that can be used in ESPHome configurations. Each component is a self-contained unit with its own C++ and Python code.
    *   `/tests`: Contains unit and integration tests.
    *   `/examples`: Contains examples of using components.
    *   `/docs`: Contains documentation ащк components.

## Development Guidelines
1. **ESPHome Compatibility**: All code must be compatible with ESPHome framework
2. **Memory Efficiency**: Optimize for ESP8266/ESP32 memory constraints  
3. **Code Style**: Follow ESPHome coding conventions and C++ best practices
   
## Coding standards
Follow the [ESPHome coding standards](https://esphome.io/guides/coding_standards.html).
