# ARDUCAM_B0462-to-ESP32s3
ESP32-S3 based Arducam B0462 GC2145 camera interface using ESP-IDF with patched GC2145 driver, SCCB communication, RGB565 SVGA capture, and web-based image preview.
## Description

This project demonstrates the interfacing of the Arducam B0462 GC2145 DVP camera module with an ESP32-S3 development board using ESP-IDF. The project includes a patched GC2145 camera driver, SCCB/I2C communication setup, RGB565 image capture, PSRAM-based frame buffering, and a simple web server for viewing captured images.

The system was developed by manually mapping the DVP camera signals to ESP32-S3 GPIO pins and debugging the camera interface step by step, including SCCB detection, XCLK generation, PCLK/HREF/VSYNC validation, frame capture, and D0–D7 data bus correction. The final setup captures SVGA images and displays them through a browser over Wi-Fi.
## Team Members

- Visakan , Anirudh Dhanunjay — ESP32-S3 integration, GC2145 driver patching, firmware development, and debugging, Camera hardware wiring, signal validation, testing, and documentation
