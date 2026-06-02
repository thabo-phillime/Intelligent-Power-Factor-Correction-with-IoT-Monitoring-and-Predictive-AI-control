# Intelligent Power Factor Correction (IPFC) System with Edge AI & Blynk IoT Monitoring

This repository houses the complete end-to-end research, multi-platform firmware, datasets, and analytical media for an Intelligent Power Factor Correction (IPFC) system. This project was developed as my Final Year Engineering Capstone at the Botswana International University of Science and Technology (BIUST) under the academic supervision of Dr. Mangwala.

## Project Abstract & Core Engineering Pivots
Traditional power factor correction systems rely on static, reactive relay switching thresholds that cannot adapt efficiently to dynamic, non-linear industrial load profiles. This design introduces a predictive edge-computing alternative. 

By leveraging an ESP32 microcontroller running an embedded Random Forest Regressor model, the system predictively determines exact reactive power compensation requirements to stabilize the system power factor toward unity (1.00) under variable loads.

### Co-Simulation & Hardware Architecture
Due to distinct software modeling constraints and real-world physical requirements, a multi-tier design framework was deployed:
1. **Analog Subsystem Simulation (Proteus VSM):** Because the ESP32 cannot be modeled natively in Proteus, a **PIC16 microcontroller** was utilized to simulate analog data acquisition, alternating current signal conditioning (230Vrms supply), zero-crossing detection, and phase-detection waveforms.
2. **Intelligent Digital Edge Control (Wokwi):** Simulates the core digital architecture, testing the firmware execution of the ESP32 processing data inputs and dictating optimal reactive compensation commands.
3. **Physical Hardware Prototype (Breadboard Assembly):** The final physical system was deployed on a robust bench prototype, utilizing an ESP32 microcontroller to execute edge calculations while communicating telemetry over Wi-Fi to a custom **Blynk IoT Cloud Dashboard** for real-time remote monitoring.

---

## Repository Directory Structure

```text
├── documentation/
│   └── BIUST_Final_Year_Project_Report.pdf       # Complete academic capstone thesis
│
├── ml_model/
│   ├── Power_Factor_Model.ipynb                  # Google Colab notebook (Data preprocessing & training)
│   └── industrial_load_dataset.csv               # Historical load log dataset from bench testing
│
├── hardware/
│   ├── IPFC_Analog_Circuit.pdsprj                # Proteus analog schematic & routing simulation file
│   ├── PIC16_Analog_Conditioning.hex             # Compiled hardware binary for the Proteus PIC16 controller
│   ├── ESP32_Wokwi_Firmware.ino                  # Embedded firmware optimized for Wokwi testing
│   ├── Random_Forest_Model.h                     # Exported C++ Edge AI model weights/parameters 
│   └── ESP32_Physical_Hardware_Firmware.ino      # Production firmware for bench hardware & Blynk IoT
│
└── media/                                        # 19 Chronological project timeline images (Phases 1-3)
```

## Interactive Deployment Links
- **Live Digital & Edge AI Simulation:** Access and interact with the firmware execution and ESP32 microcontroller operations directly via the web portal: [Live Wokwi Workspace Simulation](https://wokwi.com/projects/465710383710910465)

---

## Technical Competencies Demonstrated

### 1. Advanced System Simulation & Co-Design
* **Dual-Controller Simulation Pipeline:** The project required simulating two different microcontrollers, but no single tool could handle both. To work around this, a **PIC16 microcontroller** was modeled in Proteus to manage the high-voltage side of the system, including reading a 230Vrms input signal, detecting zero-crossings, and monitoring phase waveforms. At the same time, Wokwi was used separately to write and test the **ESP32** firmware that controls the core logic. Both environments were run in parallel to cover the full system.
* **Predictive Machine Learning Integration:** A **Random Forest Regressor** model was built and trained in Python using Google Colab. The training data was carefully analyzed to identify the most useful features, such as PF_x1000. To achieve true edge computing, the trained model was compiled and exported as a custom C++ header file (.h). This file was integrated directly into the Wokwi and physical ESP32 environments, allowing the microcontroller to execute AI inferences locally on-chip.

### 2. Physical Hardware Prototyping & Field Integration
* **Robust Breadboard Bench Prototyping:** After simulation, the system was built as a working physical prototype on a breadboard. The **ESP32 microcontroller** was wired up and tested with real sensors, resistive and inductive test loads, status indicator LEDs, and relay-controlled capacitor banks, bringing the design out of software and into a real, measurable circuit.
* **Full-Stack IoT Telemetry Engine:** A live data monitoring pipeline was set up using the **Blynk IoT platform**. The ESP32 was configured to send real-time readings, including power factor values, load changes, and relay switching times, over Wi-Fi to a custom cloud dashboard. This allowed the system's performance to be tracked and observed remotely without being physically present at the bench.
