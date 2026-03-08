# Rocket Flight Software (Collegiate Rocketry Team)
For more info on my electrical/mechanical hardware contributions / the rocket itself, look [here](https://sites.google.com/view/mc-aero/rockets?authuser=0):
<img width="361" height="212" alt="image" src="https://github.com/user-attachments/assets/645ccd17-9769-43ae-8802-84777ea8f750" />
![Uploading image.png…]()



Flight software developed for a high-power sounding rocket as part of my collegiate rocketry team. The software runs on a custom avionics stack and manages sensor processing, flight phase detection, and recovery system deployment in real time.

---

# Overview

The flight computer runs a real-time C++ system responsible for:

- Determining the current flight phase
- Processing data from multiple onboard sensors
- Executing recovery system events
- Maintaining fault tolerance through sensor redundancy
- Running deterministic real-time tasks with FreeRTOS

The system is designed to operate reliably in a high dynamic environment including launch acceleration, high vibration, and high altitude conditions.

---

# Flight State Machine

<img width="638" height="446" alt="Flight State Machine" src="https://github.com/user-attachments/assets/41feb677-8053-46e2-9cc3-45c362c72428" />

The core control logic is implemented as a flight state machine that transitions through each phase of the rocket’s flight.

### Flight Phases

1. Pre-launch initialization  
2. Launch detection  
3. Boost phase  
4. Coast phase  
5. Apogee detection  
6. Drogue parachute deployment  
7. Main parachute deployment  
8. Landing detection  

Transitions between states are triggered using sensor-based event detection.

---

# Sensor-Based Event Detection

Multiple sensors are used to determine the rocket’s state and detect key flight events.

### Sensors Used

- **IMU**  
  Used for acceleration detection during launch and for motion estimation.

- **Barometric Altimeters**  
  Used to determine altitude and detect apogee.

- **GPS**  
  Used for position tracking and recovery operations.

### Event Detection Examples

| Event | Detection Method |
|------|------|
| Launch | Sustained acceleration threshold |
| Apogee | Peak altitude detection |
| Drogue deployment | Apogee confirmation |
| Main deployment | Altitude threshold |
| Landing | Near-zero velocity and altitude stabilization |

Filtering and threshold logic are used to prevent false triggers caused by noise or transient sensor spikes.

---

# Sensor Redundancy and Fault Tolerance

Reliability is critical for flight systems.

The avionics system uses redundant sensors and validation logic to reduce the probability of false deployment events.

Redundancy strategies include:

- Multiple barometric sensors + GPS for altitude
- Cross validation between sensors
- Outlier rejection and sanity checks
- Conservative event thresholds

This architecture ensures that a sensor failures cannot trigger incorrect recovery events.

---

# Real-Time Architecture (FreeRTOS)

The software runs on **FreeRTOS**, enabling deterministic scheduling of critical tasks.

### Key Tasks

| Task | Responsibility |
|-----|-----|
| Sensor Task | Poll IMU, barometers, and GPS |
| State Machine Task | Evaluate flight state transitions |
| Logging Task | Record telemetry data |
| Recovery Task | Control parachute deployment hardware |

FreeRTOS ensures predictable timing and prioritization of safety-critical operations such as recovery deployment.

---

# Implementation

- **Language:** C++  
- **Framework:** PlatformIO  
- **RTOS:** FreeRTOS  
- **Target Hardware:** Custom rocket flight computer


# Custom Flight Computer

This software runs on a custom avionics board designed specifically for the rocket.

The flight computer was co-designed with  
Hou Dren Yuen  
https://www.linkedin.com/in/hou-dren-yuen-3586122b1/

The hardware integrates:

- Microcontroller
- IMU
- Barometric sensors
- GPS
- Telemetry

---

# Skills 

- Embedded systems development  
- Real-time operating systems (FreeRTOS)  
- C++ firmware architecture  
- Flight state machine design  
- Sensor integration  
- Aerospace avionics software
