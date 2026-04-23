# Magician ROS 2 — Endüstriyel Robot Hücresi Kontrol Sistemi

Endüstriyel bir robot hücresini (PLC, sensing/cleaning cobotları, slider mekanizmaları) ROS 2 üzerinden yönetmek için geliştirilmiş bir otomasyon yazılım sistemidir. OPC UA protokolü ile PLC'ye bağlanır, Qt5 tabanlı operatör arayüzü sunar ve BehaviorTree tabanlı robot orkestrasyon mantığı içerir.

> Kurulum adımları için bkz. [INSTALLATION.md](INSTALLATION.md)

---

## İçindekiler

- [Sistem Mimarisi](#sistem-mimarisi)
- [Paketler](#paketler)
  - [backend — OPC UA ↔ ROS 2 Köprüsü](#backend--opc-ua--ros-2-köprüsü)
  - [gui_app — Operatör Arayüzü](#gui_app--operatör-arayüzü)
  - [demonstrator_tree — BehaviorTree Orkestrasyon](#demonstrator_tree--behaviortree-orkestrasyon)
- [Sistemi Çalıştırma](#sistemi-çalıştırma)
- [ROS 2 Arayüzleri](#ros-2-arayüzleri)
- [OPC UA Adres Alanı](#opc-ua-adres-alanı)
- [Kod Yapısı](#kod-yapısı)
- [Hata Ayıklama](#hata-ayıklama)

---

## Sistem Mimarisi

Sistem üç ROS 2 paketinden oluşur ve katmanlı bir mimari izler:

```
                        ┌───────────────────────────────────────────────────────────┐
                        │                  Operatör Katmanı                        │
                        │                                                           │
                        │  ┌─────────────────────────────────────────────────────┐  │
                        │  │              gui_app  (Qt5 GUI)                     │  │
                        │  │  Hız ayarı · Mod değiştirme · Robot kontrolleri    │  │
                        │  │  Slider konumları · Gerçek zamanlı durum izleme    │  │
                        │  └──────────────────────┬──────────────────────────────┘  │
                        │                         │ ROS 2 Service Çağrıları         │
                        │                         │ ROS 2 Topic Abonelikleri        │
                        └─────────────────────────┼────────────────────────────────┘
                                                  │
                        ┌─────────────────────────┼────────────────────────────────┐
                        │              Haberleşme Katmanı                          │
                        │                         │                                │
                        │  ┌──────────────────────▼──────────────────────────────┐  │
                        │  │           backend  (opc_bridge düğümü)             │  │
                        │  │                                                     │  │
                        │  │  ┌─────────────┐   ┌────────────┐   ┌───────────┐  │  │
                        │  │  │ UaClient    │   │ RosBridge  │   │ Config    │  │  │
                        │  │  │ (open62541) │◄─►│ (rclcpp)   │   │ (YAML)   │  │  │
                        │  │  └──────┬──────┘   └────────────┘   └───────────┘  │  │
                        │  │         │                                           │  │
                        │  └─────────┼───────────────────────────────────────────┘  │
                        │            │ OPC UA (TCP)                                 │
                        └────────────┼─────────────────────────────────────────────┘
                                     │
                        ┌────────────┼─────────────────────────────────────────────┐
                        │  Saha      │  Katmanı                                    │
                        │            ▼                                              │
                        │  ┌──────────────────┐                                    │
                        │  │   Siemens PLC     │ ← Gerçek üretim ortamı            │
                        │  │  (veya test_server│   veya localhost simülasyonu       │
                        │  │   simülasyonu)    │                                    │
                        │  └──────────────────┘                                    │
                        └──────────────────────────────────────────────────────────┘

                        ┌──────────────────────────────────────────────────────────┐
                        │               Orkestrasyon Katmanı                       │
                        │                                                          │
                        │  ┌────────────────────────────────────────────────────┐   │
                        │  │         demonstrator_tree  (BehaviorTree.CPP)      │   │
                        │  │                                                    │   │
                        │  │  Sequence                                          │   │
                        │  │  ├── Fallback                                      │   │
                        │  │  │   ├── IsRobotAtHome  → joint_states kontrol     │   │
                        │  │  │   └── CallHoming     → homing servis çağrısı    │   │
                        │  │  └── CallOpcUI          → PLC'ye safe-transfer     │   │
                        │  └────────────────────────────────────────────────────┘   │
                        │         │                              │                  │
                        │         │ /xbotcore/joint_states       │ /ros2_comm/      │
                        │         │ /xbotcore/homing/switch*     │ safetransfer_set │
                        │         ▼                              ▼                  │
                        │  ┌──────────────┐           ┌──────────────────┐          │
                        │  │ Robot        │           │ backend          │          │
                        │  │ Kontrolcüler │           │ (opc_bridge)     │          │
                        │  │ (xbotcore)   │           │                  │          │
                        │  └──────────────┘           └──────────────────┘          │
                        └──────────────────────────────────────────────────────────┘
```

### Veri Akışı

```
PLC ──(OPC UA subscription)──► UaClient ──(callback)──► RosBridge ──(publish)──► ROS 2 Topics ──► GUI / BT
# Magician ROS 2 OPC UA Control System

Magician is a ROS 2 workspace for supervising an industrial robot cell from ROS through OPC UA. It combines three packages:

- `backend` — ROS 2 ↔ OPC UA bridge
- `gui_app` — Qt5 operator panel
- `demonstrator_tree` — BehaviorTree-based orchestration logic

The project can run against a real PLC or against the built-in OPC UA test server for development.

For setup instructions, see [INSTALLATION.md](INSTALLATION.md).

---

## Overview

The workspace is organized around a simple control loop:

1. `backend` subscribes to PLC variables through OPC UA.
2. `backend` republishes them as ROS 2 topics.
3. The GUI and automation tree consume those topics.
4. GUI actions and BehaviorTree actions call ROS 2 services.
5. `backend` translates those service requests into OPC UA writes.

### High-level architecture

```text
PLC / OPC UA test server
        │
        ▼
backend (open62541 + rclcpp)
        ├── publishes ROS 2 topics
        └── exposes ROS 2 services
             │
             ├── gui_app (Qt5 operator interface)
             └── demonstrator_tree (BehaviorTree.CPP automation)
```

### Data flow

```text
PLC --OPC UA subscription--> UaClient --> RosBridge --> ROS 2 topics --> GUI / BT
GUI --ROS 2 service--------> RosBridge --> UaClient --> OPC UA write --> PLC
BT  --ROS 2 service--------> RosBridge --> UaClient --> OPC UA write --> PLC
```

---

## Workspace packages

### 1. `backend`

The `backend` package is the runtime core of the system.

It provides:

- OPC UA client connectivity through `open62541`
- asynchronous write queue handling
- ROS 2 publishers for PLC state
- ROS 2 services for PLC commands
- a local OPC UA simulation server for testing

#### Main components

| Component | Role |
|---|---|
| `UaClient` | Handles OPC UA connection, subscriptions, reconnect logic, and queued writes on a worker thread. |
| `RosBridge` | Exposes the ROS 2 API and forwards reads/writes between ROS 2 and OPC UA. |
| `ConfigLoader` | Loads bridge settings from YAML files. |
| `test_server` | Simulates the PLC address space locally on `opc.tcp://localhost:4840`. |

#### Current bridge features

- Int16 and Bool state bridging
- slider target and slider actual-position handling with floating-point values
- reconnect timing from YAML
- production/test configuration switching through a ROS parameter
- ROS 2 launch files for production and test modes

#### Configuration files

Two bridge configuration files are shipped in `backend/config/`:

| File | Purpose |
|---|---|
| `opcua.yaml` | Production configuration for the real PLC endpoint |
| `opcua_test.yaml` | Development configuration for the local test server |

Example:

```yaml
endpoint: "opc.tcp://192.168.1.1:4840"
namespace_index: 3

nodes:
  speed:   '"ROS2_COMM"."SPEED"'
  slider1: '"ROS2_COMM"."GO_TO_POS_1"'
  slider2: '"ROS2_COMM"."GO_TO_POS_2"'

structs:
  mod_root:      '"ROS2_COMM"."MOD"'
  stat_root:     '"ROS2_COMM"."STAT"'
  sensing_root:  '"ROS2_COMM"."STAT"."Robot_Sensing_Status"'
  cleaning_root: '"ROS2_COMM"."STAT"."Robot_Cleaning_Status"'
  Workcell:      '"ROS2_COMM"."MOD"."Workcell_Status"'

timing:
  sampling_ms: 50
  write_timeout_ms: 200
  reconnect: { initial_ms: 500, max_ms: 10000, multiplier: 2.0 }
```

### 2. `gui_app`

`gui_app` is a Qt5 Widgets-based operator interface.

It provides:

- speed input
- slider 1 / slider 2 target position input
- slider move triggers
- toggle controls for COBOT state
- toggle controls for sensing robot signals
- toggle controls for cleaning robot signals
- live UI updates from ROS 2 subscriptions

#### GUI behavior

- Each toggle button is connected to a ROS 2 `SetBool` service.
- Incoming topic updates refresh the toggle state and styling.
- Speed changes are sent through the custom `backend/srv/SetInt16` service.
- Slider position targets are sent through `backend/srv/SetFloat32`.

### 3. `demonstrator_tree`

`demonstrator_tree` runs a small BehaviorTree that checks whether both robots are at home, triggers homing when needed, and finally enables safe-transfer flags through the backend.

#### Tree logic

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="magician_sequence">
      <Fallback>
        <IsRobotAtHome name="check_home_pos"/>
        <CallHoming name="call_homing_service"/>
      </Fallback>
      <CallOpcUI name="call_opcua_service"/>
    </Sequence>
  </BehaviorTree>
</root>
```

#### Node roles

| Node | Role |
|---|---|
| `MagicianSubNode` | Subscribes to robot joint states and checks whether both robots match their configured home positions. |
| `MagicianClientNode` | Calls the configured homing services for the sensing and cleaning robots. |
| `MagicianOpcUA` | Calls `/ros2_comm/sensing/safetransfer_set` and `/ros2_comm/cleaning/safetransfer_set`. |

#### Current BehaviorTree config

The active `parameters.yaml` expects these external robot interfaces:

```yaml
cobot1:
  robot_name: "sensing_cobot"
  sensing_joint_states: "/sensing/joint_states"
  sensing_service: "/sensing/go_home"

cobot2:
  robot_name: "cleaning_cobot"
  cleaning_joint_states: "/cleaning/joint_states"
  cleaning_service: "/cleaning/go_home"
```

> Note: the executable currently loads `parameters.yaml` and `bt_tree.xml` through absolute paths inside the workspace, so the package is intended to run from this workspace layout as-is.

---

## ROS 2 interfaces

### Published topics

| Topic | Type | Description |
|---|---|---|
| `/ros2_comm/speed` | `std_msgs/msg/Int16` | Current speed value |
| `/ros2_comm/mod/cobot` | `std_msgs/msg/Bool` | COBOT mode flag |
| `/ros2_comm/sensing/home_st` | `std_msgs/msg/Bool` | Sensing robot safe-transfer/home status |
| `/ros2_comm/sensing/finished` | `std_msgs/msg/Bool` | Sensing finished |
| `/ros2_comm/sensing/touch_finished` | `std_msgs/msg/Bool` | Touch sensing finished |
| `/ros2_comm/sensing/sensing_active` | `std_msgs/msg/Bool` | Sensing active |
| `/ros2_comm/sensing/touch_active` | `std_msgs/msg/Bool` | Touch sensing active |
| `/ros2_comm/sensing/slide_command` | `std_msgs/msg/Bool` | Sensing slide command |
| `/ros2_comm/sensing/running` | `std_msgs/msg/Bool` | Sensing running |
| `/ros2_comm/sensing/slider_actual_pos` | `std_msgs/msg/Float32` | Sensing slider actual position |
| `/ros2_comm/cleaning/home_st` | `std_msgs/msg/Bool` | Cleaning robot safe-transfer/home status |
| `/ros2_comm/cleaning/finished` | `std_msgs/msg/Bool` | Cleaning finished |
| `/ros2_comm/cleaning/cleaning_active` | `std_msgs/msg/Bool` | Cleaning active |
| `/ros2_comm/cleaning/slide_command` | `std_msgs/msg/Bool` | Cleaning slide command |
| `/ros2_comm/cleaning/running` | `std_msgs/msg/Bool` | Cleaning running |
| `/ros2_comm/cleaning/slider_actual_pos` | `std_msgs/msg/Float32` | Cleaning slider actual position |

### Services

| Service | Type | Description |
|---|---|---|
| `/ros2_comm/speed_set` | `backend/srv/SetInt16` | Set speed |
| `/ros2_comm/mod/cobot_set` | `std_srvs/srv/SetBool` | Set COBOT mode |
| `/ros2_comm/sensing/safetransfer_set` | `std_srvs/srv/SetBool` | Set sensing safe-transfer |
| `/ros2_comm/sensing/finished_set` | `std_srvs/srv/SetBool` | Set sensing finished |
| `/ros2_comm/sensing/touch_finished_set` | `std_srvs/srv/SetBool` | Set touch-sensing finished |
| `/ros2_comm/sensing/active_set` | `std_srvs/srv/SetBool` | Set sensing active |
| `/ros2_comm/sensing/touch_active_set` | `std_srvs/srv/SetBool` | Set touch-sensing active |
| `/ros2_comm/sensing/slide_command_set` | `std_srvs/srv/SetBool` | Set sensing slide command |
| `/ros2_comm/sensing/running` | `std_srvs/srv/SetBool` | Set sensing running |
| `/ros2_comm/cleaning/safetransfer_set` | `std_srvs/srv/SetBool` | Set cleaning safe-transfer |
| `/ros2_comm/cleaning/cleaning_finished_set` | `std_srvs/srv/SetBool` | Set cleaning finished |
| `/ros2_comm/cleaning/cleaning_active_set` | `std_srvs/srv/SetBool` | Set cleaning active |
| `/ros2_comm/cleaning/slide_command_set` | `std_srvs/srv/SetBool` | Set cleaning slide command |
| `/ros2_comm/cleaning/running_set` | `std_srvs/srv/SetBool` | Set cleaning running |
| `/ros2_comm/slider1/set_pos` | `backend/srv/SetFloat32` | Set slider 1 target/actual position node |
| `/ros2_comm/slider1/go_pos` | `std_srvs/srv/SetBool` | Trigger slider 1 movement |
| `/ros2_comm/slider2/set_pos` | `backend/srv/SetFloat32` | Set slider 2 target/actual position node |
| `/ros2_comm/slider2/go_pos` | `std_srvs/srv/SetBool` | Trigger slider 2 movement |

### Custom service definitions

```text
backend/srv/SetInt16
int16 data
---
bool success
string message

backend/srv/SetFloat32
float32 data
---
bool success
string message
```

---

## OPC UA address space

The test server reproduces the PLC structure under namespace `ns=3`:

```text
Objects/
└── ROS2_COMM
    ├── STATUS                           Int16
    ├── MODE                             Int16
    ├── COMMAND                          Int16
    ├── SPEED                            Int16
    ├── GO_TO_POS_1                      Double
    ├── GO_TO_POS_2                      Double
    ├── MOD/
    │   ├── STARTUP                      Bool
    │   ├── CALIBRATION                  Bool
    │   ├── LEARNING                     Bool
    │   ├── MAINTENANCE                  Bool
    │   ├── EMERGENCY                    Bool
    │   ├── COBOT                        Bool
    │   ├── FULLY_AUTOMATIC              Bool
    │   ├── SHUTDOWN_MODE                Bool
    │   └── Workcell_Status/
    │       ├── Slider_1_actual position-linear   Double
    │       └── Slider_2_actual position-linear   Double
    └── STAT/
        ├── STARTUP                      Bool
        ├── CALIBRATION                  Bool
        ├── LEARNING                     Bool
        ├── MAINTENANCE                  Bool
        ├── EMERGENCY                    Bool
        ├── COBOT                        Bool
        ├── Robot_Sensing_Status/
        │   ├── robothome_safetransfer   Bool
        │   ├── sensing-finised          Bool
        │   ├── touchsensing-finished    Bool
        │   ├── sensing-active           Bool
        │   ├── touchsensing-active      Bool
        │   ├── slide command            Bool
        │   └── running                  Bool
        └── Robot_Cleaning_Status/
            ├── robothome_safetransfer   Bool
            ├── cleaning-finished        Bool
            ├── cleaning-active          Bool
            ├── slide command            Bool
            └── running                  Bool
```

---

## Build and run

### Build

```bash
cd ~/magician_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Production mode

Edit `backend/config/opcua.yaml` and set the real PLC endpoint, then run:

```bash
ros2 launch backend system.launch.py
```

Or run packages manually:

```bash
ros2 run backend opc_bridge
ros2 run gui_app gui_node
ros2 run demonstrator_tree demo
```

### Test mode

Backend only:

```bash
ros2 launch backend test_system.launch.py
```

Backend + GUI:

```bash
ros2 launch backend full_test_system.launch.py
```

Manual test setup:

```bash
ros2 run backend test_server
ros2 run backend opc_bridge --ros-args -p config:=opcua_test.yaml
ros2 run gui_app gui_node
```

### BehaviorTree test

If you want to test only the tree logic, provide matching joint-state publishers and homing services, or adapt `demonstrator_tree/config/parameters.yaml` to your own robot stack.

---

## Repository layout

```text
magician_ws/src/
├── backend/
│   ├── config/
│   ├── include/backend/
│   ├── launch/
│   ├── src/
│   ├── srv/
│   └── test/
├── demonstrator_tree/
│   ├── config/
│   ├── include/demonstrator_tree/
│   ├── src/
│   └── test/
├── gui_app/
│   ├── include/gui_app/
│   ├── png/
│   └── src/
├── INSTALLATION.md
└── README.md
```

---

## Debugging tips

```bash
# ROS 2 interfaces
ros2 topic list | grep ros2_comm
ros2 service list | grep ros2_comm

# Monitor live values
ros2 topic echo /ros2_comm/speed
ros2 topic echo /ros2_comm/sensing/home_st
ros2 topic echo /ros2_comm/cleaning/slider_actual_pos

# Manual service calls
ros2 service call /ros2_comm/speed_set backend/srv/SetInt16 "{data: 500}"
ros2 service call /ros2_comm/mod/cobot_set std_srvs/srv/SetBool "{data: true}"
ros2 service call /ros2_comm/sensing/active_set std_srvs/srv/SetBool "{data: true}"
ros2 service call /ros2_comm/slider1/set_pos backend/srv/SetFloat32 "{data: 150.0}"
ros2 service call /ros2_comm/slider1/go_pos std_srvs/srv/SetBool "{data: true}"
```

---

## Requirements

| Component | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| C++ | 17 |
| Qt | Qt5 |
| OPC UA library | open62541 >= 1.4 |
| BehaviorTree | `ros-humble-behaviortree-cpp` |
| YAML | `libyaml-cpp-dev` |

For full installation steps, see [INSTALLATION.md](INSTALLATION.md).

---

## Maintainer

frknbyrm05@gmail.com
