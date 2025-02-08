# 2025 FIRST REEFSCAPE

[![CI](https://github.com/strykeforce/reefscape/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/reefscape/actions/workflows/main.yml)

## Controls

### Driver Controller
![flysky](docs/driver-controls.png)

### Operator Controller
![operator](docs/operator-controls.png)

## State Diagrams

### Robot State
![robotState](docs/reefscape-RobotState-Light.png)


## CAN Bus

| Subsystem | Type     | Talon                     | ID  | CAN BUS | Comp PDP | Proto PDP | Motor  | Breaker |
| --------- | -------- | ------------------------- | --- | ------- | -------- | --------- | ------ | ------- |
| Drive     | FXS      | azimuth                   | 0   | FD      |          | 16        | Minion |         |
| Drive     | FXS      | azimuth                   | 1   | FD      |          | 17        | Minion |         |
| Drive     | FXS      | azimuth                   | 2   | FD      |          | 6         | Minion |         |
| Drive     | FXS      | azimuth                   | 3   | FD      |          | 7         | Minion |         |
| Drive     | FX       | drive                     | 10  | FD      |          | 23        | kraken |         |
| Drive     | FX       | drive                     | 11  | FD      |          | 22        | kraken |         |
| Drive     | FX       | drive                     | 12  | FD      |          | 0         | kraken |         |
| Drive     | FX       | drive                     | 13  | FD      |          | 1         | kraken |         |
| Elevator  | FX       | elevatorMain              | 20  | rio     |          | 21        | kraken |         |
| Elevator  | FX       | elevatorFollow            | 21  | rio     |          | 20        | kraken |         |
| Biscuit   | FXS      | biscuit                   | 25  | rio     |          | 3         | Minion |         |
| Algae     | FXS      | algae                     | 30  | rio     |          |           | Minion |         |
| Coral     | FXS      | coral                     | 35  | rio     |          | 2         | Minion |         |
| Funnel    | FXS      | rollers                   | 40  | rio     |          |           | Minion |         |
| Climb     | FX       | frontMain                 | 45  | rio     |          |           | Minion |         |
| Climb     | FX       | backFollow                | 46  | rio     |          |           | kraken |         |
| Climb     | CANcoder | CANCoder                  | 47  | ri0     |          |           | n/a    |         |
| -         | -        | rio                       | -   | both    |          | 12        |        |         |
| -         | -        | vrm (radio, pigeon)       | -   | -       |          | 13        |        |         |
| -         | -        | custom circuit (pi power) | -   | -       |          |           |        |         |

## VRM
| Device          | Voltage | Current | ID  |
| --------------- | ------- | ------- | --- |
| Pigeon 2        | 12      | 0.5     | 4   |
| Ethernet Switch | 12      | 2       |     |
|                 |         |         |     |

## Beam Breaks
| Subsystem | Talon   | ID  | Fwd/Rev | Purpose            |
| --------- | ------- | --- | ------- | ------------------ |
| Funnel    | rollers | 40  |         | Coral Presence     |
| Coral     | wheels  | 30  |         | Coral partially in |
| Coral     | wheels  | 30  |         | Coral fully in     |


## Roborio
| Subsystem | Interface | Device   |
| --------- | --------- | -------- |
| n/a       | USB       | CANivore |

## DIO
| Subsystem  | name      | ID  |
| ---------- | --------- | --- |
| BattMon    | Batt V    | 0   |
| BattMon    | Batt I    | 1   |
| BattMon    | PDP V     | 2   |
| BattMon    | Breaker T | 3   |
| AutoSwitch | switch    | 4   |
| AutoSwitch | switch    | 5   |
| AutoSwitch | switch    | 6   |
| AutoSwitch | switch    | 7   |
| AutoSwitch | switch    | 8   |
| AutoSwitch | switch    | 9   |

## MXP
| Subsystem | name        | ID  |
| --------- | ----------- | --- |
| TagServo  | wallSense   | 10  |
| Climb     | cageAligned | 11  |
|           |             | 12  |
|           |             | 13  |
|           |             | 14  |
|           |             | 15  |
|           |             | 16  |
|           |             | 17  |
|           |             | 18  |
|           |             | 19  |
|           |             | 20  |
|           |             | 21  |
|           |             | 22  |
|           |             | 23  |
|           |             | 24  |
|           |             | 25  |


## PWM
| Subsystem | name         | ID  |
| --------- | ------------ | --- |
| LED       | lights       | 0   |
| Climb     | deployServo  | 1   |
| Climb     | ratchetServo | 2   |
|           |              | 3   |
|           |              | 4   |
|           |              | 5   |
|           |              | 6   |
|           |              | 7   |
|           |              | 8   |
|           |              | 9   |    

## Analog
| Subsystem | name   | ID  |
| --------- | ------ | --- |
| Elevator  | height | 0   |
|           |        | 1   |
|           |        | 2   |
|           |        | 3   |
|           |        | 4   |
|           |        | 5   |
|           |        | 6   |
|           |        | 7   |
|           |        | 8   |
|           |        | 9   |

## Cameras
| Camera      | IP Address  | Type    |
| ----------- | ----------- | ------- |
| Left Servo  | 10.27.67.XX | USB 3.0 |
| Right Servo | 10.27.67.YY | USB 3.0 |
| Upper Left  | 10.27.67.XX | USB 2.0 |
| Upper Right | 10.27.67.YY | USB 2.0 |
| Rear        | 10.27.67.ZZ | USB 2.0 |
