# 2025 FIRST REEFSCAPE

[![CI](https://github.com/strykeforce/reefscape/actions/workflows/main.yml/badge.svg)](https://github.com/strykeforce/reefscape/actions/workflows/main.yml)

## Controls

### Driver Controller


### Operator Controller


## CAN Bus

| Subsystem | Type     | Talon                     | ID  | CAN BUS | Comp PDP | Proto PDP | Motor | Breaker |
| --------- | -------- | ------------------------- | --- | ------- | -------- | --------- | ----- | ------- |
| Drive     | FXS      | azimuth                   | 0   | FD      |          | Minion    |       |         |
| Drive     | FXS      | azimuth                   | 1   | FD      |          | Minion    |       |         |
| Drive     | FXS      | azimuth                   | 2   | FD      |          | Minion    |       |         |
| Drive     | FXS      | azimuth                   | 3   | FD      |          | Minion    |       |         |
| Drive     | FX       | drive                     | 10  | FD      |          | kraken    |       |         |
| Drive     | FX       | drive                     | 11  | FD      |          | kraken    |       |         |
| Drive     | FX       | drive                     | 12  | FD      |          | kraken    |       |         |
| Drive     | FX       | drive                     | 13  | FD      |          | kraken    |       |         |
| Elevator  | FX       | elevatorMain              | 20  | rio     |          | kraken    |       |         |
| Elevator  | FX       | elevatorFollow            | 21  | rio     |          | kraken    |       |         |
| Biscuit   | FXS      | biscuit                   | 25  | rio     |          | minion    |       |         |
| Algae     | FXS      | rollers                   | 30  | rio     |          | minion    |       |         |
| Coral     | FXS      | wheels                    | 35  | rio     |          | minion    |       |         |
| Funnel    | FXS      | rollers                   | 40  | rio     |          | minion    |       |         |
| Climb     | FX       | rollers                   | 45  | rio     |          |           |       |         |
| Climb     | FX       | pivot                     | 46  | rio     |          | kraken    |       |         |
| Climb     | CANcoder | CANCoder                  | 47  | ri0     |          |           |       |         |
| -         | -        | rio                       | -   | both    |          |           |       |         |
| -         | -        | radio                     | -   | -       |          |           |       |         |
| -         | -        | custom circuit (pi power) | -   | -       |          |           |       |         |

## VRM
| Device          | Voltage | Current |
| --------------- | ------- | ------- |
| Pigeon 2        | 12      | 2       |
| Ethernet Switch | 12      | 2       |
|                 |         |         |

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
| Subsystem | name      | ID  |
| --------- | --------- | --- |
| TagServo  | wallSense | 10  |
|           |           | 11  |
|           |           | 12  |
|           |           | 13  |
|           |           | 14  |
|           |           | 15  |
|           |           | 16  |
|           |           | 17  |
|           |           | 18  |
|           |           | 19  |
|           |           | 20  |
|           |           | 21  |
|           |           | 22  |
|           |           | 23  |
|           |           | 24  |
|           |           | 25  |


## PWM
| Subsystem | name         | ID  |
| --------- | ------------ | --- |
|           |              | 0   |
|           |              | 1   |
|           |              | 2   |
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