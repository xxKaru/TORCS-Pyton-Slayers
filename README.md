# TORCS Pyton Slayers

This repository contains our TORCS autonomous racing driver for the IBM Global AI Racing League.

The main driver code is located in:

- `gym_torcs/torcs_jm_par.py`

The driver controls the car using live telemetry from TORCS. It reads values such as track sensors, speed, angle, track position, lateral speed and RPM, then uses them to decide steering, braking, acceleration and gear changes.

## Code Overview

The driver is built around several main parts:

### Turn Detection

The code checks the track sensor values to detect whether the car is approaching a left turn, right turn or straight section.

To avoid reacting to random sensor noise, the detected direction must be confirmed for several simulation steps before it becomes the active turn.

### Corner Stages

The code divides driving into different stages, such as:

- straight
- prepare
- entry
- apex
- exit

Each stage changes how the car positions itself and how strongly it steers. This helps the driver prepare before a corner, enter the corner, pass through the apex and return toward a better position on exit.

### Steering

Steering is calculated from the current track position, car angle, track sensors and the detected corner stage.

The driver tries to keep the car stable while still taking corners aggressively enough to maintain speed.

### Braking

The braking logic estimates whether the current speed is safe for the available track distance ahead.

It uses the front track sensors, current speed and an estimated safe speed. If the required braking distance is too large, the driver applies braking.

### Throttle

Throttle is controlled together with steering and braking.

The driver accelerates strongly on straights, but reduces acceleration when braking is active or when steering demand is high.

### Gear Shifting

Gear changes are based on RPM thresholds. The driver shifts up when RPM is high enough and shifts down when RPM becomes too low.

### Recovery Logic

The code also includes recovery behavior for unstable situations.

Anti-spin recovery helps when the car has too much lateral movement. Border recovery helps when the car moves too far from the center of the track.

## Final Driver

The final driver is a telemetry-based controller tuned through repeated testing on the Corkscrew track.
