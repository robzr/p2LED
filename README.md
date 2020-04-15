# p2LED
Digispark / ATtiny85 multi-mode LED strip controller with rotary encoder switch control

## What is it?
A high performance, efficient, menu driven, addressable LED controller that uses the dirt cheap Digispark board (available for $1.50 shipped on eBay). The menu system is based on input from a rotary encoder with a switch. One remaining pin can be used to wake/sleep the unit, which is useful for things like cabinet lighting. With low quiescent current requirements, this controller is usable in battery driven applications like Camper Vans.

## Features
- APA102C/SK9822 LED strip control via SPI bus
- HSV interface and effects via FastLED library
- Interrupt driven, queued event driven handling of rotary incremental encoder + switch control
- Logarithmic dampened linear encoder acceleration algorithm for smooth and quick rotary input
- Supports at least 144 LED (high density 1m) strips in all modes
- Multi-mode - switchable and tunable solid, rainbow, breathing effects
- Holding the switch while running will save the current mode as startup mode, and the current mode settings to EEPROM
- Holding the switch on boot enters setup mode, used to change the LED strip length
- Holding the switch on boot for a real long time resets the EEPROM
- (TODO) Optional external switch mode to use with cabinet switch input
- (TODO) Include micronucleus image for instant boot with V-USB mode on grounded pin

## Pinout
- PB0 -> APA102 clock 
- PB1 -> APA102 data
- PB2 -> Encoder CLK (to ground)
- PB3 -> Encoder DT (follows CLK by 90 degrees) (to ground)
- PB4 -> Encoder Switch (to ground)
- PB5 -> (Optional, to ground) external switch for instant wake/sleep

## UI
Description of UI modes & controls

### Switch Timing
| Type             | Threshold  | Description | 
|------------------|------------|-------------|
| Short Press      | 0ms        | Typical operations, menu navigation, confirmation |
| Long Press       | 750ms      | Power off, escape to previous menu |
| Super Long Press | 2000ms     | "Destructive" operations, save mode & settings to EEPROM; enter setup... |
| Mega Long Press  | 5000ms     | "Mega Destructive" operation; reset EEPROM |

### Modes
| Mode        | Control     | Action |
|-------------|-------------|--------|
| Off | Short Press | On |
|     | Super Long Press  | Setup Menu |
|     | Mega Long Press  | Reset EEPROM |
| Setup | Rotate            | Set strip length |
|       | Super Long Press  | Save setting & restart |
| On | Rotate             | adjust current setting |
|    | Short Press        | next setting |
|    | Double Short Press | next mode |
|    | Long Press       | Off |
|    | Super Long Press | Save current settings as default |

### Effect Modes
| Effect | Submenu | Control | Action |
|--------|---------|---------|--------|
| Solid  | | Rotate      | Adjust brightness |
|        | | Short Press | Enter submenu 1   |
|        | submenu 1 | Rotate      | Adjust Hue      |
|        | submenu 1 | Short Press | Enter submenu 2 |
|        | submenu 1 | Long Press  | Main            |
|        | submenu 1 | Timeout     | Main            |
|        | submenu 2 | Rotate      | Adjust Saturation |
|        | submenu 2 | Short Press | Main              |
|        | submenu 2 | Long Press  | Main              |
|        | submenu 1 | Timeout     | Main              |
| Wheel  | | Rotate      | Adjust brightness |
|        | | Short Press | Enter submenu 1   |
|        | submenu 1 | Rotate      | Adjust Speed |
|        | submenu 1 | Long Press  | Main         |
|        | submenu 1 | Timeout     | Main         |

