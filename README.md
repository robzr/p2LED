# p2LED
Digispark / ATtiny85 multi-mode LED strip controller with rotary encoder switch control

## Features
- APA102C control via SPI bus
- Interrupt driven, queued event driven handling of rotary incremental encoder + switch control
- Supports at least 144 LED (high density 1m) strips in all modes
- Multi-mode - adjustable solid, as well as effects
- (TODO) Setup via UI with values saved to EEPROM
- (TODO) Optional external switch mode to use with cabinet switch input
- (TODO) HSV solid mode

## Pinout
- PB0 -> APA102 clock 
- PB1 -> APA102 data
- PB2 -> Encoder CLK (to ground)
- PB3 -> Encoder DT (follows CLK by 90 degrees) (to ground)
- PB4 -> Encoder Switch (to ground)
- PB5 -> (Optional, to ground) external switch for instant wake/sleep

## UI
Description of UI modes & controls

### Button Timing
- 0ms - 750ms    -> Short Press
- 750ms - 1500ms -> Long Press
- 1500ms+        -> Super Long Press

### Modes
| Mode        | Control     | Action |
|-------------|-------------|--------|
| Off | Short Press | On |
|     | Long Press  | Setup Menu |
| Setup | Rotate      | Set strip length |
|       | Short Press | Setup 2 menu |
|       | Long Press  | Save to EEPROM |
| On | Rotate           | effect adjustment 1 |
|    | Short Press      | effect submenu 1 |
|    | 2x Short Press   | Select effect submenu |
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

