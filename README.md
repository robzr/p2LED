# p2LED
ATtiny85 APA102C controller with rotary incremental encoder UI

## Features
- APA102C control via SPI bus
- Interrupt driven queued rotary incremental encoder w/ switch controls
- (TODO) Sleep mode
- (TODO) Setup via UI with values saved to EEPROM
- (TODO) HSV solid mode

## Pinout
- PB0 -> LED clock 
- PB1 -> LED data
- PB2 -> Encoder0
- PB3 -> Encoder1
- PB4 -> Switch
- PB5 -> Tiny Serial out (debug); else EXT pin out to control external regulator

## UI
Description of UI modes & controls

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

