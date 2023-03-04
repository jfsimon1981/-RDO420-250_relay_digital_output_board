# Firmware for relay board RDO420-250 RDO420-250

RCO 8/80-28-MCP is a modular Relay Digital Output board
with ATTiny461 20-pin Atmel microcontroller.

More info on product page [LCElectronics/RDO420-250](http://lecomptoirelectronique.fr/store/index.php?route=product/product&path=66&product_id=65).

The board has 4 auxiliary relays with 2 3-terminal contacts per relay,
Digital input and resial input ports, local command push-buttons.

It can drive auxiliary and small power devices.

Please see product page and ressources for more informations.

## Operations

The board

  - Mode 1 Momentary push-button toggle
  - Mode 2 Copy push-button
  - Mode 3 Timer operation

Configuration is done via I2C serial bus or by push-buttons

  - Set mode by I2C and time-out by I2C for individual relays
  - Set mode by switches: keep SW1, SW2 or SW3 pressed and reset board for mode 1, 2, 3 respectively (note 1)
  - If mode 3 is selected, release SW3 and press again any of SW1 to SW4
to change their timeout. Press again to select time-out common to all relays: (note 2)
    - SW1 for 1s pulse
    - SW2 for 30s pulse
    - SW3 for 3 minutes pulse
    - SW4 for 5 minutes pulse

Note 1: Configuration will persist across next reset/powerup.

Note 2: I2C allows fine grain configuration mode and time-out settings per relay

## Ressources

LCElectronics portal
  - [Home](http://lecomptoirelectronique.fr/store)
  - [Product page RDO420-250](http://lecomptoirelectronique.fr/store/index.php?route=product/product&path=66&product_id=65)