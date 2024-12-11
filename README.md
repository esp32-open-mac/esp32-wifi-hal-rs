# esp32-wifi-hal-rs
This repo contains an experimental port of esp32-open-mac to Rust, with embassy. It is not intended to replace the C version, but to explore, how this can be done in Rust idiomatically. We still rely on the proprietary blobs for initializing the RF frontend and some more minor initialization. 
## DISCLAIMER
This is experimental software. USE AT YOUR OWN RISK! We'll not take any liability for damage to the hardware. We do not condone the use of this for malicious purposes.
## Usage
The actual crate lives in `esp32-wifi-hal-rs/` and examples are in `examples/`.

For further information see the docs.
## Building
To set up a development environment follow the guide at https://docs.esp-rs.org/book/installation/index.html. Since this only works on the ESP32 right now, only the Xtensa section is of interest.
To try one of these examples:
1. Clone the repo
2. Connect the ESP32
3. `cd examples`
4. Run `cargo run -r --bin [EXAMPLE_NAME_GOES_HERE]`
## Technical Notes
The ESP32 WiFi peripheral has five TX slots, which we number 0-4. The MMIO addresses, where these are configured are in reverse order. This means, that slot zero starts at the HIGHEST address and slot four at the lowest. This numbering is also suggested by the TX status registers. We could in theory reverse this ordering to ascending addresses, this would however cause headaches with TX slot status handling, so we chose to stick with descending addresses. This is also the way the proprietary task handles this.

We have reason to believe, that slot four is in some way special, as the TX completion handler in the proprietary task masks away the bit corresponding to that slot (See `hal_mac_get_txq_state`). However, as our tests do not indicate any special behaviour, we just use it like a normal slot.

