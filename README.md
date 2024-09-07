# esp32-wifi-hal-rs
This repo contains an experimental port of esp32-open-mac to Rust, with embassy. It is not intended to replace the C version, but to explore, how this can be done in Rust idiomatically. We still rely on the proprietary blobs for initializing the RF frontend and some more minor initialization. 
## Usage
The crate is a library and there are example binaries in `src/bin/`. The only public API is the `WiFi` struct, which provides all currently implemented functioniality. Currently the RX policy is fixed to three, which means only beacons will be received.
## Building
To set up a development environment follow the guide at https://docs.esp-rs.org/book/installation/index.html. Since this only works on the ESP32 right now, only the Xtensa section is of interest.
To try one of these examples:
1. Clone the repo
2. Connect the ESP32
3. Run `cargo run -r --bin [EXAMPLE_NAME_GOES_HERE]`
## DISCLAIMER
This is experimental software. USE AT YOUR OWN RISK! We'll not take any liability for damage to the hardware. We do not condone the use of this for malicious purposes.
