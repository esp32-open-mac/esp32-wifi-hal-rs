# esp32-wifi-hal-rs
This repo contains an experimental port of esp32-open-mac to Rust, with embassy. It is not intended to replace the C version, but to explore, how this can be done in Rust idiomatically. We still rely on the proprietary blobs for initializing the RF frontend and some more minor initialization. 
## Usage
The crate is a library and there are example binaries in `src/bin/`. The only public API is the `WiFi` struct, which provides all currently implemented functioniality. Currently the RX policy is fixed to three, which means only beacons will be received.
## DISCLAIMER
This is experimental software. USE AT YOUR OWN RISK! We'll not take any liability for damage to the hardware. We do not condone the use of this for malicious purposes.
