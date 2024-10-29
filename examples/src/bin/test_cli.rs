#![no_std]
#![no_main]
use core::{fmt::Write, iter::repeat, marker::PhantomData, mem::MaybeUninit, str};

use alloc::{
    collections::btree_set::BTreeSet,
    string::{String, ToString},
};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker};
use esp32_wifi_hal_rs::{DMAResources, RxFilterBank, RxFilterInterface, WiFi, WiFiRate};
use esp_backtrace as _;
use esp_hal::{
    efuse::Efuse,
    gpio::Io,
    peripherals::UART0,
    timer::timg::TimerGroup,
    uart::{config::Config, Uart, UartRx, UartTx},
    Async,
};
use ieee80211::{
    common::{CapabilitiesInformation, TU},
    element_chain,
    elements::{
        tim::{StaticBitmap, TIMBitmap, TIMElement},
        DSSSParameterSetElement, SSIDElement,
    },
    mac_parser::{MACAddress, BROADCAST},
    match_frames,
    mgmt_frame::{body::BeaconBody, BeaconFrame, ManagementFrameHeader},
    scroll::Pwrite,
    supported_rates, GenericFrame,
};
use log::LevelFilter;
extern crate alloc;
use esp_alloc as _;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

const QUIT_SIGNAL: u8 = b'q';

async fn wait_for_quit(uart_rx: &mut UartRx<'_, UART0, Async>) {
    loop {
        let mut buf = [0x0u8];
        let _ = uart_rx.read_async(buf.as_mut_slice()).await;
        if buf[0] == QUIT_SIGNAL {
            break;
        }
    }
}
fn channel_cmd<'a>(
    wifi: &mut WiFi,
    uart0_tx: &mut UartTx<'_, UART0, Async>,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    match args.next() {
        Some("set") => {
            let Some(channel_number) = args.next() else {
                let _ = writeln!(uart0_tx, "Missing argument channel number.");
                return;
            };
            let Ok(channel_number) = channel_number.trim().parse() else {
                let _ = writeln!(uart0_tx, "Argument channel number wasn't a number.");
                return;
            };
            if wifi.set_channel(channel_number).is_ok() {
                let _ = writeln!(
                    uart0_tx,
                    "Successfully switched to channel {channel_number}"
                );
            } else {
                let _ = writeln!(uart0_tx, "Invalid channel {channel_number}.");
            }
        }
        Some("get") => {
            let _ = writeln!(uart0_tx, "Current channel: {}", wifi.get_channel());
        }
        None => {
            let _ = writeln!(uart0_tx, "Missing operand. Valid operands are: set,get");
        }
        Some(operand) => {
            let _ = writeln!(
                uart0_tx,
                "Unknown operand: {operand}, valid operands are: set,get"
            );
        }
    }
}
async fn scan_on_channel(
    wifi: &mut WiFi,
    uart0_tx: &mut UartTx<'_, UART0, Async>,
    known_aps: &mut BTreeSet<String>,
) {
    loop {
        let received = wifi.receive().await;
        let mpdu_buffer = received.mpdu_buffer();
        let _ = match_frames! {
            mpdu_buffer,
            beacon_frame = BeaconFrame => {
                let Some(ssid) = beacon_frame.ssid() else {
                    continue;
                };
                if ssid.trim().is_empty() {
                    continue;
                }
                if !known_aps.contains(ssid) {
                    known_aps.insert(ssid.to_string());
                    let _ = writeln!(uart0_tx, "Found AP with SSID: {ssid} on channel: {}", wifi.get_channel());
                }
            }
        };
    }
}
async fn scan_command<'a>(
    wifi: &mut WiFi,
    uart0_tx: &mut UartTx<'_, UART0, Async>,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    wifi.set_filter_status(RxFilterBank::BSSID, RxFilterInterface::Zero, false);
    wifi.set_filter_status(
        RxFilterBank::ReceiverAddress,
        RxFilterInterface::Zero,
        false,
    );
    wifi.set_scanning_mode(RxFilterInterface::Zero, true);
    let mut known_aps = BTreeSet::new();
    let mode = args.next();
    match mode {
        Some("hop") => {
            let mut hop_interval = Ticker::every(Duration::from_millis(200));
            let mut hop_sequence = repeat(1..13).flatten();
            loop {
                if let Either::First(_) = select(
                    hop_interval.next(),
                    scan_on_channel(wifi, uart0_tx, &mut known_aps),
                )
                .await
                {
                    let _ = wifi.set_channel(hop_sequence.next().unwrap());
                }
            }
        }
        None => scan_on_channel(wifi, uart0_tx, &mut known_aps).await,
        Some(operand) => {
            let _ = writeln!(
                uart0_tx,
                "Unknown operand: {operand}, valid operands are: hop"
            );
        }
    }
}
async fn beacon_command<'a>(
    wifi: &mut WiFi,
    uart0_tx: &mut UartTx<'_, UART0, Async>,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    let Some(ssid) = args.next() else {
        let _ = writeln!(uart0_tx, "Missing SSID.");
        return;
    };
    let channel_number = if let Some(channel_number) = args.next() {
        let Ok(channel_number) = channel_number.parse() else {
            let _ = writeln!(uart0_tx, "Channel number wasn't a number.");
            return;
        };
        if wifi.set_channel(channel_number).is_err() {
            let _ = writeln!(uart0_tx, "Invalid channel number {channel_number}");
            return;
        }
        channel_number
    } else {
        wifi.get_channel()
    };
    let _ = writeln!(
        uart0_tx,
        "Transmitting beacons with SSID: {ssid} on channel {channel_number}."
    );
    let mac_address = MACAddress::new(Efuse::get_mac_address());
    let mut beacon_template = BeaconFrame {
        header: ManagementFrameHeader {
            bssid: mac_address,
            transmitter_address: mac_address,
            receiver_address: BROADCAST,
            ..Default::default()
        },
        body: BeaconBody {
            capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
            timestamp: 0,
            beacon_interval: 100,
            elements: element_chain! {
                SSIDElement::new(ssid).unwrap(),
                supported_rates![
                        1 B,
                        2 B,
                        5.5 B,
                        11 B,
                        6,
                        9,
                        12,
                        18
                    ],
                DSSSParameterSetElement {
                    current_channel: 1
                },
                TIMElement {
                    dtim_count: 1,
                    dtim_period: 2,
                    bitmap: None::<TIMBitmap<StaticBitmap>>,
                    _phantom: PhantomData
                }
            },
            _phantom: PhantomData,
        },
    };
    let start_timestamp = Instant::now();
    let mut seq_num = 0;
    let mut beacon_interval = Ticker::every(Duration::from_micros(100 * TU.as_micros() as u64));
    loop {
        let mut buf = [0x00; 200];
        beacon_template.body.timestamp = start_timestamp.elapsed().as_micros();
        beacon_template
            .header
            .sequence_control
            .set_sequence_number(seq_num);
        let written = buf.pwrite(beacon_template, 0).unwrap();
        wifi.transmit(&buf[..written], WiFiRate::PhyRate6M).await;
        seq_num += 1;
        beacon_interval.next().await;
    }
}
fn dump_command(wifi: &mut WiFi, uart0_tx: &mut UartTx<'_, UART0, Async>) {
    let _ = writeln!(uart0_tx, "Current channel: {}", wifi.get_channel());
}
fn parse_mac(mac_str: &str) -> Option<MACAddress> {
    let mut mac = [0x00u8; 6];
    let mut octet_iter = mac_str
        .split(':')
        .map(|octet| u8::from_str_radix(octet, 0x10).ok());
    for octet in mac.iter_mut() {
        *octet = octet_iter.next()??;
    }
    Some(MACAddress::new(mac))
}
fn filter_command<'a>(
    wifi: &mut WiFi,
    uart0_tx: &mut UartTx<'_, UART0, Async>,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    let bank = match args.next() {
        Some("BSSID") => RxFilterBank::BSSID,
        Some("RA") => RxFilterBank::ReceiverAddress,
        _ => {
            let _ = writeln!(
                uart0_tx,
                "Expected argument [bank], valid banks are BSSID, RA"
            );
            return;
        }
    };
    let interface = match args.next() {
        Some("0") => RxFilterInterface::Zero,
        Some("1") => RxFilterInterface::One,
        _ => {
            let _ = writeln!(
                uart0_tx,
                "Expected argument [interface], valid banks are 0, 1"
            );
            return;
        }
    };
    match args.next() {
        Some("set") => {
            let Some(mac_address) = args.next() else {
                let _ = writeln!(uart0_tx, "A MAC address is required.");
                return;
            };
            let Some(mac_address) = parse_mac(mac_address) else {
                let _ = writeln!(uart0_tx, "The provided MAC address was invalid.");
                return;
            };
            wifi.set_filter(bank, interface, *mac_address, *BROADCAST);
        }
        Some("enable") => {
            wifi.set_filter_status(bank, interface, true);
        }
        Some("disable") => {
            wifi.set_filter_status(bank, interface, false);
        }
        None => {
            let _ = writeln!(
                uart0_tx,
                "Missing operand, valid operands are: set,enable,disable"
            );
        }
        Some(operand) => {
            let _ = writeln!(
                uart0_tx,
                "Invalid operand {operand}, valid operands are: set,enable,disable"
            );
        }
    }
}
async fn sniff_command(wifi: &mut WiFi, uart0_tx: &mut UartTx<'_, UART0, Async>) {
    loop {
        let received = wifi.receive().await;
        let buffer = received.mpdu_buffer();
        let Ok(generic_frame) = GenericFrame::new(buffer, false) else {
            continue;
        };
        let _ = write!(
            uart0_tx,
            "Type: {:?} Address 1: {}",
            generic_frame.frame_control_field().frame_type(),
            generic_frame.address_1()
        );
        if let Some(address_2) = generic_frame.address_2() {
            let _ = write!(uart0_tx, " Address 2: {address_2}");
        } else {
            let _ = writeln!(uart0_tx);
        }
        if let Some(address_3) = generic_frame.address_3() {
            let _ = writeln!(uart0_tx, " Address 3: {address_3}");
        } else {
            let _ = writeln!(uart0_tx);
        }
    }
}
fn scanning_mode_command<'a>(
    wifi: &mut WiFi,
    uart0_tx: &mut UartTx<'_, UART0, Async>,
    mut args: impl Iterator<Item = &'a str> + 'a,
) {
    let interface = match args.next() {
        Some("0") => RxFilterInterface::Zero,
        Some("1") => RxFilterInterface::One,
        _ => {
            let _ = writeln!(
                uart0_tx,
                "Expected argument [interface], valid banks are 0, 1"
            );
            return;
        }
    };
    match args.next() {
        Some("enabled") => {
            wifi.set_scanning_mode(interface, true);
        }
        Some("disabled") => {
            wifi.set_scanning_mode(interface, false);
        }
        _ => {
            let _ = writeln!(uart0_tx, "Expected argument [enabled|disable].");
        }
    }
}
async fn run_command<'a>(
    wifi: &mut WiFi,
    uart0_tx: &mut UartTx<'_, UART0, Async>,
    command: &str,
    args: impl Iterator<Item = &'a str> + 'a,
) {
    match command.trim() {
        "help" => {
            let _ = writeln!(
                uart0_tx,
                "\
                ESP32-OPEN-MAC TEST CLI\n\n\
                Enter the command you wish to execute and hit enter.\n\
                If you want to terminate the currently executing command type 'q'.\n\n\
                Available commands:\n\n\
                channel [get|set] <CHANNEL_NUMBER> - Get or set the current channel.\n\
                scan [hop] - Look for available access points.\n\
                beacon [SSID] <CHANNEL_NUMBER> - Transmit a beacon every 100 TUs.\n\
                dump - Dump status information.\n\
                filter [BSSID|RA] [0|1] [set|enable|disable] <FILTER_ADDRESS> - Set filter status.\n\
                sniff - Logs frames with rough info.\n\
                scanning_mode [enabled|disabled] - Set the scanning mode.
            "
            );
        }
        "channel" => {
            channel_cmd(wifi, uart0_tx, args);
        }
        "scan" => {
            scan_command(wifi, uart0_tx, args).await;
        }
        "beacon" => {
            beacon_command(wifi, uart0_tx, args).await;
        }
        "dump" => {
            dump_command(wifi, uart0_tx);
        }
        "filter" => {
            filter_command(wifi, uart0_tx, args);
        }
        "sniff" => sniff_command(wifi, uart0_tx).await,
        "scanning_mode" => scanning_mode_command(wifi, uart0_tx, args),
        _ => {
            let _ = writeln!(uart0_tx, "Unknown command {command}");
        }
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();
    esp_println::logger::init_logger(LevelFilter::Info);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let (tx_pin, rx_pin) = (io.pins.gpio1, io.pins.gpio3);
    let (mut uart0_rx, mut uart0_tx) = Uart::new_async_with_config(
        peripherals.UART0,
        Config::default().rx_fifo_full_threshold(64),
        rx_pin,
        tx_pin,
    )
    .unwrap()
    .split();

    let dma_resources = mk_static!(DMAResources<1500, 10>, DMAResources::new());
    let mut wifi = WiFi::new(
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        dma_resources,
    );
    let _ = uart0_tx.flush_async().await;
    let _ = writeln!(&mut uart0_tx, "READY");
    loop {
        let _ = write!(&mut uart0_tx, "> ");
        let mut buf = [0x00u8; 0xff];
        let mut current_offset = 0;
        loop {
            let Ok(received) = uart0_rx.read_async(&mut buf[current_offset..]).await else {
                break;
            };
            current_offset += received;
            if current_offset > buf.len() {
                let _ = writeln!(&mut uart0_tx, "Command length exceed buffer limit.");
                break;
            }
            match buf[current_offset - 1] {
                0xd => {
                    let _ = writeln!(&mut uart0_tx);
                    break;
                }
                0x08 => {
                    if current_offset == 1 {
                        current_offset = 0;
                    } else {
                        current_offset -= received + 1;
                        let _ = uart0_tx.write_async(b"\x08\x1bd \x08").await;
                    }
                    continue;
                }
                _ => {}
            }
            let _ = uart0_tx
                .write_async(&buf[(current_offset - received)..current_offset])
                .await;
        }
        let mut cmd = str::from_utf8(&buf[..current_offset])
            .unwrap()
            .split_whitespace();
        select(
            wait_for_quit(&mut uart0_rx),
            run_command(&mut wifi, &mut uart0_tx, cmd.next().unwrap(), cmd),
        )
        .await;
    }
}
