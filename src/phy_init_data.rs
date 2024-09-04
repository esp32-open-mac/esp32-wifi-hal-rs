use esp_wifi_sys::include::esp_phy_init_data_t;

const CONFIG_ESP32_PHY_MAX_TX_POWER: u8 = 20;

const fn limit(val: u8, low: u8, high: u8) -> u8 {
    if val < low {
        low
    } else if val > high {
        high
    } else {
        val
    }
}

pub(crate) static PHY_INIT_DATA_DEFAULT: esp_phy_init_data_t = esp_phy_init_data_t {
    params: [
        3,
        3,
        0x05,
        0x09,
        0x06,
        0x05,
        0x03,
        0x06,
        0x05,
        0x04,
        0x06,
        0x04,
        0x05,
        0x00,
        0x00,
        0x00,
        0x00,
        0x05,
        0x09,
        0x06,
        0x05,
        0x03,
        0x06,
        0x05,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0xfc,
        0xfc,
        0xfe,
        0xf0,
        0xf0,
        0xf0,
        0xe0,
        0xe0,
        0xe0,
        0x18,
        0x18,
        0x18,
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 40, 84),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 40, 72),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 40, 66),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 40, 60),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 40, 56),
        limit(CONFIG_ESP32_PHY_MAX_TX_POWER * 4, 40, 52),
        0,
        1,
        1,
        2,
        2,
        3,
        4,
        5,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],
};