MODE = {
    "SLEEP": 0x00,
    "STDBY": 0x01,
    "FSTX": 0x02,
    "TX": 0x03,
    "FSRX": 0x04,
    "RXCONT": 0x05,
    "RX_SINGLE": 0x06,
    "CAD": 0x07
}

REG = {
    "OP_MODE": 0x01,
    "FR_MSB": 0x06,
    "FR_MID": 0x07,
    "FR_LSB": 0x08,
    "PA_CONFIG": 0x09,
    "OCP": 0x0B,
    "LNA": 0x0C,
    "FIFO_ADDR_PTR": 0x0D,
    "FIFO_TX_BASE_ADDR": 0x0E,
    "FIFO_RX_BASE_ADDR": 0x0F,
    "FIFO_RX_CURRENT_ADDR": 0x10,
    "IRQ_FLAGS": 0x12,
    "RX_NB_BYTES": 0x13,
    "PKT_SNR_VALUE": 0x19,
    "PKT_RSSI_VALUE": 0x1A,
    "MODEM_CONFIG1": 0x1D,
    "MODEM_CONFIG2": 0x1E,
    "PREAMBLE_MSB": 0x20,
    "PREAMBLE_LSB": 0x21,
    "PAYLOAD_LENGTH": 0x22,
    "MODEM_CONFIG3": 0x26,
    "DIO_MAPPING_1": 0x40,
    "DIO_MAPPING_2": 0x41,
    "VERSION": 0x42,
    "PA_DAC": 0x4D
}

BW = {
    "BW7800": 0,
    "BW10400": 1,
    "BW15600": 2,
    "BW20800": 3,
    "BW31250": 4,
    "BW41700": 5,
    "BW62500": 6,
    "BW125000": 7,
    "BW250000": 8,
    "BW500000": 9
}

CR = {
    "CR4_5": 1,
    "CR4_6": 2,
    "CR4_7": 3,
    "CR4_8": 4
}
