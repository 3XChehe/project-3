def calc(data):
    crc = data
    for i in range(8):
        if crc & 0x80:
            crc = ((crc << 1) ^ 0xD5) & 0xFF
        else:
            crc = (crc << 1) & 0xFF
    return crc
tab = [calc(i) for i in range(256)]
print(",".join(hex(x) for x in tab[128:192]))
