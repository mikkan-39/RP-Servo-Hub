import struct

START_BYTE = 0xAA

COMMANDS = {
    "PING": 0x01,
    "QUERY_POSITIONS": 0x02,
    "QUERY_SPEED": 0x03,
    "SET_POS": 0x04,
    "SET_SPEED": 0x05,
    "SET_ENABLED": 0x06,
    "EXIT": 0x07
}

def crc8(data):
    crc = 0x00
    for b in data:
        extract = b
        for _ in range(8):
            sum_ = (crc ^ extract) & 0x01
            crc >>= 1
            if sum_:
                crc ^= 0x8C
            extract >>= 1
    return crc

def build_packet(command, payload_bytes=b''):
    cmd = COMMANDS[command]
    length = len(payload_bytes)
    packet = bytearray([START_BYTE, cmd, length]) + payload_bytes
    checksum = crc8(packet)
    packet.append(checksum)
    return packet

def print_packet(name, packet):
    print(f"{name}:")
    print(" ".join(f"{b:02X}" for b in packet))
    print()

if __name__ == "__main__":
    # PING
    print_packet("PING", build_packet("PING"))

    # QUERY_POSITIONS
    print_packet("QUERY_POSITIONS", build_packet("QUERY_POSITIONS"))

    # QUERY_SPEED
    print_packet("QUERY_SPEED", build_packet("QUERY_SPEED"))

    # SET_POS for servo 1 = 1500, servo 2 = -1200
    payload = struct.pack("<BhBh", 1, 1500, 2, -1200)
    print_packet("SET_POS (1=1500, 2=-1200)", build_packet("SET_POS", payload))

    # SET_SPEED for servo 1 = 250, servo 2 = -250
    payload = struct.pack("<BhBh", 1, 250, 2, -250)
    print_packet("SET_SPEED (1=250, 2=-250)", build_packet("SET_SPEED", payload))
    
    # SET_ENABLED: servo 1 = true, servo 2 = false
    payload = struct.pack("<BB", 1, 1) + struct.pack("<BB", 2, 0)
    print_packet("SET_ENABLED (1=1, 2=0)", build_packet("SET_ENABLED", payload))

    # EXIT
    print_packet("EXIT", build_packet("EXIT"))
