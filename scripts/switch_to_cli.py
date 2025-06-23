import serial as ser
from binascii import unhexlify
import crcmod
import time

CRC16_FUNC = crcmod.predefined.mkCrcFun('xmodem')

def main():
    deescape_ng_packet(send_ng_command(0, 1, None))
    time.sleep(1)
    try:
        deescape_ng_packet(send_ng_command(0, 40, None))
    except:
        pass
    print("Switched to CLI. NG Commands will no more work. Exit the cli using the cli command 'exitcli'")

def escape_ng_packet(list_to_escape):
    escaped_list = []
    for byte in list_to_escape:
        if byte == '02':
            escaped_list.append('1B')
            escaped_list.append('22')
        elif byte == '03':
            escaped_list.append('1B')
            escaped_list.append('23')
        elif byte == '1B':
            escaped_list.append('1B')
            escaped_list.append('3B')
        else:
            escaped_list.append(byte)
    return escaped_list

def deescape_ng_packet(list_to_deescape):
    deescaped_list = []
    idx = 0
    while True:
        if list_to_deescape[idx] == '1b' and list_to_deescape[idx+1] == '22':
            deescaped_list.append('02')
            idx += 2
        elif list_to_deescape[idx] == '1b' and list_to_deescape[idx+1] == '23':
            deescaped_list.append('03')
            idx += 2
        elif list_to_deescape[idx] == '1b' and list_to_deescape[idx+1] == '3B':
            deescaped_list.append('1b')
            idx += 2
        else:
            deescaped_list.append(list_to_deescape[idx])
            idx += 1
        if idx >= len(list_to_deescape):
            break
    return deescaped_list
        
#data is a list of bytes that needs to be sent
def send_ng_command(cmd, sub_cmd, data = None):
    STX = 2
    ETX = 3
    data_len = 0
    NG_Command = []
    response_list = []
    #NG_Command.append(f'{STX:02x}')
    NG_Command.append(f'{cmd:02x}')
    NG_Command.append(f'{sub_cmd:02x}')
    if data is not None:
        data_str = [f'{byte:02x}' for byte in data]
        NG_Command.extend(data_str)
        data_len = len(data_str)
    NG_Command.append(f'{((data_len & 0xFF00) >> 8) :02x}')
    NG_Command.append(f'{(data_len & 0xFF):02x}')
    send_pkt = "".join(NG_Command[1:])
    send_pkt_bytes = unhexlify(send_pkt)
    crc16 = CRC16_FUNC(send_pkt_bytes)
    NG_Command.append(f'{((crc16 & 0xFF00) >> 8) :02x}')
    NG_Command.append(f'{(crc16 & 0xFF) :02x}')
    #NG_Command.append(f'{ETX:02x}')
    escaped_NG_command = escape_ng_packet(NG_Command)
    escaped_NG_command.insert(0,f'{STX:02x}')
    escaped_NG_command.append(f'{ETX:02x}')
    # print(escaped_NG_command)
    cmd_pkt = unhexlify("".join(escaped_NG_command))
    with ser.Serial('COM4', 9600, timeout=5) as ser_port:
        ser_port.write(cmd_pkt)
        response = ser_port.read_until(expected=b'\x03')
        response_list = [f'{x:02x}' for x in response]
    return response_list




if __name__ == "__main__":
    main()