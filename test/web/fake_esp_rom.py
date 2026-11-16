#!/usr/bin/env python3
"""A software ESP32 / ESP32-S3 ROM serial bootloader (esptool protocol, ROM flavour, no stub)
for testing flashers without a board.

    fake_esp_rom.py --chip esp32|esp32s3|esp32c6|esp32p4 --stdio          bytes on stdin/stdout (browser test bridge)
    fake_esp_rom.py --chip esp32 --tcp 4000              for esptool: --port socket://localhost:4000
    --dump FILE   writes the flash contents when the client goes away

Implements SYNC, READ_REG / WRITE_REG, GET_SECURITY_INFO (S3), SPI_ATTACH, SPI_SET_PARAMS,
CHANGE_BAUDRATE, FLASH_BEGIN / DATA / END, FLASH_DEFL_BEGIN / DATA / END (zlib), SPI_FLASH_MD5.
Checks payload checksums and the parameter formats of the two ROMs (FLASH_BEGIN of the S3 ROM
takes 5 words, the ESP32 ROM 4). Status = 4 bytes (status, error, 0, 0) like the real ROMs.
"""
import argparse, hashlib, os, socket, struct, sys, zlib

ESP32_MAGIC, S3_MAGIC = 0x00F01D83, 0x9
MAGIC = {"esp32": ESP32_MAGIC, "esp32s3": S3_MAGIC, "esp32c6": 0x2CE0806F, "esp32p4": 0x0ADDBAD0}
CHIP_ID = {"esp32s3": 9, "esp32c6": 13, "esp32p4": 18}  # GET_SECURITY_INFO chip_id
FLASH_MB = {"esp32": 4, "esp32s3": 8, "esp32c6": 8, "esp32p4": 16}


class Rom:
    def __init__(self, chip, size):
        self.chip = chip
        self.flash = bytearray(b"\xff" * size)
        self.attached = False
        self.wr = None  # current write: dict(offset, blocks, block_size, seq, comp)
        self.log = []

    def reply(self, op, val=0, data=b"", status=0, err=0):
        body = data + bytes([status, err, 0, 0])
        return struct.pack("<BBHI", 1, op, len(body), val) + body

    def handle(self, pkt):
        if len(pkt) < 8 or pkt[0] != 0:
            return []
        op, size, chk = pkt[1], struct.unpack("<H", pkt[2:4])[0], struct.unpack("<I", pkt[4:8])[0]
        d = pkt[8:8 + size]
        self.log.append(op)
        if op == 0x08:  # SYNC: the ROM answers several times
            if d != b"\x07\x07\x12\x20" + b"\x55" * 32:
                return []
            return [self.reply(op, 0x20120707)] * 8
        if op == 0x0A:  # READ_REG
            addr = struct.unpack("<I", d[:4])[0]
            v = MAGIC[self.chip] if addr == 0x40001000 else 0
            return [self.reply(op, v)]
        if op == 0x09:  # WRITE_REG
            return [self.reply(op)]
        if op == 0x14:  # GET_SECURITY_INFO: not in the ESP32 ROM
            if self.chip == "esp32":
                return [self.reply(op, status=1, err=5)]
            return [self.reply(op, data=struct.pack("<IB7sII", 0, 0, b"\0" * 7, CHIP_ID[self.chip], 0))]
        if op == 0x0D:  # SPI_ATTACH: ROM form has 8 bytes
            if len(d) != 8:
                return [self.reply(op, status=1, err=5)]
            self.attached = True
            return [self.reply(op)]
        if op == 0x0B:  # SPI_SET_PARAMS
            if len(d) != 24:
                return [self.reply(op, status=1, err=5)]
            return [self.reply(op)]
        if op == 0x0F:  # CHANGE_BAUDRATE
            return [self.reply(op)]
        if op in (0x02, 0x10):  # FLASH_BEGIN / FLASH_DEFL_BEGIN
            want = 16 if self.chip == "esp32" else 20
            if len(d) != want or not self.attached:
                return [self.reply(op, status=1, err=5)]
            erase, blocks, bs, off = struct.unpack("<IIII", d[:16])
            if off + erase > len(self.flash):
                return [self.reply(op, status=1, err=6)]
            self.flash[off:off + erase] = b"\xff" * erase
            self.wr = {"off": off, "pos": off, "blocks": blocks, "bs": bs, "seq": 0, "comp": op == 0x10,
                       "z": zlib.decompressobj() if op == 0x10 else None}
            return [self.reply(op)]
        if op in (0x03, 0x11):  # FLASH_DATA / FLASH_DEFL_DATA
            w = self.wr
            if not w or w["comp"] != (op == 0x11):
                return [self.reply(op, status=1, err=5)]
            n, seq = struct.unpack("<II", d[:8])
            payload = d[16:16 + n]
            c = 0xEF
            for b in payload:
                c ^= b
            if c != chk or len(payload) != n:
                return [self.reply(op, status=1, err=7)]
            if seq != w["seq"]:
                return [self.reply(op, status=1, err=6)]
            w["seq"] += 1
            out = w["z"].decompress(payload) if w["comp"] else payload
            self.flash[w["pos"]:w["pos"] + len(out)] = out
            w["pos"] += len(out)
            return [self.reply(op)]
        if op in (0x04, 0x12):  # FLASH_END / FLASH_DEFL_END
            return [self.reply(op)]
        if op == 0x13:  # SPI_FLASH_MD5: 32 hex characters in the ROM
            addr, n = struct.unpack("<II", d[:8])
            return [self.reply(op, data=hashlib.md5(bytes(self.flash[addr:addr + n])).hexdigest().encode())]
        return [self.reply(op, status=1, err=5)]


def slip_frames(read):
    buf, inside, esc = bytearray(), False, False
    while True:
        chunk = read()
        if not chunk:
            return
        for b in chunk:
            if b == 0xC0:
                if inside and buf:
                    yield bytes(buf)
                buf, inside, esc = bytearray(), True, False
            elif inside:
                if esc:
                    buf.append(0xC0 if b == 0xDC else 0xDB if b == 0xDD else b)
                    esc = False
                elif b == 0xDB:
                    esc = True
                else:
                    buf.append(b)


def slip(p):
    return b"\xc0" + p.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc") + b"\xc0"


def serve(rom, read, write, dump):
    try:
        for f in slip_frames(read):
            for r in rom.handle(f):
                write(slip(r))
    finally:
        if dump:
            with open(dump, "wb") as fh:
                fh.write(rom.flash)
            with open(dump + ".ops", "w") as fh:
                fh.write(" ".join("%02x" % o for o in rom.log))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chip", default="esp32", choices=sorted(MAGIC))
    ap.add_argument("--stdio", action="store_true")
    ap.add_argument("--tcp", type=int)
    ap.add_argument("--dump")
    a = ap.parse_args()
    rom = Rom(a.chip, FLASH_MB[a.chip] << 20)
    if a.stdio:
        inp, out = sys.stdin.buffer.raw, sys.stdout.buffer

        def w(b):
            out.write(b)
            out.flush()
        serve(rom, lambda: os.read(inp.fileno(), 65536), w, a.dump)
    else:
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", a.tcp))
        s.listen(1)
        c, _ = s.accept()
        serve(rom, lambda: c.recv(65536), c.sendall, a.dump)


if __name__ == "__main__":
    main()
