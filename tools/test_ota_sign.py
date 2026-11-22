#!/usr/bin/env python3
"""Signed WiFi updates (tools/ota_sign.py): sign / verify round trip, tampering, re-signing, wrong key.
The firmware checks the same trailer with mbedtls (src/web/web_esp32.cpp, onOta)."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ota_sign as o
from cryptography.hazmat.primitives.asymmetric import ec

fails = 0
def check(ok, msg):
    global fails
    print("[%s] %s" % ("PASS" if ok else "FAIL", msg)); fails += not ok

k1, k2 = ec.generate_private_key(ec.SECP256R1()), ec.generate_private_key(ec.SECP256R1())
img = bytes([0xE9]) + os.urandom(200000)
s = o.sign_bytes(img, k1)
check(len(s) == len(img) + 72 and s[:len(img)] == img and s[len(img):len(img) + 8] == o.MAGIC, "trailer: image unchanged + 72 bytes")
check(o.verify_bytes(s, o.pub_bytes(k1)), "signature verifies with its key")
t = bytearray(s); t[1234] ^= 1
check(not o.verify_bytes(bytes(t), o.pub_bytes(k1)), "one flipped bit in the image: refused")
check(not o.verify_bytes(s, o.pub_bytes(k2)), "another key: refused")
check(not o.verify_bytes(img, o.pub_bytes(k1)), "unsigned image: refused")
check(o.sign_bytes(s, k1)[:len(img)] == img and len(o.sign_bytes(s, k1)) == len(s), "signing a signed image replaces the trailer")
print("\n%d/%d" % (6 - fails, 6))
sys.exit(1 if fails else 0)
