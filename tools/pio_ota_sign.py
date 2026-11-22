# PlatformIO post-build hook (ESP32): with a signing key (tools/ota_sign.py keygen) every build also
# writes firmware.signed.bin next to firmware.bin: the file to pick for a WiFi update. No key: nothing.
Import("env")  # noqa: F821
import importlib.util
import os

root = env["PROJECT_DIR"]  # noqa: F821


def _sign(source, target, env):
    spec = importlib.util.spec_from_file_location("ota_sign", os.path.join(root, "tools", "ota_sign.py"))
    ota_sign = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ota_sign)
    if not os.path.exists(ota_sign.key_path()):
        return
    try:
        key = ota_sign.load_key()
    except SystemExit:
        print("ota_sign: pip install cryptography (in the PlatformIO Python) to sign updates")
        return
    src = str(target[0])
    data = open(src, "rb").read()
    out = src.replace(".bin", ".signed.bin")
    open(out, "wb").write(ota_sign.sign_bytes(data, key))
    print("ota_sign: %s" % out)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", _sign)  # noqa: F821
