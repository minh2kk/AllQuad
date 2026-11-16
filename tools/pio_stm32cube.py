# AllQuad - PlatformIO extra script for the STM32Cube builds (no framework).
# The bare-metal builder of platform-ststm32 sets only -mcpu: add the FPU / ABI flags to the compiler,
# the assembler AND the linker (newlib must be the hard-float variant), newlib-nano and float printf.
Import("env")

cpu = env.BoardConfig().get("build.cpu", "cortex-m4")
fpu = "fpv4-sp-d16"
mcu = env.BoardConfig().get("build.mcu", "").lower()
if mcu.startswith("stm32h7"):
    fpu = "fpv5-d16"
elif mcu.startswith("stm32f7"):
    fpu = "fpv5-sp-d16"

fp = ["-mfpu=%s" % fpu, "-mfloat-abi=hard"]
env.Append(
    ASFLAGS=fp,
    ASPPFLAGS=fp,
    CCFLAGS=fp + ["-ffunction-sections", "-fdata-sections"],
    LINKFLAGS=fp + [
        "--specs=nano.specs",
        "--specs=nosys.specs",
        "-u", "_printf_float",
        "-Wl,--gc-sections",
        "-Wl,--print-memory-usage",
    ],
)
