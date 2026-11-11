# PlatformIO post-upload step.
#
# Uploads enter the ROM bootloader via the TinyUSB CDC 1200-baud touch, which
# sets RTC_CNTL_FORCE_DOWNLOAD_BOOT. That bit survives resets, so after
# flashing the chip would keep rebooting into the bootloader until power is
# removed. Clear it, then reset into the new firmware.
import time

from serial.tools import list_ports

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

RTC_CNTL_OPTION1_REG = "0x6000812C"


def clear_force_download(source, target, env):
    port = env.subst("$UPLOAD_PORT")
    esptool = '"$PYTHONEXE" "$UPLOADER" --chip esp32s3 --port "%s" --before no_reset --no-stub' % port
    env.Execute(env.VerboseAction(
        "%s --after no_reset write_mem %s 0 0x1" % (esptool, RTC_CNTL_OPTION1_REG),
        "Clearing force-download flag"))
    # Closing the port normally resets the chip into the new firmware. A reset
    # in the same esptool session as the write was seen to leave it in the
    # bootloader, so only if the bootloader port is still there, reset again.
    time.sleep(2)
    if any(p.device == port for p in list_ports.comports()):
        env.Execute(env.VerboseAction(
            "%s --after hard_reset read_mem %s" % (esptool, RTC_CNTL_OPTION1_REG),
            "Restarting into new firmware"))


env.AddPostAction("upload", clear_force_download)  # noqa: F821
