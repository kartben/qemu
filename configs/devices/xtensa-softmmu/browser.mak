# Device configuration for zephyr-in-the-browser's xtensa-softmmu artifact.
#
# Selected by tools/build-qemu-wasm.sh with --with-devices-xtensa=browser.
# Same approach as the other browser.mak files: disable every board except the
# ESP32 parts this artifact boots. Leaving CONFIG_XTENSA_ESP32 and
# CONFIG_XTENSA_ESP32S3 alone keeps their `select` edges (SSI, HOST_I2C,
# HOST_SPI, CAN_SJA1000, ESP_RGB, …) and therefore the browser bridges.
#
# XTENSA_VIRT is the expensive one to drop: it selects PCI_EXPRESS_GENERIC_BRIDGE
# and PCI_DEVICES, and neither ESP32 machine has a PCI bus at all.

# Boards other than the ESP32 parts.
CONFIG_XTENSA_SIM=n
CONFIG_XTENSA_VIRT=n
CONFIG_XTENSA_XTFPGA=n

# Optional device catalogues — keep off to shrink the wasm binary.
CONFIG_PCI_DEVICES=n
CONFIG_TEST_DEVICES=n
