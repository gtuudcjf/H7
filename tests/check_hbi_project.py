"""Validate the build inputs and the device description as one deliverable."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def check():
    project = ET.parse(ROOT / "MDK-ARM/emptytest.uvprojx").getroot()
    target = project.find("./Targets/Target")
    control = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
    assert "ETHERCAT_BRINGUP_ONLY=1" in control.findtext("Define")
    paths = []
    for node in target.findall("./Groups/Group/Files/File"):
        path = Path(node.findtext("FilePath").replace("\\", "/"))
        if not path.is_absolute():
            path = ROOT / "MDK-ARM" / path
        assert path.is_file(), f"Missing build input: {path}"
        paths.append(path.resolve())
    assert len(paths) == len(set(paths)), "Duplicate compilation units"
    for unit in (ROOT / "Core/Src/ethercat").glob("*.c"):
        assert unit.resolve() in paths, unit
    assert (ROOT / "Core/Src/fmc.c").resolve() in paths
    assert not any("407 LAN9252" in str(p) or p.name == "lan9252_spi.c" for p in paths)
    esi = ET.parse(ROOT / "EtherCAT/H743-LAN9252-HBI-LAB.xml").getroot()
    device = esi.find("./Descriptions/Devices/Device")
    assert esi.findtext("./Vendor/Id").lower() == "#x00f0c743"
    assert device.find("Type").attrib == {"ProductCode": "#x00009252", "RevisionNo": "#x00000002"}
    assert device.findtext("Name") == "H743-LAN9252-HBI-LAB"
    assert bytes.fromhex(device.findtext("./Eeprom/ConfigData")) == bytes.fromhex("890e80cc8813f000000000800000")
    assert device.findtext("./Eeprom/ByteSize") == "2048"
    assert device.find("Dc") is None
    assert device.find("./Mailbox/CoE") is not None
    assert [(s.text, s.get("StartAddress"), s.get("DefaultSize"), s.get("ControlByte")) for s in device.findall("Sm")] == [
        ("MBoxOut", "#x1000", "128", "#x26"), ("MBoxIn", "#x1080", "128", "#x22"),
        ("Outputs", "#x1100", "2", "#x64"), ("Inputs", "#x1400", "6", "#x20")]
    assert [(p.tag, sum(int(x.text) for x in p.findall("./Entry/BitLen"))) for p in device if p.tag in ("RxPdo", "TxPdo")] == [("RxPdo", 16), ("TxPdo", 48)]
    revision = device.find(".//Object[Index='#x1018']/Info/SubItem[Name='Revision']/Info/DefaultData")
    assert revision.text == "02000000"
    name_object = device.find(".//Object[Index='#x1008']")
    assert name_object.findtext("Type") == "STRING(20)"
    assert name_object.findtext("BitSize") == "160"
    assert bytes.fromhex(name_object.findtext("./Info/DefaultData")) == b"H743-LAN9252-HBI-LAB"
    identity = (ROOT / "Core/Inc/ethercat/ethercat_identity.h").read_text()
    assert re.search(r"ETHERCAT_REVISION\s+UINT32_C\(2\)", identity)
    ioc = dict(line.split("=", 1) for line in (ROOT / "emptytest.ioc").read_text().splitlines() if "=" in line and not line.startswith("#"))
    for pin in ("PA4", "PA5", "PA6", "PA7", "PF0", "PF2", "PF3"):
        assert ioc[pin + ".Signal"] == "GPIO_Analog"
    assert ioc["PB5.PinState"] == ioc["PC4.PinState"] == "GPIO_PIN_RESET"
    assert ioc["PF1.PinState"] == "GPIO_PIN_SET"
    assert ioc["PB7.Signal"] == "FMC_NL" and ioc["PG12.Signal"] == "FMC_NE4"
    assert ioc["FMC.DataAddressMux4-NorPsramChipSelect4_4"] == "FMC_DATA_ADDRESS_MUX_ENABLE"
    assert ioc["FMC.NSBank4-NorPsramChipSelect4_4"] == "FMC_NORSRAM_BANK4"
    assert len([k for k in ioc if re.fullmatch(r"Mcu.Pin\d+", k)]) == int(ioc["Mcu.PinsNb"])
    print(f"HBI project/ESI/CubeMX: PASS ({len(paths)} build inputs)")


if __name__ == "__main__":
    check()
