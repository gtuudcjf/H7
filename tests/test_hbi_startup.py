"""Preprocess the actual firmware with ARMCC; inspect selected startup paths."""
from pathlib import Path
import argparse
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def body(source, name):
    match = re.search(r"\b(?:int|void)\s+" + name + r"\(void\)\s*\{", source)
    assert match, name
    pos = match.end()
    depth = 1
    while depth:
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
        pos += 1
    return source[match.start():pos]


def check(armcc):
    control = ET.parse(ROOT / "MDK-ARM/emptytest.uvprojx").find(".//TargetArmAds/Cads/VariousControls")
    includes = control.findtext("IncludePath").split(";")
    common = [armcc, "--cpu", "Cortex-M7", "--c99", "-E"]
    for path in includes:
        common += ["-I", path]
    for define in control.findtext("Define").split(","):
        if not define.startswith("ETHERCAT_BRINGUP_ONLY="):
            common += ["-D", define]
    out = ROOT / "MDK-ARM/emptytest"
    out.mkdir(exist_ok=True)
    for mode in (0, 1):
        result = subprocess.run(common + [f"-DETHERCAT_BRINGUP_ONLY={mode}", "../Core/Src/main.c"], cwd=ROOT / "MDK-ARM", capture_output=True, check=True)
        (out / f"main-mode{mode}.i").write_bytes(result.stdout)
        text = result.stdout.decode("utf-8", errors="replace")
        main = body(text, "main")
        mpu = body(text, "MPU_Config")
        if mode:
            assert "MX_FMC_Init()" in main and "EtherCAT_Init()" in main and "EtherCAT_Service()" in main
            for call in ("MotorControl_", "MotorTelemetry_", "BissEncoder_", "MX_ADC", "MX_TIM", "MX_SPI", "MX_USB", "MX_USART"):
                assert call not in main, call
            assert "MotorControl_FastTick(" not in text
            assert "0x6C000000" in mpu
        else:
            assert "MotorControl_Start()" in main and "MotorControl_Service()" in main
            assert "EtherCAT_Init()" not in main and "MX_FMC_Init()" not in main
            assert "0x6C000000" not in mpu
    print("ARMCC actual startup mode 0/1 preprocessing: PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--armcc", default="D:/Keil_v5/ARM/ARMCC/bin/armcc.exe")
    check(parser.parse_args().armcc)
