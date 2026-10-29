"""Run motor host tests. Example: python tests/run_control_tests.py --compiler C:/tools/tcc.exe"""
import argparse
import os
import re
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent
MODULES = {
    "adc_sequence_callback": [],
    "biss_frame": ["biss_frame"],
    "biss_dma_buffer": ["biss_dma_buffer"],
    "biss_sequence": [],
    "biss_transport": ["biss_encoder", "biss_dma_buffer", "biss_frame"],
    "biss_concurrency": ["biss_dma_buffer"],
    "open_loop": ["open_loop", "svpwm"],
    "current_control": ["current_sense", "current_pi", "foc_transform"],
    "encoder_angle": ["encoder_angle"],
    "encoder_calibration": ["encoder_calibration"],
    "motor_config": ["motor_config_store", "crc32"],
    "motor_runtime_policy": ["motor_runtime_policy"],
    "speed_estimator": ["speed_estimator"],
    "speed_pi": ["speed_pi"],
    "position_controller": ["position_controller"],
    "motor_telemetry_core": ["motor_telemetry_core"],
    "motor_telemetry_adapter": ["motor_telemetry", "motor_telemetry_core"],
    "control_upgrade": ["motor_runtime_policy", "speed_pi", "position_controller",
                        "current_sense", "encoder_calibration"],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=os.environ.get("H7_HOST_CC"))
    args = parser.parse_args()
    bundled = ROOT / ".tools/host-cc/tcc/tcc.exe"
    candidate = args.compiler or (str(bundled) if bundled.is_file() else shutil.which("tcc"))
    if not candidate:
        parser.error("Host compiler unavailable. Install TinyCC in .tools/host-cc/tcc, "
                     "or specify --compiler / H7_HOST_CC (TinyCC/GCC/Clang).")
    compiler_path = Path(shutil.which(candidate) or candidate).expanduser()
    if not compiler_path.is_file():
        parser.error(f"Compiler not found: {candidate}\n"
                     "Set --compiler or H7_HOST_CC to an existing compiler executable. "
                     "The project-local TinyCC path is .tools/host-cc/tcc/tcc.exe.")
    compiler = str(compiler_path.resolve())
    passed = 0
    with tempfile.TemporaryDirectory(prefix="h7-control-tests-") as directory:
        output = Path(directory)
        # Exercise the actual callback, without compiling MCU startup or clocks.
        source = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8-sig")
        callback = re.search(r"void HAL_ADCEx_InjectedConvCpltCallback\([^)]*\)\s*\{[\s\S]*?^\}",
                             source, re.MULTILINE)
        if not callback:
            raise AssertionError("Production ADC callback not found in main.c")
        (output / "adc_callback_under_test.h").write_text(callback.group(), encoding="utf-8")
        for bus in (24, 48):
            for name, modules in MODULES.items():
                exe = output / f"{name}_{bus}.exe"
                command = [compiler, "-Wall", "-Werror", "-include", "tests/host_math_compat.h",
                           f"-DMOTOR_BUS_VOLTAGE_SELECT_V={bus}", "-DMOTOR_CONFIG_HOST_TEST",
                           "-I", "tests/stubs/encoder" if name in ("biss_transport", "biss_concurrency") else "tests/stubs/telemetry",
                           "-I", "Core/Inc/motor", "-I", str(output),
                           f"tests/test_{name}.c"]
                command += [f"Core/Src/motor/{module}.c" for module in modules]
                if "tcc" not in Path(compiler).stem.lower():
                    command += ["-lm"]
                command += ["-o", str(exe)]
                subprocess.run(command, cwd=ROOT, check=True, timeout=60)
                subprocess.run([str(exe)], cwd=ROOT, check=True, timeout=30)
                passed += 1
            print(f"{bus} V: {len(MODULES)} host suites passed", flush=True)
        probe = output / "invalid_bus.c"
        probe.write_text('#include "motor_params.h"\nint main(void) { return 0; }\n', encoding="utf-8")
        for invalid in (36, "24.0f", "48.0f"):
            result = subprocess.run([compiler, "-I", "Core/Inc/motor",
                                     f"-DMOTOR_BUS_VOLTAGE_SELECT_V={invalid}",
                                     str(probe), "-o", str(output / "invalid.exe")],
                                    cwd=ROOT, capture_output=True, text=True, timeout=60)
            if result.returncode == 0:
                raise AssertionError(f"Invalid bus {invalid} unexpectedly compiled")
            passed += 1
    print(f"PASS: {passed} checks ({2 * len(MODULES)} executable suites + 3 rejected bus settings)")


if __name__ == "__main__":
    main()
