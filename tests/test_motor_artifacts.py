import importlib.util
import pathlib
import struct
import unittest
import xml.etree.ElementTree as ET
import zipfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('motor_artifacts', ROOT / 'tools/generate_motor_artifacts.py')
artifacts = importlib.util.module_from_spec(spec)
spec.loader.exec_module(artifacts)

class MotorArtifacts(unittest.TestCase):
    def test_delivered_archive_has_current_sources_and_no_boot_payload(self):
        with zipfile.ZipFile(ROOT / 'EtherCAT/motor/plc/EtherCATMotorSpeedDemo.tpzip') as z:
            for name in z.namelist():
                self.assertNotIn('..', pathlib.PurePosixPath(name).parts)
                self.assertNotIn('_Boot', name)
            compiled = ET.fromstring(z.read('POUs/MAIN.TcPOU')).find('./POU')
            current = ET.parse(ROOT / 'EtherCAT/motor/plc/MAIN.TcPOU').find('./POU')
            for path in ['Declaration', 'Implementation/ST']:
                self.assertEqual(compiled.findtext(path).split(), current.findtext(path).split())
            self.assertEqual(ET.fromstring(z.read('PlcTask.TcTTO')).findtext('./Task/CycleTime'), '10000')

    def test_plc_reset_waits_for_slave_acknowledgement(self):
        # Structural PLC contract; actual ST compilation is a separate XAE check.
        st = ET.parse(ROOT / 'EtherCAT/motor/plc/MAIN.TcPOU').findtext('./POU/Implementation/ST')
        reset_phase = st.split('\n90:', 1)[1].split('\nELSE', 1)[0]
        self.assertNotIn('nPhase := 0', reset_phase)
        self.assertIn('nPhase := 80', reset_phase)
        self.assertIn('\n80:', st)
    def test_revision2_signed_pdo_and_wire_lengths(self):
        root = artifacts.build_esi()
        d = root.find('./Descriptions/Devices/Device')
        self.assertEqual(d.find('Type').get('RevisionNo'), '#x00000002')
        self.assertEqual([int(s.get('DefaultSize')) for s in d.findall('Sm')], [128,128,8,20])
        self.assertEqual([e.findtext('DataType') for e in d.findall('./RxPdo/Entry')], ['UINT','INT','UDINT'])
        self.assertEqual([int(e.findtext('BitLen')) for e in d.findall('./TxPdo/Entry')], [16,32,16,16,16,16,16,32])
        self.assertEqual(d.find("./Profile/Dictionary/Objects/Object[Index='#x1018']/Info/SubItem[4]/Info/DefaultData").text,'02000000')
        self.assertIsNone(d.find('Dc'))
    def test_sii_crc_identity_sm_and_mapping(self):
        b = artifacts.build_sii(artifacts.build_esi())
        self.assertEqual(len(b),2048)
        self.assertEqual(b[:16], bytes.fromhex('800e00cc8813f0000000008000002200'))
        self.assertEqual(struct.unpack_from('<IIII',b,16),(0xf0c743,0x9252,2,1))
        self.assertEqual(struct.unpack_from('<HHHHH',b,48),(0x1000,128,0x1080,128,4))
        cats={}; off=128
        while True:
            kind,size=struct.unpack_from('<HH',b,off)
            if kind==0xffff: break
            self.assertNotIn(kind,cats);cats[kind]=b[off+4:off+4+2*size];off+=4+2*size
        self.assertLess(off,2048)
        self.assertEqual(struct.unpack_from('<HHBBBB',cats[41],16),(0x1100,8,0x64,0,1,3))
        self.assertEqual(struct.unpack_from('<HHBBBB',cats[41],24),(0x1400,20,0x20,0,1,4))
        self.assertEqual(struct.unpack_from('<HBB',cats[51]),(0x1600,3,2))
        self.assertEqual(struct.unpack_from('<HBB',cats[50]),(0x1a00,8,3))
        self.assertEqual(cats[51][8+8+4:8+8+6],bytes([3,16])) # TargetSpeed INTEGER16
        self.assertNotIn(60,cats)
    def test_no_claim_of_hardware_verification(self):
        manifest=artifacts.manifest()
        self.assertFalse(manifest['hardware_verified'])
        self.assertFalse(manifest['write_authorized'])
        self.assertEqual(manifest['revision'],2)

if __name__=='__main__': unittest.main()
