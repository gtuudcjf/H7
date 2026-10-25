"""Validate the delivered PLC/I-O contract, not claim execution of ST logic."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
NEW=ROOT/'EtherCAT/motor/openangle/plc'
class PlcArtifacts(unittest.TestCase):
    def test_io_contract_preserves_all_13_links(self):
        def fields(path):
            text=ET.parse(path).find('.//Declaration').text
            return re.findall(r'(\w+)\s+AT\s+(%[IQ]\*)\s*:\s*(\w+)',text)
        self.assertEqual(fields(NEW/'GVL_MotorIO.TcGVL'),fields(ROOT/'EtherCAT/motor/plc/GVL_MotorIO.TcGVL'))
        self.assertEqual(len(fields(NEW/'GVL_MotorIO.TcGVL')),13)
    def test_task_calls_main_every_10ms(self):
        task=ET.parse(NEW/'PlcTask.TcTTO')
        self.assertEqual(task.findtext('.//CycleTime'),'10000')
        self.assertEqual(task.findtext('.//PouCall/Name'),'MAIN')
    def test_main_has_no_extra_mapped_outputs(self):
        doc=ET.parse(NEW/'MAIN.TcPOU')
        self.assertEqual(doc.find('.//POU').get('Name'),'MAIN')
        self.assertNotRegex(doc.findtext('.//Declaration'),r'\bAT\s+%[IQ]')
        self.assertIsNotNone(doc.find('.//Implementation/ST'))
if __name__=='__main__':unittest.main()
