"""Build lab revision-2 ESI and an UNVERIFIED SII candidate, never program hardware.

Wire layout follows the approved 8/20-byte ABI. SII category layout was checked
against the primary siitool reader/writer, not imported from its source:
https://github.com/synapticon/siitool/blob/master/sii.c
CRC-8 polynomial 0x07, initial 0xff, over the first 14 configuration bytes.
"""
import argparse
import hashlib
import json
import pathlib
import struct
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]
RX = [('ControlWord','UINT',16,1),('TargetSpeed','INT',16,2),('CommandSequence','UDINT',32,3)]
TX = [('StatusWord','UINT',16,1),('CycleCounter','UDINT',32,2),
      ('ActualSpeed','INT',16,3),('ActualIq','INT',16,4),
      ('MotorFault','UINT',16,5),('BridgeFault','UINT',16,6),
      ('ActiveTargetSpeed','INT',16,7),('AcceptedSequence','UDINT',32,8)]

def child(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent,tag,attrs)
    if text is not None: node.text=str(text)
    return node

def build_esi():
    root=ET.parse(ROOT/'EtherCAT/H743-LAN9252-LAB.xml').getroot()
    device=root.find('./Descriptions/Devices/Device')
    device.find('Type').set('RevisionNo','#x00000002')
    device.find('Name').text='H743-LAN9252-LAB Motor rev2'
    types=device.find('./Profile/Dictionary/DataTypes')
    dt=child(types,'DataType');child(dt,'Name','INT');child(dt,'BitSize',16)
    for type_name,fields in [('DT1600',[(f'{n} mapping','UDINT',32,s) for n,_,_,s in RX]),
                             ('DT1A00',[(f'{n} mapping','UDINT',32,s) for n,_,_,s in TX]),
                             ('DT7000',RX),('DT6000',TX)]:
        dt=types.find(f"DataType[Name='{type_name}']")
        for item in list(dt):
            if item.tag not in ('Name','BitSize'): dt.remove(item)
        fields=[('Number of entries','USINT',8,0)]+fields
        offset=0
        for name,kind,bits,sub in fields:
            alignment=32 if bits==32 else (16 if sub else 8)
            offset=(offset+alignment-1)//alignment*alignment
            item=child(dt,'SubItem');child(item,'SubIdx',sub);child(item,'Name',name)
            child(item,'Type',kind);child(item,'BitSize',bits);child(item,'BitOffs',offset)
            flags=child(item,'Flags');child(flags,'Access','ro');child(flags,'Category','o')
            offset+=bits
        dt.find('BitSize').text=str(offset)
    objects=device.find('./Profile/Dictionary/Objects')
    identity=objects.find("Object[Index='#x1018']")
    identity.findall('./Info/SubItem')[3].find('./Info/DefaultData').text='02000000'
    for index,fields,mapping in [(0x1600,RX,0x7000),(0x1a00,TX,0x6000),(0x7000,RX,None),(0x6000,TX,None)]:
        obj=objects.find(f"Object[Index='#x{index:04x}']")
        obj.find('BitSize').text=types.find(f"DataType[Name='DT{index:04X}']/BitSize").text
        old=obj.find('Info')
        if old is not None: obj.remove(old)
        info=child(obj,'Info')
        for name,_,bits,sub in [('Number of entries','USINT',8,0)]+fields:
            item=child(info,'SubItem');child(item,'Name',name)
            value=len(fields) if sub==0 else (((mapping<<16)|(sub<<8)|bits) if mapping else 0)
            width=1 if sub==0 else (4 if mapping else bits//8)
            child(child(item,'Info'),'DefaultData',value.to_bytes(width,'little').hex().upper())
        obj.find('./Flags/Access').text='ro'
    for sm,size in zip(device.findall('Sm'),[128,128,8,20]): sm.set('DefaultSize',str(size))
    for tag,fields,index,sm,obj in [('RxPdo',RX,0x1600,2,0x7000),('TxPdo',TX,0x1a00,3,0x6000)]:
        pdo=device.find(tag)
        for entry in pdo.findall('Entry'): pdo.remove(entry)
        pdo.find('Name').text='Motor outputs' if tag=='RxPdo' else 'Motor inputs'
        for name,kind,bits,sub in fields:
            entry=child(pdo,'Entry');child(entry,'Index',f'#x{obj:04x}')
            child(entry,'SubIndex',sub);child(entry,'BitLen',bits)
            child(entry,'Name',name);child(entry,'DataType',kind)
    return root

def crc8(data):
    crc=255
    for byte in data:
        crc^=byte
        for _ in range(8): crc=((crc<<1)^(7 if crc&128 else 0))&255
    return crc

def build_sii(root):
    device=root.find('./Descriptions/Devices/Device')
    strings=['H743-LAN9252-LAB','H743-LAN9252-LAB Motor rev2','Motor outputs','Motor inputs']
    for name,_,_,_ in RX+TX:
        if name not in strings: strings.append(name)
    def sid(name): return strings.index(name)+1
    def category(kind,data):
        data+=b'\0'*(len(data)%2)
        return struct.pack('<HH',kind,len(data)//2)+data
    image=bytearray(2048*b'\xff');image[:128]=128*b'\0'
    config=bytes.fromhex(device.findtext('./Eeprom/ConfigData'))
    if len(config)!=14: raise ValueError('Expected 14 configuration bytes')
    image[:14]=config;image[14]=crc8(config)
    struct.pack_into('<IIII',image,16,0xf0c743,0x9252,2,1)
    struct.pack_into('<HHHHH',image,48,0x1000,128,0x1080,128,4)
    struct.pack_into('<HH',image,124,15,1)
    string_data=bytes([len(strings)])+b''.join(bytes([len(s)])+s.encode('ascii') for s in strings)
    general=bytearray(32);general[0]=1;general[3]=2;general[5]=3;general[11]=4;general[16]=0x11
    managers=b''.join(struct.pack('<HHBBBB',int(sm.get('StartAddress')[2:],16),int(sm.get('DefaultSize')),
                      int(sm.get('ControlByte')[2:],16),0,1,i+1) for i,sm in enumerate(device.findall('Sm')))
    cats=category(10,string_data)+category(30,bytes(general))+category(40,bytes([1,2,3,0]))+category(41,managers)
    for kind,tag in [(50,'TxPdo'),(51,'RxPdo')]:
        pdo=device.find(tag);entries=pdo.findall('Entry')
        data=struct.pack('<HBBBBH',int(pdo.findtext('Index')[2:],16),len(entries),int(pdo.get('Sm')),0,sid(pdo.findtext('Name')),0x11)
        for entry in entries:
            data+=struct.pack('<HBBBBH',int(entry.findtext('Index')[2:],16),int(entry.findtext('SubIndex')),
                    sid(entry.findtext('Name')),{'UINT':6,'INT':3,'UDINT':7}[entry.findtext('DataType')],int(entry.findtext('BitLen')),0)
        cats+=category(kind,data)
    cats+=struct.pack('<HH',0xffff,0)
    if 128+len(cats)>len(image): raise ValueError('SII exceeds EEPROM size')
    image[128:128+len(cats)]=cats
    return bytes(image)

def manifest():
    return {'revision':2,'rx_bytes':8,'tx_bytes':20,'hardware_verified':False,
            'write_authorized':False,'status':'CANDIDATE ONLY - compare with TwinCAT generated SII before any write',
            'power_default':'locked','speed_unit':'motor shaft 0.1 rpm','iq_unit':'mA',
            'rx':RX,'tx':TX}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir',type=pathlib.Path,default=ROOT/'EtherCAT/motor')
    args=parser.parse_args();args.out_dir.mkdir(parents=True,exist_ok=True)
    root=build_esi();ET.indent(root,space='  ')
    esi=ET.tostring(root,encoding='utf-8',xml_declaration=True)+b'\n'
    sii=build_sii(root);m=manifest()
    m.update(esi_sha256=hashlib.sha256(esi).hexdigest(),sii_sha256=hashlib.sha256(sii).hexdigest())
    (args.out_dir/'H743-LAN9252-MOTOR.xml').write_bytes(esi)
    (args.out_dir/'H743-LAN9252-MOTOR-rev2-CANDIDATE.bin').write_bytes(sii)
    (args.out_dir/'candidate-manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Generated revision-2 ESI / 2048-byte UNVERIFIED SII candidate. No device access.')
if __name__=='__main__': main()
