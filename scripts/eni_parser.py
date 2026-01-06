#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ENI (EtherCAT Network Information) Parser

Copyright (c) 2025, sakumisu

SPDX-License-Identifier: Apache-2.0

"""

import xml.etree.ElementTree as ET
import sys
import os
from typing import Dict, List, Tuple, Optional

class ENIParser:
    def __init__(self):
        self.slaves = []

    def parse_hex_value(self, hex_str: str) -> int:
        """解析十六进制字符串"""
        if not hex_str:
            return 0
        hex_str = hex_str.strip()
        if hex_str.startswith('#x'):
            return int(hex_str[2:], 16)
        elif hex_str.startswith('0x'):
            return int(hex_str[2:], 16)
        else:
            try:
                return int(hex_str, 16)
            except:
                try:
                    return int(hex_str, 10)
                except:
                    return 0

    def parse_slave_info(self, slave_elem):
        """解析从站基本信息"""
        slave_info = {}

        info_elem = slave_elem.find('Info')
        if info_elem is not None:
            name_elem = info_elem.find('Name')
            if name_elem is not None:
                slave_info['name'] = name_elem.text.strip() if name_elem.text else ""

            vendor_id_elem = info_elem.find('VendorId')
            if vendor_id_elem is not None:
                slave_info['vendor_id'] = int(vendor_id_elem.text)

            product_code_elem = info_elem.find('ProductCode')
            if product_code_elem is not None:
                slave_info['product_code'] = int(product_code_elem.text)

            revision_no_elem = info_elem.find('RevisionNo')
            if revision_no_elem is not None:
                slave_info['revision_no'] = int(revision_no_elem.text)

        return slave_info

    def parse_pdo_entry(self, entry_elem):
        """解析单个PDO条目"""
        entry_info = {}

        # 解析Index
        index_elem = entry_elem.find('Index')
        if index_elem is not None and index_elem.text:
            entry_info['index'] = self.parse_hex_value(index_elem.text)
        else:
            entry_info['index'] = 0x0000

        # 解析SubIndex
        subindex_elem = entry_elem.find('SubIndex')
        if subindex_elem is not None and subindex_elem.text:
            entry_info['subindex'] = int(subindex_elem.text)
        else:
            entry_info['subindex'] = 0x00

        # 解析BitLen
        bitlen_elem = entry_elem.find('BitLen')
        if bitlen_elem is not None and bitlen_elem.text:
            entry_info['bit_length'] = int(bitlen_elem.text)
        else:
            entry_info['bit_length'] = 16

        # 解析Name (作为注释)
        name_elem = entry_elem.find('Name')
        if name_elem is not None and name_elem.text:
            entry_info['name'] = name_elem.text.strip()
        else:
            # 如果Index是0或#x0，标记为Padding
            if entry_info['index'] == 0:
                entry_info['name'] = 'Padding'
            else:
                entry_info['name'] = f'Object_{entry_info["index"]:04X}'

        # 解析DataType
        datatype_elem = entry_elem.find('DataType')
        if datatype_elem is not None and datatype_elem.text:
            entry_info['data_type'] = datatype_elem.text.strip()
        else:
            entry_info['data_type'] = 'UINT'

        # 解析Comment
        comment_elem = entry_elem.find('Comment')
        if comment_elem is not None and comment_elem.text:
            entry_info['comment'] = comment_elem.text.strip()
        else:
            entry_info['comment'] = ''

        return entry_info

    def parse_process_data(self, slave_elem):
        """解析过程数据配置"""
        process_data = {
            'rx_pdos': [],  # 输出PDO (主站->从站)
            'tx_pdos': [],  # 输入PDO (从站->主站)
            'syncs': []
        }

        process_elem = slave_elem.find('ProcessData')
        if process_elem is None:
            return process_data

        # 解析RxPDO (输出)
        for rxpdo_elem in process_elem.findall('RxPdo'):
            pdo_info = {}

            # 解析PDO Index
            index_elem = rxpdo_elem.find('Index')
            if index_elem is not None:
                pdo_info['index'] = self.parse_hex_value(index_elem.text)

            # 解析PDO Name
            name_elem = rxpdo_elem.find('Name')
            if name_elem is not None:
                pdo_info['name'] = name_elem.text.strip() if name_elem.text else ""

            # 解析所有Entry
            entries = []
            for entry_elem in rxpdo_elem.findall('Entry'):
                entry_info = self.parse_pdo_entry(entry_elem)
                entries.append(entry_info)

            pdo_info['entries'] = entries
            process_data['rx_pdos'].append(pdo_info)

        # 解析TxPDO (输入)
        for txpdo_elem in process_elem.findall('TxPdo'):
            pdo_info = {}

            # 解析PDO Index
            index_elem = txpdo_elem.find('Index')
            if index_elem is not None:
                pdo_info['index'] = self.parse_hex_value(index_elem.text)

            # 解析PDO Name
            name_elem = txpdo_elem.find('Name')
            if name_elem is not None:
                pdo_info['name'] = name_elem.text.strip() if name_elem.text else ""

            # 解析所有Entry
            entries = []
            for entry_elem in txpdo_elem.findall('Entry'):
                entry_info = self.parse_pdo_entry(entry_elem)
                entries.append(entry_info)

            pdo_info['entries'] = entries
            process_data['tx_pdos'].append(pdo_info)

        # 解析同步管理器配置
        sm2_elem = process_elem.find('Sm2')
        if sm2_elem is not None:
            sm_info = {
                'index': 2,
                'direction': 'EC_DIR_OUTPUT',
                'type': sm2_elem.find('Type').text if sm2_elem.find('Type') is not None else 'Outputs'
            }
            process_data['syncs'].append(sm_info)

        sm3_elem = process_elem.find('Sm3')
        if sm3_elem is not None:
            sm_info = {
                'index': 3,
                'direction': 'EC_DIR_INPUT',
                'type': sm3_elem.find('Type').text if sm3_elem.find('Type') is not None else 'Inputs'
            }
            process_data['syncs'].append(sm_info)

        return process_data

    def parse_eni(self, eni_file: str) -> bool:
        """解析ENI文件"""
        try:
            tree = ET.parse(eni_file)
            root = tree.getroot()

            # 解析从站配置
            for slave_elem in root.findall('.//Slave'):
                slave_info = self.parse_slave_info(slave_elem)
                process_data = self.parse_process_data(slave_elem)

                slave_config = {
                    'info': slave_info,
                    'process_data': process_data
                }

                self.slaves.append(slave_config)

            return True

        except Exception as e:
            print(f"Error parsing ENI file: {e}")
            import traceback
            traceback.print_exc()
            return False

    def generate_slave_name(self, slave_info):
        """生成从站名称标识符"""
        name = slave_info.get('name', 'slave')
        # 清理名称，只保留字母数字和下划线
        clean_name = ''.join(c if c.isalnum() or c == '_' else '_' for c in name.lower())
        clean_name = clean_name.replace('__', '_').strip('_')

        # 根据产品代码生成后缀
        product_code = slave_info.get('product_code', 0)
        return f'eni_{product_code:04x}'

    def generate_c_code(self) -> str:
        """生成C代码"""
        lines = [
            "/*",
            " * Generated CherryECAT PDO configuration from ENI file",
            " * Auto-generated - do not modify manually",
            " */",
            "",
            "#include \"ec_master.h\"",
            ""
        ]

        for slave_idx, slave in enumerate(self.slaves):
            slave_info = slave['info']
            process_data = slave['process_data']

            slave_name = self.generate_slave_name(slave_info)

            lines.append(f"// Slave {slave_idx + 1}: {slave_info.get('name', 'Unknown')}")
            lines.append(f"// Vendor ID: 0x{slave_info.get('vendor_id', 0):08X}")
            lines.append(f"// Product Code: 0x{slave_info.get('product_code', 0):08X}")
            lines.append("")

            # 生成RxPDO entries (输出)
            rx_entries_generated = set()
            for pdo in process_data['rx_pdos']:
                pdo_index = pdo.get('index', 0)
                pdo_hex = f"{pdo_index:04x}"
                entries_name = f"{slave_name}_output_pdo_entries"

                if entries_name not in rx_entries_generated:
                    lines.append(f"static ec_pdo_entry_info_t {entries_name}[] = {{")

                    # 生成每个entry
                    for entry in pdo.get('entries', []):
                        comment = entry.get('name', 'Padding')
                        lines.append(f"    {{ 0x{entry['index']:04x}, 0x{entry['subindex']:02x}, 0x{entry['bit_length']:02x} }},  // {comment}")

                    lines.append("};")
                    lines.append("")
                    rx_entries_generated.add(entries_name)

            # 生成TxPDO entries (输入)
            tx_entries_generated = set()
            for pdo in process_data['tx_pdos']:
                pdo_index = pdo.get('index', 0)
                pdo_hex = f"{pdo_index:04x}"
                entries_name = f"{slave_name}_input_pdo_entries"

                if entries_name not in tx_entries_generated:
                    lines.append(f"static ec_pdo_entry_info_t {entries_name}[] = {{")

                    # 生成每个entry
                    for entry in pdo.get('entries', []):
                        comment = entry.get('name', 'Padding')
                        lines.append(f"    {{ 0x{entry['index']:04x}, 0x{entry['subindex']:02x}, 0x{entry['bit_length']:02x} }},  // {comment}")

                    lines.append("};")
                    lines.append("")
                    tx_entries_generated.add(entries_name)

            # 生成统一的PDO info数组（合并RxPDO和TxPDO）
            if process_data['rx_pdos'] or process_data['tx_pdos']:
                lines.append(f"static ec_pdo_info_t {slave_name}_pdos[] = {{")

                # 添加RxPDO (输出)
                for pdo in process_data['rx_pdos']:
                    pdo_index = pdo.get('index', 0)
                    entries_name = f"{slave_name}_output_pdo_entries"
                    entry_count = len(pdo.get('entries', []))
                    lines.append(f"    {{ 0x{pdo_index:04x}, {entry_count}, {entries_name} }},")

                # 添加TxPDO (输入)
                for pdo in process_data['tx_pdos']:
                    pdo_index = pdo.get('index', 0)
                    entries_name = f"{slave_name}_input_pdo_entries"
                    entry_count = len(pdo.get('entries', []))
                    lines.append(f"    {{ 0x{pdo_index:04x}, {entry_count}, {entries_name} }},")

                lines.append("};")
                lines.append("")

            # 生成同步管理器配置
            if process_data['rx_pdos'] or process_data['tx_pdos']:
                lines.append(f"static ec_sync_info_t {slave_name}_syncs[] = {{")

                pdo_index = 0  # PDO数组中的索引

                # 添加SM2 (输出)
                if process_data['rx_pdos']:
                    rx_pdo_count = len(process_data['rx_pdos'])
                    lines.append(f"    {{ 2, EC_DIR_OUTPUT, {rx_pdo_count}, &{slave_name}_pdos[{pdo_index}], EC_WD_DISABLE }},")
                    pdo_index += rx_pdo_count

                # 添加SM3 (输入)
                if process_data['tx_pdos']:
                    tx_pdo_count = len(process_data['tx_pdos'])
                    lines.append(f"    {{ 3, EC_DIR_INPUT, {tx_pdo_count}, &{slave_name}_pdos[{pdo_index}], EC_WD_DISABLE }},")

                lines.append("};")
                lines.append("")

        return "\n".join(lines)

def main():
    if len(sys.argv) != 3:
        print("Usage: python eni_parser.py <input.xml> <output.h>")
        print("  input.xml  - ENI XML file")
        print("  output.h   - Output C header file")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' not found")
        sys.exit(1)

    # 创建解析器
    parser = ENIParser()

    # 解析ENI文件
    print(f"Parsing ENI file: {input_file}")
    if not parser.parse_eni(input_file):
        print("Failed to parse ENI file")
        sys.exit(1)

    # 生成C代码
    print("Generating C code...")
    c_code = parser.generate_c_code()

    # 写入输出文件
    try:
        with open(output_file, 'w') as f:
            f.write(c_code)

        print(f"✓ Successfully converted '{input_file}' to '{output_file}'")
        print(f"✓ Generated C code for {len(parser.slaves)} slave(s)")

        # 显示生成的PDO映射信息
        for slave_idx, slave in enumerate(parser.slaves):
            process_data = slave['process_data']
            print(f"✓ Slave {slave_idx + 1}:")
            for pdo in process_data['rx_pdos']:
                print(f"  - RxPDO 0x{pdo.get('index', 0):04X}: {len(pdo.get('entries', []))} entries")
            for pdo in process_data['tx_pdos']:
                print(f"  - TxPDO 0x{pdo.get('index', 0):04X}: {len(pdo.get('entries', []))} entries")

    except Exception as e:
        print(f"Error writing output file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
