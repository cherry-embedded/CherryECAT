**English | [简体中文](README_zh.md)**

<h1 align="center" style="margin: 30px 0 30px; font-weight: bold;">CherryECAT</h1>
<p align="center">
	<a href="https://github.com/cherry-embedded/CherryECAT/releases"><img src="https://img.shields.io/github/release/cherry-embedded/CherryECAT.svg"></a>
	<a href="https://github.com/cherry-embedded/CherryECAT/blob/master/LICENSE"><img src="https://img.shields.io/github/license/cherry-embedded/CherryECAT.svg?style=flat-square"></a>
	<a href="https://github.com/cherry-embedded/CherryECAT/actions/workflows/build_demo.yml"><img src="https://github.com/cherry-embedded/CherryECAT/actions/workflows/build_demo.yml/badge.svg"> </a>
</p>

CherryECAT is a tiny and beautiful, high real-time and low-jitter EtherCAT master stack, specially designed for MCUs running with RTOS.

## Feature

- ~ 4K ram, ~40K flash(24K + 16K shell cmd, including log)
- Asynchronous queue-based transfer (one transfer can carry multiple datagrams)
- Zero-copy technology: directly use enet tx/rx buffer to fill and parse ethercat data
- Support hot-plugging
	- Automatic scanning bus
	- Automatic updating slave information when the topology changes
- Support automatic monitoring slave status
- Support distributed clocks
- Support CANopen over EtherCAT(COE)
- Support File over EtherCAT(FOE)
- Support Ethernet over EtherCAT(EOE)
- Support Slave SII access
- Support Slave register access
- Support multi master
- **Support backup redundancy(TODO)**
- **Minimum PDO cyclic time < 40 us (depends on master and slave hardware)**
- **DC jitter < 3us (depends on master and slave hardware)**
- **Support multi cyclic time(every slave can use different proportional cyclic time)**
- Support ethercat cmd with shell, ref to IgH

The pic shows dc jitter < 3us (hpm6800evk with flash_xip):
![ethercat](docs/assets/ethercat_dc.png)

## Hardware limitations

- **Master**
	- CPU (**Coremark/Mhz > 5.0**, cache > 16K, **memcpy speed > 100MB/s**)
	- ENET must support descriptor dma and iperf with lwip > 90 Mbps
	- Must support High-Precision Timer (jitter < 1us)
	- Must support High-Precision timestamp (ARM DWT/RISC-V MCYCLE)
	- Must support long long print

- **Slave**
	- Must support DC and system time
	- Must support sdo complete access
	- SII must have sync manager information

## Shell cmd

![ethercat](docs/assets/ethercat.png)
![ethercat](docs/assets/ethercat1.png)
![ethercat](docs/assets/ethercat2.png)
![ethercat](docs/assets/ethercat3.png)
![ethercat](docs/assets/ethercat4.png)
![ethercat](docs/assets/ethercat5.png)
![ethercat](docs/assets/ethercat6.png)
![ethercat](docs/assets/ethercat7.png)
![ethercat](docs/assets/ethercat8.png)
![ethercat](docs/assets/ethercat9.png)
![ethercat](docs/assets/ethercat10.png)

## Tool

- esi_parser

Use **esi_parser.py** to generate slave eeprom information and download eeprom to slave.

```
python ./esi_parser.py ECAT_CIA402_ESI.xml eeprom.bin eeprom.h

Parsing XML file: ECAT_CIA402_ESI.xml
Parsed XML: Vendor=0x0048504D, Product=0x00000003
Device Name: ECAT_CIA402
Mailbox RX: 0x1000(128)
Mailbox TX: 0x1080(128)
Generating EEPROM data...
✓ Successfully converted 'ECAT_CIA402_ESI.xml' to 'eeprom.bin'
✓ Generated 2048 bytes of EEPROM data
✓ Vendor ID: 0x0048504D
✓ Product Code: 0x00000003
✓ Revision: 0x00000001
✓ Device Name: ECAT_CIA402
✓ Generated C header file: eeprom.h
```

- eni_parser

Use **eni_parser.py** to generate CherryECAT slave sync config.

```
python ./eni_parser.py ECAT_CIA402_ENI.xml sync_config.h

Parsing ENI file: ECAT_CIA402_ENI.xml
Generating C code...
✓ Successfully converted 'ECAT_CIA402_ENI.xml' to 'sync_config.h'
✓ Generated C code for 1 slave(s)
✓ Slave 1:
  - RxPDO 0x1602: 3 entries
  - TxPDO 0x1A02: 3 entries
```

## Support Boards

- HPM6750EVK2/HPM6800EVK/**HPM5E00EVK**(hybrid internal)
- RT-Thread RZN2L-EtherKit/RA8P1 Titan Board

## Contact

QQ group: 563650597

## License

FOE, EOE and backup redundancy features are available in **Royalty-Free**; other are free to use