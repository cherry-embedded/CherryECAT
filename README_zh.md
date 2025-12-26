**[English](README.md) | 简体中文**

<h1 align="center" style="margin: 30px 0 30px; font-weight: bold;">CherryECAT</h1>
<p align="center">
	<a href="https://github.com/cherry-embedded/CherryECAT/releases"><img src="https://img.shields.io/github/release/cherry-embedded/CherryECAT.svg"></a>
	<a href="https://github.com/cherry-embedded/CherryECAT/blob/master/LICENSE"><img src="https://img.shields.io/github/license/cherry-embedded/CherryECAT.svg?style=flat-square"></a>
	<a href="https://github.com/cherry-embedded/CherryECAT/actions/workflows/build_demo.yml"><img src="https://github.com/cherry-embedded/CherryECAT/actions/workflows/build_demo.yml/badge.svg"> </a>
</p>

CherryECAT 是一个小而美的、高实时性、低抖动的 EtherCAT 主机协议栈，专为跑在 RTOS 下的 MCU 设计。

## 特性

- ~ 4K ram，~40K flash（24K + 16K shell cmd, including log）
- 异步队列式传输（一次传输可以携带多个 datagram）
- 零拷贝技术：直接使用 enet tx/rx buffer 填充和解析 ethercat 数据
- 支持热插拔
	- 自动扫描总线
	- 拓扑结构发生变化时自动更新 Slave 信息
- 支持自动监控 Slave 状态
- 支持分布式时钟
- 支持 CANopen over EtherCAT (COE)
- 支持 File over EtherCAT(FOE)
- 支持 Ethernet over EtherCAT(EOE)
- 支持 Slave SII 读写
- 支持 Slave 寄存器读写
- 支持多主站
- **支持备份冗余(TODO)**
- **最小 PDO cyclic time < 40 us (实际数值受主站硬件和从站硬件影响)**
- **DC 抖动 < 3us (实际数值受主站硬件和从站硬件影响)**
- **支持多周期（每个从站可以使用不同的成比例的周期）**
- 支持 ethercat 命令行交互，参考 IgH

下图展示 dc 抖动 < 3us （hpm6800evk + flash_xip）:
![ethercat](docs/assets/ethercat_dc.png)

## 硬件限制

- 主站
	- CPU (**Coremark/Mhz > 5.0**, cache > 16K, **memcpy speed > 100MB/s**)
	- 以太网必须支持 descriptor dma 并且 iperf with lwip > 90 Mbps
	- 必须支持高精度定时器（抖动小于 1us）
	- 必须支持高精度时间戳 (ARM DWT/RISC-V MCYCLE)
	- 必须支持 64 位打印

- 从站
	- 必须支持 DC 和 system time 功能
	- 必须支持 sdo complete access
	- SII 必须携带 sync manager 信息

## Shell 命令

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

## 工具

- esi_parser

使用 **esi_parser.py** 生成从站 eeprom 信息用于烧录从站

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

使用 **eni_parser.py** 生成 CherryECAT slave sync 配置

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

## 支持的开发板

- HPM6750EVK2/HPM6800EVK/**HPM5E00EVK**(hybrid internal)
- RT-Thread RZN2L-EtherKit/RA8P1 Titan Board

## 联系

QQ group: 563650597

## License

FOE，EOE 以及备份冗余功能采用 **免版税授权** 性质，其余功能可免费商用