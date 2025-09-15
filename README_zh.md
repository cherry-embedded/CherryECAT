**[English](README.md) | 简体中文**

<h1 align="center" style="margin: 30px 0 30px; font-weight: bold;">CherryECAT</h1>
<p align="center">
	<a href="https://github.com/cherry-embedded/CherryECAT/releases"><img src="https://img.shields.io/github/release/cherry-embedded/CherryECAT.svg"></a>
	<a href="https://github.com/cherry-embedded/CherryECAT/blob/master/LICENSE"><img src="https://img.shields.io/github/license/cherry-embedded/CherryECAT.svg?style=flat-square"></a>
</p>

CherryECAT 是一个小而美的、高实时性、低抖动的 EtherCAT 主机协议栈，专为跑在 RTOS 下的 MCU 设计。

## 特性

- ~ 4K ram，~32K flash（24K + 8K shell cmd + debug log）
- 异步队列式传输（一次传输可以携带多个 datagram）
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
- 支持备份冗余
- 最小 PDO cyclic time < 40 us (实际数值受主站硬件和从站硬件影响)
- 支持 ethercat 命令行交互，参考 IgH

## 硬件限制

- 主站
	- CPU (cache > 16K, memcpy speed > 100MB/s)
	- 以太网必须支持 descriptor dma 并且 iperf with lwip > 90 Mbps
	- 代码必须跑在 ram 上，如果不使用 DC 同步则忽视
	- 必须支持高精度定时器（抖动小于 1us）
	- 必须支持高精度时间戳 (比如 ARM DWT)
	- 必须支持 64 位打印

- 从站
	- 必须支持 COE
	- 必须支持 sdo complete access
	- SII 必须携带 sync manager 信息

## Shell 命令

![ethercat](docs/assets/ethercat.png)
![ethercat](docs/assets/ethercat2.png)
![ethercat](docs/assets/ethercat3.png)
![ethercat](docs/assets/ethercat4.png)
![ethercat](docs/assets/ethercat5.png)

## 支持的开发板

- HPM6750EVK2/HPM6800EVK/**HPM5E00EVK**(hybrid internal)

## 联系

QQ group: 563650597

## License

FOE，EOE，备份冗余功能为商用收费，其余功能免费商用