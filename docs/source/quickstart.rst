快速入门
===========================

本节主要介绍如何使用 CherryECAT。当前我们推荐在以下开发板中使用 CherryECAT。

- HPMicro 系列开发板
- RT-Thread Titan、 etherkit、RuiQing 开发板

ethercat 命令行
-----------------

我们提供了一个简单的命令行工具 `ethercat` 用于测试 CherryECAT 功能。为了方便使用过 IgH 的 linux 用户过渡，格式部分参考的是 IgH。

- ethercat master: 用于查看主站状态，数据统计等信息
- ethercat rescan: 重新扫描总线
- ethercat slaves: 查看从站信息，使用 `-p` 参数选择指定从站，使用 `-v` 参数查看详细信息
- ethercat pdos: 查看 PDO 映射信息，使用 `-p` 参数选择指定从站
- ethercat states: 请求从站状态转换， 使用 `-p` 参数选择指定从站，状态需要从 `ec_slave_state_t` 枚举中选择并以十六进制输入
- ethercat coe_read: 通过 CoE 读取 SDO, 使用 `-p` 参数选择指定从站，后续参数依次为 index, subindex, 如果忽略 subindex 则使用 sdo complete 模式
- ethercat coe_write: 通过 CoE 写入 SDO，使用 `-p` 参数选择指定从站，后续参数依次为 index, subindex, data，不使用 sdo complete 模式
- ethercat foe_read: 通过 FoE 读取文件, 使用 `-p` 参数选择指定从站，后续参数依次为 filename, pwd 和 16进制数组，数组从低位到高位输入
- ethercat foe_write: 通过 FoE 写入文件，使用 `-p` 参数选择指定从站，后续参数依次为 filename, pwd 和 16进制数组，数组从低位到高位输入
- ethercat eoe_start: 启动 EoE 功能
- ethercat pdo_read: 读取过程数据，使用 `-p` 参数选择指定从站
- ethercat pdo_write: 写入过程数据，使用 `-p` 参数选择指定从站，后续参数依次为 offset 和 16进制数组，数组从低位到高位输入，offset 表示写入数据在 PDO 中的偏移位置
- ethercat sii_read: 读取 SII 数据， 使用 `-p` 参数选择指定从站
- ethercat sii_write: 写入 SII 数据， 使用 `-p` 参数选择指定从站
- ethercat wc: 查看主站工作计数器
- ethercat perf: 性能测试， 使用 `-s` 参数开始，使用 `-v` 参数查看测试结果，使用 `-d` 参数停止测试
- ethercat timediff: 系统时间差监测，使用 `-s` 参数开始，使用 `-v` 参数查看测试结果，使用 `-d` 参数停止测试

典型使用流程
-----------------

ethercat 命令行中并没有提供启动周期性传输的命令，因为需要用户配置 slave 的一些参数后才能使用周期性传输功能。下面是一个典型的使用流程：

- 调用 `ec_master_init` 初始化主站
- 为每个 slave 配置 config 参数，包括 PDO 映射、DC 同步等， `ec_slave_config_t` 结构体定义如下：

.. code-block:: c
   :linenos:

    ec_sync_info_t *sync;                           /**< Sync manager configuration. */
    uint8_t sync_count;                             /**< Number of sync managers. */
    ec_pdo_callback_t pdo_callback;                 /**< PDO process data callback. */
    uint16_t dc_assign_activate;                    /**< dc assign control */
    ec_sync_signal_t dc_sync[EC_SYNC_SIGNAL_COUNT]; /**< DC sync signals. */

- 配置主站 master 相关参数，包括 `cycle_time`、 `shift_time`、`dc_sync_with_dc_ref_enable` 等

`cycle_time` 代表主站定时器的周期，单位为纳秒。 `shift_time` 代表相对于 cycle_time 的滞后时间，单位为纳秒，通常设置为 20% 的 cycle_time。 `dc_sync_with_dc_ref_enable` 代表 DC 同步是与主站时钟同步还是和第一个带有 DC 的从站同步。

- 最后调用 `ec_master_start` 启动周期性传输，如果从站没有扫描就调用，则会一直死等，直到所有从站扫描完成。

- 对于时间敏感的 PDO 处理，可以在设置的 `pdo_callback` 中进行。对于不敏感的，异步的处理，则可以使用 `ec_master_get_slave_domain_*` 系列函数进行读写。