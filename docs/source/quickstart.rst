快速入门
===========================

本节主要介绍如何使用 CherryECAT。当前我们推荐在以下开发板中使用 CherryECAT。

- HPMicro 系列开发板
- RT-Thread Titan、 etherkit、RuiQing 开发板

ethercat 命令行
-----------------

我们提供了一个简单的命令行工具 `ethercat` 用于测试 CherryECAT 功能。为了方便使用过 IgH 的 linux 用户过渡，格式部分参考的是 IgH。

- **ethercat start**: 启动主站 PDO 定时器，切换所有 slaves 到 OP 状态，后续参数依次为主站定时器周期（单位微秒）和 CIA402 模式（可选，默认为 0），支持的 CIA402 模式有：

.. code-block:: c
   :linenos:

    #define EC_CIA402_OPERATION_MODE_NO_MODE 0x00
    #define EC_CIA402_OPERATION_MODE_CSP     0x01
    #define EC_CIA402_OPERATION_MODE_CSV     0x02
    #define EC_CIA402_OPERATION_MODE_CSP_CSV 0x03
    #define EC_CIA402_OPERATION_MODE_CST     0x04
    #define EC_CIA402_OPERATION_MODE_HOME    0x05

- **ethercat stop**: 停止主站 PDO 定时器，切换所有 slaves 到 PREOP 状态
- **ethercat master**: 用于查看主站状态，数据统计等信息
- **ethercat rescan**: 重新扫描总线
- **ethercat slaves**: 查看从站信息，使用 `-p` 参数选择指定从站，使用 `-v` 参数查看详细信息
- **ethercat pdos**: 查看 PDO 映射信息，使用 `-p` 参数选择指定从站
- **ethercat states**: 请求从站状态转换， 使用 `-p` 参数选择指定从站，状态需要从 `ec_slave_state_t` 枚举中选择并以十六进制输入
- **ethercat coe_read**: 通过 CoE 读取 SDO, 使用 `-p` 参数选择指定从站，后续参数依次为 index, subindex, 如果忽略 subindex 则使用 sdo complete 模式
- **ethercat coe_write**: 通过 CoE 写入 SDO，使用 `-p` 参数选择指定从站，后续参数依次为 index, subindex, data，不使用 sdo complete 模式
- **ethercat foe_read**: 通过 FoE 读取文件, 使用 `-p` 参数选择指定从站，后续参数依次为 filename, pwd 和 16进制数组，数组从低位到高位输入
- **ethercat foe_write**: 通过 FoE 写入文件，使用 `-p` 参数选择指定从站，后续参数依次为 filename, pwd 和 16进制数组，数组从低位到高位输入
- **ethercat eoe_start**: 启动 EoE 功能
- **ethercat pdo_read -p <idx>**: 读取指定从站的输入和输出 PDO 十六进制数据，索引从 0 开始。需先启动主站，命令直接从 PDO 映射地址分别打印输出 RxPDO 和输入 TxPDO，不复制到临时缓冲区。
- **ethercat sii_read**: 读取 SII 数据， 使用 `-p` 参数选择指定从站
- **ethercat sii_write**: 写入 SII 数据， 使用 `-p` 参数选择指定从站
- **ethercat wc**: 查看主站工作计数器
- **ethercat perf**: 性能测试， 使用 `-s` 参数开始，使用 `-v` 参数查看测试结果，使用 `-d` 参数停止测试

典型使用流程
-----------------

- 调用 `ec_master_init` 初始化主站
- 为每个 slave 配置 config 参数，包括 PDO 映射、DC 同步等， `ec_slave_config_t` 结构体定义如下：

.. code-block:: c
   :linenos:

    typedef struct {
        ec_sync_info_t *sync;                           /**< Sync manager configuration. */
        uint8_t sync_count;                             /**< Number of sync managers. */
        ec_pdo_callback_t pdo_callback;                 /**< PDO process data callback. */
        uint16_t dc_assign_activate;                    /**< dc assign control */
        ec_sync_signal_t dc_sync[EC_SYNC_SIGNAL_COUNT]; /**< DC sync signals. */
    } ec_slave_config_t;


    // 举例从协议栈提供的部分 slave 信息中寻找 sync 信息，并进行配置
    for (uint32_t i = 0; i < global_cmd_master->slave_count; i++) {
        ret = ec_master_find_slave_sync_info(global_cmd_master->slaves[i].sii.vendor_id,
                                              global_cmd_master->slaves[i].sii.product_code,
                                              global_cmd_master->slaves[i].sii.revision_number,
                                              motor_mode,
                                              &slave_config[i].sync,
                                              &slave_config[i].sync_count);
        if (ret != 0) {
            EC_LOG_ERR("Failed to find slave sync info: vendor_id=0x%08x, product_code=0x%08x\n",
                        global_cmd_master->slaves[i].sii.vendor_id,
                        global_cmd_master->slaves[i].sii.product_code);
            return -1;
        }

        slave_config[i].dc_assign_activate = 0x300;

        slave_config[i].dc_sync[0].cycle_time = atoi(argv[1]) * 1000;
        slave_config[i].dc_sync[0].shift_time = 1000000;
        slave_config[i].dc_sync[1].cycle_time = 0;
        slave_config[i].dc_sync[1].shift_time = 0;
        slave_config[i].pdo_callback = ec_pdo_callback;

        global_cmd_master->slaves[i].config = &slave_config[i];
    }

.. note:: 定义的 config 变量需要是全局变量

- 配置主站 master 相关参数，包括 `cycle_time`、 `shift_time`、 `dc_sync_with_dc_ref_enable` 等

  - `cycle_time` 代表主站定时器的周期，单位为纳秒。
  - `shift_time` 代表相对于 cycle_time 的滞后时间，单位为纳秒，通常设置为 20% 的 cycle_time。
  - `dc_sync_with_dc_ref_enable` 代表 DC 同步是与主站时钟同步还是和第一个带有 DC 的从站同步。通常推荐和从站同步。

- 最后调用 `ec_master_start` 启动周期性传输，如果从站没有扫描就调用，则会一直死等，直到所有从站扫描完成。

.. code-block:: c
   :linenos:

    global_cmd_master->cycle_time = atoi(argv[2]) * 1000;       // cycle time in ns
    global_cmd_master->shift_time = atoi(argv[2]) * 1000 * 0.2; // 20% shift time in ns
    global_cmd_master->dc_sync_with_dc_ref_enable = true;       // enable DC sync with dc reference clock
    ec_master_start(global_cmd_master);

- PDO 数据统一在配置的 ``pdo_callback`` 中处理：从 ``input`` 读取输入，向 ``output`` 写入下一周期的输出，长度分别为 ``slave->idata_size`` 和 ``slave->odata_size``。回调不能阻塞，应用应在回调中读写 PDO。诊断命令 ``pdo_read -p <idx>`` 直接读取 PDO 映射地址，打印期间数据仍可能随通信周期更新。

HPMicro IO 示例
-----------------------

示例支持 HPM IO 从站（VID ``0x0048504D``，PID ``0x00000001`` / ``0x00000005``）。
当前设备表使用 ``0x1600`` / ``0x1A00`` PDO 映射，主站输出和输入各占 4 字节。
以下示例假设总线上只有一个 IO 从站，其索引为 0：

.. code-block:: text

    ethercat start 1000
    ethercat pdo_read -p 0
    ethercat stop

``1000`` 表示通信周期为 1000 微秒

HPMicro demo 的 ``ec_pdo_callback`` 在 IO 从站处于 OP 时，将 32 位输入值复制到输出，
核心处理如下：

.. code-block:: c

    uint32_t value = EC_READ_U32(input);
    EC_WRITE_U32(output, value);

如果需要测试固定输出，可将 IO 分支的上述处理替换为：

.. code-block:: c

    EC_WRITE_U32(output, 0x00000001U);

重新编译运行后，通过 ``ethercat pdo_read -p 0`` 查看输出区，固定值 1 的字节序列为
``01 00 00 00``。``output`` 为主站发送给从站的数据，``input`` 为从站返回给主站的数据。

HPMicro CIA402 电机示例
-----------------------

示例支持 HPM CIA402 从站（VID ``0x0048504D``，PID ``0x00000003`` / ``0x00000004``），
使用设备表中的 8 字节 CSV/CSP PDO 映射。三个命令均通过 ``-p`` 指定从站索引（0 到 31）：

- ``motor_control -p <idx> <0|1>``：关闭或使能电机。命令仅更新 enable 变量。回调在 disable 时直接发送 Disable voltage；enable 时才按状态字依次发送 Shutdown、Switch on、Enable operation，遇到 Fault 时发送一次 Fault reset。关闭时保留速度和位置指令。
- ``speed_control -p <idx> <value>``：设置 CSV 目标速度（0x60FF），支持有符号 32 位整数。
- ``position_control -p <idx> <value>``：设置 CSP 绝对目标位置（0x607A），支持有符号 32 位整数。

速度和位置使用从站对象字典的原始单位。命令只更新目标值，PDO 读写仍在 ``ec_pdo_callback`` 中完成。
上电时默认关闭电机，重启周期通信不会清除命令变量；未下发位置指令时，CSP 目标跟随实际位置，避免默认目标为零。
位置命令直接设置目标，不包含轨迹插补或速度、加速度限制。

``ethercat start`` 的模式参数仅选择主站 PDO 映射，不会自动写入从站的 ``0x6060``。
当前 HPM CIA402 从站的 ``0x6060`` 默认值为 0，选择 ``0x1601`` 或 ``0x1602`` 映射也不会
自动切换运行模式，因此需要在 PREOP 阶段通过 CoE 写入 ``0x6060:00``：

.. list-table:: CSP / CSV 模式设置
   :header-rows: 1
   :widths: 15 25 30 30

   * - 模式
     - ``start`` 模式参数
     - RxPDO / TxPDO
     - ``0x6060:00``
   * - CSP
     - 1
     - ``0x1601`` / ``0x1A01``
     - 8（``08``）
   * - CSV
     - 2
     - ``0x1602`` / ``0x1A02``
     - 9（``09``）

``coe_write`` 的 index、subindex、data 均按十六进制解析，``08`` / ``09`` 会作为 1 字节写入。
启动通信并进入 OP 后，读取 ``0x6061:00`` 确认实际运行模式，再使能电机。
以下示例假设电机从站索引为 0；如果 IO 从站为 0、电机从站为 1，将电机相关命令中的
``-p 0`` 改为 ``-p 1``，``ethercat start`` 仍会启动总线上的全部从站。

CSV 示例（``0x6060 = 9``，``ethercat start`` 的模式参数为 2）：

.. code-block:: text

    ethercat coe_write -p 0 6060 00 09
    ethercat start 1000 2
    ethercat coe_read -p 0 6061 00
    motor_control -p 0 1
    speed_control -p 0 100
    ethercat pdo_read -p 0
    speed_control -p 0 0
    motor_control -p 0 0
    ethercat stop

读取的 ``0x6061`` 应为 ``09``。``speed_control`` 设置目标速度，不能代替 ``0x6060`` 的模式设置。

CSP 示例（``0x6060 = 8``，``ethercat start`` 的模式参数为 1）：

.. code-block:: text

    ethercat coe_write -p 0 6060 00 08
    ethercat start 1000 1
    ethercat coe_read -p 0 6061 00
    motor_control -p 0 1
    position_control -p 0 1000
    ethercat pdo_read -p 0
    motor_control -p 0 0
    ethercat stop

读取的 ``0x6061`` 应为 ``08``。切换模式前先关闭电机并执行 ``ethercat stop``，
回到 PREOP 后重新写入 ``0x6060``，再使用对应的模式参数启动通信。
