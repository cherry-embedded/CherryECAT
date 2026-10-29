API 手册
===========================


ec_master_init
---------------------------

创建并初始化一个主站

.. code-block:: c
   :linenos:

    int ec_master_init(ec_master_t *master, uint8_t master_index);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - master
      - 主站对象指针
    * - master_index
      - 主站索引号，从 0 开始

ec_master_deinit
---------------------------

销毁主站对象，暂不支持

.. code-block:: c
   :linenos:

    void ec_master_deinit(ec_master_t *master);

ec_master_start
---------------------------

启动主站周期任务，所有slave 进入 op 模式，开始周期性执行 PDO 通信。 **此函数需要等待 slave 扫描完成后才能调用，函数内部会检查是否扫描完成，如果没有则死等（会进行任务切换）**。

.. code-block:: c
   :linenos:

    int ec_master_start(ec_master_t *master);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - master
      - 主站对象指针

.. note:: 此时非周期性任务线程关闭，后续非周期性 datagram 将会随着周期性 datagram 一起执行

ec_master_stop
---------------------------

停止主站周期任务，所有 slave 退回到 preop 模式。

.. code-block:: c
   :linenos:

    int ec_master_stop(ec_master_t *master);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - master
      - 主站对象指针

ec_master_queue_ext_datagram
-------------------------------

向队列中发送一个 datagram。调用多次则可以发送多个 datagram，前提是 `waiter` 为 false，或者除最后一个 datagram 外，其他 datagram 的 `waiter` 都为 false。否则会阻塞等待前一个 datagram 发送完成。

.. code-block:: c
   :linenos:

    int ec_master_queue_ext_datagram(ec_master_t *master, ec_datagram_t *datagram, bool wakep_poll, bool waiter);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - master
      - 主站对象指针
    * - datagram
      - 数据报文对象指针， 需要使用 ec_datagram_init 初始化后传入
    * - wakep_poll
      - 是否立刻唤醒非周期性任务线程，立即发送 datagram。如果当前处于周期性通信，则无效
    * - waiter
      - 是否等待当前 datagram 接收完成，阻塞等待
    * - return
      - 函数执行结果，0 表示成功，非 0 表示失败

ec_coe_download
--------------------------------

使用 SDO 下载数据到从站对象字典。

.. code-block:: c
   :linenos:

    int ec_coe_download(ec_master_t *master,
                        uint16_t slave_index,
                        ec_datagram_t *datagram,
                        uint16_t index,
                        uint8_t subindex,
                        const void *buf,
                        uint32_t size,
                        bool complete_access);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - master
      - 主站对象指针
    * - slave_index
      - 从站索引号，从 0 开始
    * - datagram
      - 数据报文对象指针， 需要使用 ec_datagram_init 初始化后传入
    * - index
      - 从站对象字典索引号
    * - subindex
      - 从站对象字典子索引号
    * - buf
      - 指向数据缓冲区的指针
    * - size
      - 缓冲区大小，单位字节
    * - complete_access
      - 是否使用完整访问方式
    * - return
      - 函数执行结果，0 表示成功，非 0 表示失败

ec_coe_upload
--------------------------------

使用 SDO 上传从站对象字典的数据。

.. code-block:: c
   :linenos:

    int ec_coe_upload(ec_master_t *master,
                    uint16_t slave_index,
                    ec_datagram_t *datagram,
                    uint16_t index,
                    uint8_t subindex,
                    const void *buf,
                    uint32_t maxsize,
                    uint32_t *size,
                    bool complete_access);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - master
      - 主站对象指针
    * - slave_index
      - 从站索引号，从 0 开始
    * - datagram
      - 数据报文对象指针， 需要使用 ec_datagram_init 初始化后传入
    * - index
      - 从站对象字典索引号
    * - subindex
      - 从站对象字典子索引号
    * - buf
      - 指向数据缓冲区的指针
    * - maxsize
      - 缓冲区最大大小，单位字节
    * - size
      - 实际上传数据的大小指针，单位字节
    * - complete_access
      - 是否使用完整访问方式
    * - return
      - 函数执行结果，0 表示成功，非 0 表示失败

ec_foe_write
--------------------------------

使用 FOE 写文件到从站。

.. code-block:: c
   :linenos:

  int ec_foe_write(ec_master_t *master,
                  uint16_t slave_index,
                  ec_datagram_t *datagram,
                  const char *filename,
                  uint32_t password,
                  const void *buf,
                  uint32_t size,
                  foe_write_read_hook_t write_hook);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - master
      - 主站对象指针
    * - slave_index
      - 从站索引号，从 0 开始
    * - datagram
      - 数据报文对象指针， 需要使用 ec_datagram_init 初始化后传入
    * - filename
      - 文件名字符串指针
    * - password
      - 文件访问密码
    * - buf
      - 指向数据缓冲区的指针
    * - size
      - 缓冲区大小，单位字节。要求大小为整个文件的大小，不能分段写入
    * - write_hook
      - 写入回调函数指针。使用分段写入功能时使用
    * - return
      - 函数执行结果，0 表示成功，非 0 表示失败

ec_foe_read
--------------------------------

使用 FOE 从从站读文件。

.. code-block:: c
   :linenos:

  int ec_foe_read(ec_master_t *master,
                  uint16_t slave_index,
                  ec_datagram_t *datagram,
                  const char *filename,
                  uint32_t password,
                  void *buf,
                  uint32_t maxsize,
                  uint32_t *size,
                  foe_write_read_hook_t read_hook);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - master
      - 主站对象指针
    * - slave_index
      - 从站索引号，从 0 开始
    * - datagram
      - 数据报文对象指针， 需要使用 ec_datagram_init 初始化后传入
    * - filename
      - 文件名字符串指针
    * - password
      - 文件访问密码
    * - buf
      - 指向数据缓冲区的指针
    * - maxsize
      - 缓冲区最大大小，单位字节
    * - size
      - 实际读取数据的大小指针，单位字节。要求大小为整个文件的大小，不能分段写入
    * - read_hook
      - 读取回调函数指针。使用分段读取功能时使用
    * - return
      - 函数执行结果，0 表示成功，非 0 表示失败

ec_eoe_start
--------------------------------

配置主从站 IPv4 并启动 EoE 通信。当前仅支持同一时刻，只能允许一个 EOE 从站通信，并且推荐在 preop 下进行（如果对性能没有要求，则忽视）。

.. code-block:: c
   :linenos:

    int ec_eoe_start(ec_eoe_t *eoe,
                    ec_master_t *master,
                    uint16_t slave_index,
                    struct ec_eoe_ip_param *master_ip_param,
                    struct ec_eoe_ip_param *slave_ip_param);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - ec_eoe_t
      - EoE 对象指针
    * - master
      - 主站对象指针
    * - slave_index
      - 从站索引号，从 0 开始
    * - master_ip_param
      - 主站 IP 参数指针
    * - slave_ip_param
      - 从站 IP 参数指针
    * - return
      - 函数执行结果，0 表示成功，非 0 表示失败

.. note::

    主站和从站需要在同一个网关下才能通信成功，并且 mac 地址需要不同。

ec_datagram_init
--------------------------------

初始化数据报文对象，datagram 内存采用动态初始化。

.. code-block:: c
   :linenos:

    void ec_datagram_init(ec_datagram_t *datagram, size_t mem_size);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - datagram
      - 数据报文对象指针
    * - mem_size
      - 数据报文内存大小，单位字节

ec_datagram_init_static
--------------------------------

初始化数据报文对象，datagram 内存采用静态初始化。

.. code-block:: c
   :linenos:

    void ec_datagram_init_static(ec_datagram_t *datagram, uint8_t *data, size_t mem_size);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - datagram
      - 数据报文对象指针
    * - data
      - 指向数据缓冲区的指针
    * - mem_size
      - 数据报文内存大小，单位字节

ec_datagram_clear
--------------------------------

清理数据报文对象，释放动态分配的内存。

.. code-block:: c
   :linenos:

    void ec_datagram_clear(ec_datagram_t *datagram);

.. list-table::
    :widths: 10 10
    :header-rows: 1

    * - parameter
      - description
    * - datagram
      - 数据报文对象指针
