# 当前进度
1、基于dora开发通过页锁内存完成cpu->cuda跨进程传输tensor的功能
2、目前基本跑通，传输速度1000MB/S，校验接收方数据与发送方相等，接收方的tensor在cuda上。

# 待解决问题
## 释放页锁内存的函数node.free_pinned_memory并没有达到预期效果
测试dora run examples/pinned_memory/cpu2cuda.yml时,产生了大量的12:39:40 stdout  receiver_node:  /home/tcr/.conda/envs/my_conda/lib/python3.11/multiprocessing/resource_tracker.py:267: UserWarning: resource_tracker: '/dora_pinned_32afbf11-05f8-48e2-9f90-947f83d0d7de': [Errno 2] No such file or directory: '/dora_pinned_32afbf11-05f8-48e2-9f90-947f83d0d7de'
12:39:40 stdout  receiver_node:    warnings.warn('resource_tracker: %r: %s' % (name, e))。
疑似是没有成功释放页锁内存，导致内存泄漏。
## 未达到零拷贝的效果，传输速度过慢。
当cpu2cuda.yml的环境参数mode为ipc时，使用IPC handle传输数据，速度可达30000MB/S。
在我们的页锁内存中，应达到零拷贝的效果，速度也应该达到数万MB/S
思路：可能因为没有使用 DMA 等高速读取机制，也有可能在torch_to_pinned_buffer中使用numpy中转，导致多次拷贝。
## 删除dora.cuda中的冗余函数
dora.cuda中，你新增了_cuda_free_host、_cleanup_pinned_memory函数，疑似用来释放页锁内存。这些函数的功能与dora的释放页锁内存函数重叠，会导致代码可读性很差，应当删去。
页锁内存的解注册和释放，都应由dora统一管理，由free_pinned_memory完成。
dora.cuda中的torch_to_pinned_buffer和pinned_buffer_to_torch仅负责辅助dora解析和生成tensor张量。
## 修改读取页锁内存函数的入参
观察到pinned_buffer_to_torch有一个入参为free_source，删去这个入参。
为read_pinned_memory增加入参free，调用该函数时，使用DMA机制高速读取页锁内存至cuda，若free为True，则调用一次free_pinned_memory将原来的页锁内存释放。若free为True，则不进行任何操作。

# 测试
## 测试方法
在dora/examples/pinned_memory文件夹下进行测试。
改完dora底层之后，你可以用cargo build --release --package dora-cli进行编译。
我已经将dora/target/release写入环境变量了，因此你可以直接在pinned_memory文件夹下用dora run cpu2cuda.yml进行测试。


