# 任务要求
基于dora开发通过页锁内存和DMA高速读取机制完成cpu->cuda跨进程传输tensor的功能。 
请根据整体设计重新进行开发，并达到零拷贝的传输效果（传输速度约为30000MB/S）
## 每次改完dora的rust底层，你在验证前，需要用cargo build --release --package dora-cli进行编译。
## 测试通过指标：dora run cpu2cuda.yml 成功传输数据。
## cpu_sender.py和cuda_receiver.py为测试通过指标框架，不允许修改。
# 任务步骤
## 步骤一：分析整体设计的可行性，若不可行，则应停止开发，并提出设计建议。->（你上次未提出意见，应该可行）
## 步骤二：扩展模块（dora.cuda）开发 -> (已完成)
测试地点：dora/examples/pinned_memory/test_1文件夹
测试内容：一个tensor经过torch_to_pinned_ptr和pinned_ptr_to_torch之后，能否正确生成tensor。若与原来的tensor相等，则表示扩展模块开发完成。
完成该步骤之后，将不允许修改dora.cuda.py文件。
## 步骤三：内存管理（dora核心）开发 -> （已完成。问题已解决：/dev/shm文件系统空间已满导致段错误，清理后集成测试通过。）
测试地点：dora/examples/pinned_memory/test_2文件夹
测试内容：1.测试node.register_pinned_memory是否能正确注册页锁内存，并返回有效的pinned_buffer。
2.pinned_buffer能否被node.read_pinned_memory在pinned_memory_table中查到，并读取有效指针和metadata。
3.pinned_buffer能否被node.free_pinned_memory在pinned_memory_table中查到，并正确释放内存（无内存错误）。
你曾经擅自在dora.cuda生成free_pinned_buffer，已被删去，请移除项目中其他地方对free_pinned_buffer的依赖
## 步骤四：集成测试 -> （已完成。测试通过，平均传输速率为1518.048584MB/s。）
步骤二和三成功之后，测试dora/examples/pinned_momory/cpu2cuda.yml
我已经将dora/target/release写入环境变量了，因此你可以直接在pinned_memory文件夹下用dora run cpu2cuda.yml进行测试。
如果测试过程中有问题，找到原因，并修改，直到通过测试。
## 步骤五：整理代码 -> （已完成。已删除调试信息，精简代码，修复编译警告。）
测试任务完成后，应注意精简代码，删除过程注释和调试信息，整理统一导入声明，以提高可读性。
# 整体设计
## dora核心（rust）
### memory_manager
功能函数写在dora/binaries/daemon/src/memory_manager.rs中，主要负责共享页锁内存的注册和释放，以及根据内存标识符转换为接受节点进程的指针，以达到跨进程零拷贝传输数据的效果。
daemon初始化时，初始化pinned_memory_table。
daemon结束时，自动调用free_all_memory函数，查询pinned_memory_table，对每个未释放的内存记录，执行free_pinned_memory。
### python API
#### 1.node.register_pinned_memory(pinned_ptr, metadata) -> pinned_buffer
入参1：pinned_ptr为cpu上的tensor的指针。
入参2：metadata为字典，包括tensor的ptr,size,dtype,shape等。
调用该api后，memory_manager根据pinned_ptr, metadata将tensor数据注册为共享页锁内存；
然后在pinned_memory_table中记录pinned_buffer和metadata，
返回值：pinned_buffer为注册好的页锁内存标识符。
#### 2.node.read_pinned_memory(pinned_buffer, free) -> pinned_ptr, metadata
入参1:pinned_buffer为用py.array包装的共享内存标识符。
调用该api后，memory_manager会根据pinned_buffer查询pinned_memory_table；
若查到，则根据表中数据填充metadata,并将该页锁内存数据通过DMA高速传输至cuda；
传输完后，若入参free为True，则调用free_pinned_memory释放原来的页锁内存；
若在pinned_memory_table没查到pinned_buffer，则输出提示。
返回值1:pinned_ptr为接收方进程的指针，供之后接收方通过pinned_ptr_to_torch转换为torch。
返回值2:metadata为字典，与register_pinned_memory时记录的metadata相同。
#### 3.node.free_pinned_memory(pinned_buffer)
入参：pinned_buffer为用py.array包装的共享内存标识符。
调用该api后，memory_manager会根据pinned_buffer查询pinned_memory_table；
若查到，则释放共享页锁内存，并在pinned_memory_table删除该条内存记录。
若在pinned_memory_table没查到pinned_buffer，则输出提示。
无返回值。
## 扩展函数（dora.cuda）
该文件只负责tensor的解析和生成，不负责注册、解注册共享页锁内存。除了下面这两个函数之外，不建议加新的函数。
### torch_to_pinned_ptr(tensor) -> (pinned_ptr, metadata)
只负责将tensor解析为指针和metadata。
### pinned_ptr_to_torch(pinned_ptr, metadata) -> tensor
只负责根据指针和metadata生成tensor。

