# 任务要求
基于dora开发通过页锁内存和DMA高速读取机制完成cpu->cuda跨进程传输tensor的功能。 
请根据整体设计重新进行开发，并达到零拷贝的传输效果（当前传输速度为1800MB/S，目标约为30000MB/S）

# 开发要点（重要）
## docs/pinned_memory_development.md为页锁内存开发文档，当你修改代码或更改架构时，同步在该文件中修改，以方便后来参考。你开发时也可以参考它，以减少盲目思考。
## 每次改完dora的rust底层，你在验证前，需要用cargo build --release --package dora-cli进行编译。
## 我已经将dora/target/release写入环境变量了，因此你可以直接用 dora run xx.yml 进行测试。
## [target_test](../examples/pinned_memory/target_test)文件夹为测试目标，里面的文件，不允许修改。

# 任务步骤
## 步骤一：检查页锁内存是否正确释放
问题：我测试dora run test1.yml时，一开始的速度为8.82it/s，到后面的速度为1.18it/s。
请找出原因，我怀疑是你的页锁内存只是解注册，并未正确释放，导致一开始传输速度很快，到后面越来越慢。
你可以通过在每次注册页锁内存时，memory_manager debug出 pinned_memory_table中有几条页锁内存数据，或者debug出页锁内存的大小占用量，从而发现是哪个环节出了问题。
## 步骤二：自释放测试
我在target_test中新增了自释放测试test5，memory_manager需要在daemon结束时根据pinned_memory_table，将未释放的页锁内存释放，并输出INFO。
发送节点注册五条页锁内存数据，接收节点只接收并释放了2条，需要在daemon结束时，由memory_manager释放剩余三条。
## 步骤三：集成测试
完成前面的任务后，最后统一测试一下所有的测试文件。如果测试过程中有问题，找到原因并修改，直到通过测试。
test1.yml为大型数据速度测试，需要无内存问题，且速度高达30000MB/S。
test2.yml为read_pinned_memory的free=True测试，需要无内存问题。
test3.yml为重复释放测试，需要由节点输出警告“重复释放页锁内存[pinned_buffer],size=[size]”，且程序不崩溃。
test4.yml为释放后读取测试，需要由节点输出警告“读取不存在的页锁内存[pinned_buffer]”，且程序不崩溃。
test5.yml为自释放测试，发送节点注册5条页锁内存数据，接收节点只释放了2条，需要在daemon结束时，由memory_manager释放，并输出INFO“memory manager检测到3条未释放的页锁内存数据，正在释放......”，“memory manager已成功释放3条未释放的页锁内存数据”。
## 步骤四：整理代码
测试任务完成后，应注意精简代码，删除死代码、过程注释和调试信息，整理统一导入声明，清理你创建的不必要的测试文件，以提高可读性。

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
调用该api后，node根据pinned_ptr, metadata将tensor数据注册为页锁内存（尽量只注册，不复制），注册后生成pinned_buffer（跨进程不变的内存标识符）。
然后node将pinned_buffer和metadata作为DaemonRequest发给daemon。
daemon收到DaemonRequest之后，会让memory_manager将pinned_buffer, metadata写入pinned_memory_table；
返回值：pinned_buffer为py.array包装的共享内存标识符。
#### 2.node.read_pinned_memory(pinned_buffer, free) -> data_ptr, metadata
入参1:pinned_buffer为用py.array包装的共享内存标识符。
调用该api后，node会将pinned_buffer作为DaemonRequest发给daemon。
daemon收到DaemonRequest之后，会让memory_manager会根据pinned_buffer查询pinned_memory_table；
查询之后，daemon向node发送DaemonReply（如果查询成功，DaemonReply为查到的metadata；如果没查到，DaemonReply为查询失败枚举）。
node接收到DaemonReply之后，如果为查询失败枚举，则直接输出日志（没有查到页锁内存数据），如果查到，则根据pinned_buffer和metadata将该页锁内存数据通过DMA高速传输至cuda；
传输完后，若入参free为True，则调用free_pinned_memory释放传输前的页锁内存（效果相当于node.free_pinned_memory）；
最后对外返回传输后的cuda数据的当前进程指针和metadata。
返回值1:data_ptr为接收节点的进程指针，供之后接收节点通过ptr_to_torch转换为torch。
返回值2:metadata为字典，与register_pinned_memory时记录的metadata相同。
#### 3.node.free_pinned_memory(pinned_buffer)
入参：pinned_buffer为用py.array包装的共享内存标识符。
调用该api后，node会将pinned_buffer作为DaemonRequest发给daemon。
daemon收到DaemonRequest之后，会让memory_manager会根据pinned_buffer查询pinned_memory_table；
查询之后，daemon向node发送DaemonReply（如果查询成功，DaemonReply为查到的metadata，并在pinned_memory_table删除该条内存记录；如果没查到，DaemonReply为查询失败枚举）。
node接收到DaemonReply之后，如果为查询失败枚举，则直接输出日志（没有查到页锁内存数据），如果为metadata，则根据pinned_buffer和metadata将该页锁内存数据释放；
无返回值。
## 扩展模块（dora.cuda）
该文件只负责tensor的解析和生成，不负责注册、解注册共享页锁内存。除了下面这两个函数之外，不建议加新的函数。
### torch_to_ptr(tensor) -> (pinned_ptr, metadata)
只负责将tensor解析为指针和metadata。
### ptr_to_torch(data_ptr, metadata) -> tensor
只负责根据指针和metadata生成tensor。
对指针的位置进行判断，如果指针在cpu上，则生成cpu上的tensor，如果指针在cuda上，则生成cuda上的tensor。
