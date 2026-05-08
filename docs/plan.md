# 当前情况
## 基于dora开发通过页锁内存和DMA高速读取机制完成cpu->cuda跨进程传输tensor的功能，并达到零拷贝的传输效果（当前传输速度为6000MB/S）
## 目前的api设计仍有问题，比如read_memory的free入参、free_memory并不实际释放。
## 我重构一下页锁内存池的设计，请基于整体设计，完成开发任务。

# 开发要点（重要）
## docs/pinned_memory_development.md为页锁内存开发文档，当你修改代码或更改架构时，同步在该文件中修改，以方便后来参考。你开发时也可以参考它，以减少盲目思考。
## 每次改完dora的rust底层，你在验证前，需要用cargo build --release --package dora-cli进行编译。
## 我已经将dora/target/release写入环境变量了，因此你可以直接用 dora run xx.yml 进行测试。
## examples/memory_pool中的文件为测试目标，原则上不允许修改。但你在开发过程中可以在examples/memory_pool/temporary_test中进行测试
## 开发过程中，代码风格、注释风格应与dora原来的代码一致，并且能复用原来的函数尽可能复用，但不要改与页锁内存无关的代码。

# 开发任务
## 完善扩展模块
我将dora.cuda中的ptr_to_torch和torch_to_ptr重新设计为get_tensor_info和tensor_from_info，以简化入参。
请帮我完善这两个函数，并保证一个tensor经过et_tensor_info和tensor_from_info生成的另一个tensor为同一个数据（零拷贝）。
## 重构dora核心的页锁内存相关代码
我需要重新设计页锁内存api如pinned_memory_to_cuda、pinned_memory_to_cpu、read_memory、get_memory重新设计了，请根据整体设计，对其进行修改。
apis/python/node/dora/__init__.pyi为dora的python api声明，同步修改，并保证注释风格与其它的一样。
## 目标测试
完成前面的任务后，最后统一测试一下所有的测试文件。如果测试过程中有问题，找到原因并修改，直到通过测试。
coy2cda.yml为正向速度测试，cpu发送，cuda接收，需要无内存问题，且速度不低于原来的6000MB/S。
cuda2cpu.yml为反向速度测试，cuda发送，cpu接收，需要无内存问题，且速度不低于原来的7000MB/S。
初次之外，仿照这两个测试，帮我进行如下测试:
重复释放测试，接收节点连续两次node.free_memory_pool,需要由节点输出警告“释放不存在的内存池[memory_pool_id]”，且程序不崩溃。
空读测试，接收节点node.free_memory_pool后node.read_memory_pool，需要由节点输出警告“读取不存在的内存池[memory_pool_id]”，且程序不崩溃。
空写测试，发送节点node.free_memory_pool后node.write_memory_pool，需要由节点输出警告“写入不存在的内存池[memory_pool_id]”，且程序不崩溃。
自动释放测试，接收节点不node.free_memory_pool，需要在daemon结束时，由memory_manager释放，并输出INFO“memory manager检测到xx条未释放的页锁内存数据，正在释放......”，“memory manager已成功释放xx条未释放的页锁内存数据”。
## 整理代码和文档
测试任务完成后，应注意精简代码，删除死代码、过程注释和调试信息，整理统一导入声明，清理你创建的不必要的测试文件，以提高可读性。
更新docs/pinned_memory_development.md，删去原来的架构，并统一文档结构。
# 整体设计
## dora核心（rust）
### memory_manager
功能函数写在dora/binaries/daemon/src/memory_manager.rs中，主要负责共享页锁内存的注册和释放，以及根据内存标识符转换为接受节点进程的指针，以达到跨进程零拷贝传输数据的效果。
daemon初始化时，初始化memory_pool。
daemon结束时，自动调用free_all_pool函数，查询memory_pool，对每个未释放的内存池，执行free_memory_pool。
### python API
#### 1.node.register_memory_pool(tensor_info, device) -> memory_pool_id
该函数的作用为：注册共享内存池，将tensor通过页锁内存传入其中，以达到跨进程零拷贝cpu<->cuda的作用。
入参1：tensor_info为字典，由get_tensor_info生成。
入参2：device为表示设备的字符串，表示内存池应该注册在哪个设备，可以填cpu、cuda、cuda:0等，与pytorch中的device形式相同。
调用该api后，node根据tensor_info在指定device注册共享内存池。
注册之后，生成字典memory_info，除了包含了共享内存池的信息，还包含了tensor的大小、类型、形状信息。
如果共享内存池的device与原tensor的device不同，则通过注册页锁内存和DMA高速传输，将tensor数据在cpu和cuda之间传输。
注册的共享内存池应在头部嵌入256字节的元数据，以方便node.read_memory_pool时校验。
最后node将memory_pool_id和memory_info作为DaemonRequest发给daemon。
daemon收到DaemonRequest之后，会让memory_manager将memory_pool_id和memory_info写入memory_pool；
返回值：memory_pool_id为用py.array包装的共享内存标识符。
#### 3.node.write_memory_pool(memory_pool_id, tensor_info)
该函数的作用为：根据已经注册好的共享内存池，将tensor数据覆盖其中，以达到跨进程零拷贝cpu<->cuda的作用，相比于register_memory_pool，无需重复注册，直接拷贝数据，实现内存的复用。。
入参1：memory_pool_id为用py.array包装的共享内存标识符。
入参2：tensor_info为字典，由get_tensor_info生成。
调用该api后，node根据memory_pool_id访问共享内存池，如果通过了头部元数据的校验（tensor的大小、类型、形状），则将tensor数据通过DMA和页锁内存覆盖共享内存池。
如果校验失败，node会将memory_pool_id作为DaemonRequest发给daemon。
daemon收到DaemonRequest之后，会让memory_manager会根据memory_pool_id查询memory_pool；
查询之后，daemon向node发送DaemonReply（如果查询成功，DaemonReply为查到的memory_info；如果没查到，DaemonReply为查不到枚举）。
node接收到DaemonReply之后，如果为查不到枚举，则直接输出日志（没有查到页锁内存数据），如果查到，则根据memory_info将tensor数据通过DMA和页锁内存覆盖共享内存池；
#### 2.node.read_memory_pool(memory_pool_id) -> tensor_info
该函数的作用为：查询已经注册好的共享内存池，获取tensor_info，用于接收节点零拷贝读取tensor。当write_memory_pool之后，生成的tensor的数据内容会同步修改，不需要显示的重复生成tensor，提高传输效率。
入参:memory_pool_id为用py.array包装的共享内存标识符。
调用该api后，node根据memory_pool_id访问共享内存池，如果通过了头部元数据的校验（tensor的大小、类型、形状），则根据元数据获取tensor_info。
如果校验失败，node会将memory_pool_id作为DaemonRequest发给daemon。
daemon收到DaemonRequest之后，会让memory_manager会根据memory_pool_id查询memory_pool；
查询之后，daemon向node发送DaemonReply（如果查询成功，DaemonReply为查到的tensor_info；如果没查到，DaemonReply为查不到枚举）。
node接收到DaemonReply之后，如果为查不到枚举，则直接输出日志（没有查到页锁内存数据），如果查到，则返回tensor_info；
返回值:tensor_info为字典，用于tensor_from_info零拷贝生成tensor。
#### 3.node.free_memory_pool(memory_pool_id)
入参：memory_pool_id为用py.array包装的共享内存标识符。
调用该api后，node会将memory_pool_id作为DaemonRequest发给daemon。
daemon收到DaemonRequest之后，会让memory_manager会根据memory_pool_id查询memory_pool；
daemon如果查到，则由memory_manager释放该页锁内存池，memory_pool删除该条记录，向node发送DaemonReply（查不到枚举，或者释放成功枚举）。
node接收到DaemonReply之后，如果为查询失败枚举，则输出日志（没有查到页锁内存数据）；
无返回值。
## 扩展模块（dora.cuda）
该文件只负责tensor的解析和生成，不负责注册、解注册共享页锁内存。除了下面这两个函数之外，不建议加新的函数。
### get_tensor_info(tensor) -> tensor_info
只负责将tensor解析为tensor_info。
tensor_info为字典，包含tensor的指针、大小、形状、类型、设备。
### tensor_from_info(tensor_info) -> tensor
只负责根据tensor_info生成tensor。
对指针的位置进行判断，如果指针在cpu上，则生成cpu上的tensor，如果指针在cuda上，则生成cuda上的tensor。
生成的tensor必须保证零拷贝，即tensor的指针必须与tensor_info中的指针相同。
 