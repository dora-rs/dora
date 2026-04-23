# 任务要求
基于dora开发通过页锁内存和DMA高速读取机制完成cpu->cuda跨进程传输tensor的功能。 
请根据整体设计重新进行开发，并达到零拷贝的传输效果（传输速度约为30000MB/S）
## 每次改完dora的rust底层，你在验证前，需要用cargo build --release --package dora-cli进行编译。
## 测试通过指标：dora run cpu2cuda.yml 成功传输数据。
## cpu_sender.py和cuda_receiver.py为测试通过指标框架，不允许修改。
# 当前情况

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

  当前架构分析                                                                                                                                                                          
                                               
  现有问题                                                                                                                                                                              
                                                                  
  1. 两次数据复制：CPU内存 → 共享内存 → GPU内存                                                                                                                                         
  2. 内存未注册为CUDA页锁内存：is_pinned标志始终为false                                                                                                                                 
  3. 未利用CUDA直接内存访问：始终使用cudaMemcpy复制数据                                                                                                                                 
                                                                                                                                                                                        
  零拷贝原理                                                                                                                                                                            
                                                                                                                                                                                        
  真正的零拷贝需要：                                                                                                                                                                    
  1. 页锁内存注册：使用cudaHostRegister将内存注册为CUDA可访问的页锁内存
  2. 直接设备指针获取：使用cudaHostGetDevicePointer获取GPU可直接访问的指针                                                                                                              
  3. 页对齐内存：内存地址必须对齐到4096字节边界                           
                                                                                                                                                                                        
  修改计划                                                                                                                                                                              
                                                                                                                                                                                        
  第一阶段：核心功能修改                                                                                                                                                                
                                                                                                                                                                                        
  1. 修改 dora/apis/python/node/dora/cuda.py                                                                                                                                            
                                                                  
  函数：pinned_ptr_to_torch(pinned_ptr, metadata) -> torch.Tensor                                                                                                                       
  - 当前问题：仅检查is_pinned标志，总是回退到cudaMemcpy           
  - 修改内容：                                                                                                                                                                          
    - 添加内存页对齐检查（ptr % 4096 == 0）                       
    - 尝试使用cudaHostRegister注册共享内存                                                                                                                                              
    - 成功后使用cudaHostGetDevicePointer获取设备指针                                                                                                                                    
    - 维护注册内存的全局跟踪表                                                                                                                                                          
    - 失败时优雅回退到cudaMemcpy                                                                                                                                                        
                                                                                                                                                                                        
  2. 新增辅助函数                                                                                                                                                                       
                                                                                                                                                                                        
  函数：_register_as_pinned(ptr, size) -> bool                                                                                                                                          
  - 封装cudaHostRegister调用                                                                                                                                                            
  - 处理错误码（特别是CUDA_ERROR_HOST_MEMORY_ALREADY_REGISTERED）                                                                                                                       
  - 返回注册状态                                                  
                                                                                                                                                                                        
  函数：_unregister_pinned(ptr)                                                                                                                                                         
  - 封装cudaHostUnregister调用                                                                                                                                                          
  - 从全局跟踪表中移除                                                                                                                                                                  
                                                                                                                                                                                        
  3. 修改 dora/apis/python/node/src/lib.rs                                                                                                                                              
                                                                                                                                                                                        
  函数：Node::register_pinned_memory_internal()                                                                                                                                         
  - 当前问题：创建的共享内存可能未页对齐                                                                                                                                                
  - 修改内容：                                                                                                                                                                          
    - 确保共享内存分配时使用页对齐（4096字节）                    
    - 在元数据中添加is_aligned标志                                                                                                                                                      
                                                                                                                                                                                        
  函数：Node::read_pinned_memory()                                                                                                                                                      
  - 在返回的元数据中明确设置is_pinned=false（接收端将尝试注册）                                                                                                                         
                                                                                                                                                                                        
  第二阶段：内存对齐优化                                                                                                                                                                
                                                                                                                                                                                        
  4. 修改共享内存分配                                                                                                                                                                   
                                                                                                                                                                                        
  位置：shared_memory_extended库使用                                                                                                                                                    
  - 确保ShmemConf分配页对齐的内存                                 
  - 可能需要使用libc::posix_memalign或std::alloc::aligned_alloc                                                                                                                         
  - 对齐要求：size和起始地址都必须是4096的倍数                    
                                                                                                                                                                                        
  5. 添加对齐检查                                                                                                                                                                       
                                                                                                                                                                                        
  位置：pinned_ptr_to_torch函数开始处                                                                                                                                                   
  - 检查指针是否4096字节对齐                                                                                                                                                            
  - 检查大小是否4096字节对齐                                                                                                                                                            
  - 不对齐时记录警告并回退到cudaMemcpy                            
                                                                                                                                                                                        
  第三阶段：资源管理和清理                                                                                                                                                              
                                                                                                                                                                                        
  6. 实现内存注册跟踪                                                                                                                                                                   
                                                                                                                                                                                        
  - 使用弱引用字典跟踪已注册的内存指针                                                                                                                                                  
  - 在free_pinned_memory中自动调用cudaHostUnregister              
  - 防止重复注册/取消注册                                                                                                                                                               
                                                                                                                                                                                        
  7. 修改释放逻辑                                                                                                                                                                       
                                                                                                                                                                                        
  位置：dora/apis/python/node/src/lib.rs中的free_pinned_memory                                                                                                                          
  - 调用Python端的取消注册函数                                    
  - 确保daemon清理后也清理CUDA注册                                                                                                                                                      
                                                                  
  第四阶段：性能优化                                                                                                                                                                    
                                                                                                                                                                                        
  8. 异步操作支持                                                                                                                                                                       
                                                                                                                                                                                        
  - 使用CUDA流进行异步内存注册和复制                                                                                                                                                    
  - 重叠计算和内存传输                                            
                                                                                                                                                                                        
  9. 批量处理优化                                                                                                                                                                       
                                                                                                                                                                                        
  - 支持连续内存区域的批量注册                                                                                                                                                          
  - 减少CUDA API调用开销                                          
                                                                                                                                                                                        
  关键代码位置                                                                                                                                                                          
                                                                                                                                                                                        
  主要修改文件                                                                                                                                                                          
                                                                  
  1. dora/apis/python/node/dora/cuda.py (核心修改)                                                                                                                                      
    - 第385-412行：pinned_ptr_to_torch函数                        
    - 需要添加：页对齐检查、cudaHostRegister调用、设备指针获取                                                                                                                          
  2. dora/apis/python/node/src/lib.rs (辅助修改)                                                                                                                                        
    - 第508-513行：共享内存分配                                                                                                                                                         
    - 第634-640行：元数据创建                                                                                                                                                           
  3. dora/binaries/daemon/src/lib.rs (可选)                                                                                                                                             
    - 第1631-1643行：元数据解析中的is_pinned处理                                                                                                                                        
                                                                                                                                                                                        
  新增文件/函数                                                                                                                                                                         
                                                                                                                                                                                        
  1. 内存注册管理器（Python类）                                                                                                                                                         
    - 跟踪所有注册的页锁内存                                      
    - 自动清理资源                                                                                                                                                                      
                                                                                                                                                                                        
  技术挑战和解决方案                                                                                                                                                                    
                                                                                                                                                                                        
  挑战1：内存页对齐                                                                                                                                                                     
                                                                  
  - 问题：CUDA要求页锁内存地址和大小都对齐到4096字节                                                                                                                                    
  - 方案：在共享内存分配时确保对齐，添加对齐检查                  
                                                                                                                                                                                        
  挑战2：跨进程协调                                                                                                                                                                     
                                                                                                                                                                                        
  - 问题：发送端和接收端需要协调内存注册状态                                                                                                                                            
  - 方案：接收端负责注册，发送端只负责提供对齐的内存              
                                                                                                                                                                                        
  挑战3：错误恢复                                                                                                                                                                       
                                                                                                                                                                                        
  - 问题：内存注册可能失败（权限、对齐、资源限制）                                                                                                                                      
  - 方案：优雅降级到cudaMemcpy，记录详细错误信息                  
                                                                                                                                                                                        
  挑战4：资源泄漏                                                                                                                                                                       
                                                                                                                                                                                        
  - 问题：注册的内存需要正确取消注册                                                                                                                                                    
  - 方案：使用上下文管理器或析构函数确保清理                      
                                                                                                                                                                                        
  测试验证计划                                                                                                                                                                          
                                                                                                                                                                                        
  性能测试                                                                                                                                                                              
                                                                  
  1. 基准测试：使用dora run cpu2cuda.yml测量传输速率                                                                                                                                    
  2. 目标：达到30000MB/s（当前1518MB/s，提升约20倍）              
                                                                                                                                                                                        
  功能测试                                                                                                                                                                              
                                                                                                                                                                                        
  1. 对齐检查测试：验证不同大小和地址的对齐处理                                                                                                                                         
  2. 注册/取消注册测试：确保无资源泄漏                            
  3. 错误恢复测试：测试注册失败时的回退机制                                                                                                                                             
                                                                                                                                                                                        
  集成测试                                                                                                                                                                              
                                                                                                                                                                                        
  1. 现有测试保持通过：test_1/和test_2/中的测试                                                                                                                                         
  2. 新功能测试：添加页锁内存注册的专项测试                       
                                                                                                                                                                                        
  实施优先级                                                                                                                                                                            
                                                                                                                                                                                        
  1. 高优先级：修改pinned_ptr_to_torch实现内存注册                                                                                                                                      
  2. 中优先级：确保共享内存页对齐                                 
  3. 低优先级：异步优化和批量处理                                                                                                                                                       
                                                                                                                                                                                        
  预期风险                                                                                                                                                                              
                                                                                                                                                                                        
  1. 平台兼容性：不同Linux版本和CUDA版本的差异                                                                                                                                          
  2. 性能波动：不同硬件配置下的表现差异                           
  3. 资源限制：系统页锁内存限制可能影响大内存传输                                                                                                                                       
                                                                                                                                                                                        
  成功标准                                                                                                                                                                              
                                                                                                                                                                                        
  1. 性能标准：传输速率显著提升（目标30000MB/s）                                                                                                                                        
  2. 功能标准：零拷贝路径被正确使用（cudaHostGetDevicePointer成功）
  3. 稳定性标准：无内存泄漏，错误处理健壮                                                                                                                                               
                                                                                                                                                                                        
  这个计划聚焦于最小必要修改，优先实现核心的零拷贝功能，再逐步优化和完善。