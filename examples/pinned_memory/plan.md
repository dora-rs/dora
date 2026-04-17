# 当前进度
1、基于dora开发通过页锁内存完成cpu->cuda跨进程传输tensor的功能
2、目前基本跑通，传输速度1200MB/S。

# 待解决问题
## 整理dora.cuda文件
import文件和libcudart.so函数声明仍然重复，应该去掉重复，统一写在文件的最上面；注释太多了，中间的注释应该删去，仅在函数签名后注释函数的作用。
## 未达到零拷贝的效果，传输速度过慢。
传输速度仅1200MB/S，可能因为没有使用 DMA 等高速读取机制，也有可能在torch_to_pinned_buffer中使用numpy中转，导致多次拷贝。
IPC handle的传输速度高达30000MB/S,你应该学习IPC Handle的跨进程零拷贝思路，并达到跟它差不多的传输效果。
# 测试
## 测试方法
在dora/examples/pinned_memory文件夹下进行测试。
改完dora底层之后，你可以用cargo build --release --package dora-cli进行编译。
我已经将dora/target/release写入环境变量了，因此你可以直接在pinned_memory文件夹下用dora run cpu2cuda.yml进行测试。
