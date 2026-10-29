# Natsel 性能记录

性能测量只比较 Natsel 自身的不同版本或配置。基准程序按需运行，不作为单元测试中的耗时门槛。

## 复测方法

使用同一机器、Release 构建、相同场景、步长、子步数和 worker 数。交替执行修改前后的可执行文件，
每个场景至少三次，记录全部耗时并比较最小值。世界创建、预热与最终状态哈希在计时区间之外。

```sh
swift build -c release --product Benchmarks
# 修改代码前保存基准程序，再构建修改后的版本。
cp .build/release/Benchmarks /tmp/Natsel-before
# 完成修改后重新构建。
swift build -c release --product Benchmarks
python3 Tools/Benchmarks/compare.py /tmp/Natsel-before .build/release/Benchmarks \
  --scenes many_pyramids joint_grid --repeats 3 --output /tmp/natsel-comparison
```

脚本固定单个 worker，清除继承的诊断覆盖，并保存每次输出、两个程序的 SHA-256、状态哈希和
`comparison.json`。同一程序每轮状态哈希必须一致；两个版本之间的哈希变化单独记录，
有意改变求解行为时须结合物理正确性与回放测试评估。

默认只运行 `many_pyramids` 和 `joint_grid` 两个代表场景。使用基准程序的 `--list` 查看其他场景，
通过比较脚本的 `--scenes` 或基准程序的 `--benchmark=场景名` 按需选择；`--all` 执行全部场景。`NATSEL_WIDE`、`NATSEL_WORKERS`、`NATSEL_PHASE` 等选项可用于单独诊断；
正式版本复测由脚本固定配置。

## 历史优化记录

以下为同一轮 Natsel 优化前后的历史测量，不代表当前版本的性能。
机器：Apple M1，8 核，macOS 27.0.1；`swift build -c release`；单个 worker；
`dt = 1/60`，4 子步，每场景三次取最小值。

| 场景 | 优化前 ms | 优化后 ms | 提速 |
|---|---:|---:|---:|
| large_pyramid | 6185 | 4286 | 1.44x |
| many_pyramids | 6312 | 4121 | 1.53x |
| joint_grid | 7534 | 4323 | 1.74x |
| convex_pile | 56280 | 22622 | 2.49x |
| spinner | 75562 | 12453 | 6.07x |
| washer | 83038 | 79219 | 1.05x |
| trees25 | 1464 | 1420 | 1.03x |

这轮优化主要减少了热循环中的值拷贝、数组写时复制和临时分配，并复用求解器缓冲区。

## 正确性约束

- 同一版本与配置重复运行，状态结果一致。
- 不同 worker 数、标量和宽 SIMD 路径保持确定性。
- 碰撞、关节、守恒、连续碰撞与保存恢复分别通过行为测试验证。
- 性能变化与物理行为变化分别记录，不使用耗时替代正确性检查。
