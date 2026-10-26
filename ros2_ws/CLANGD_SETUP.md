# clangd 配置手册（ROS2 Humble + MoveIt2 工作区 / Dev Container）

本文记录本工作区 clangd 的完整配置方案、踩坑原因与日常维护流程。

适用环境：

- Dev Container 内的 VSCode Remote
- ROS 2 Humble（`/opt/ros/humble`）
- colcon 工作区，根目录 `ros2_ws/`
- clangd 14（Ubuntu `clangd-14`）

---

## 1. 结论速览

clangd 能否正常工作，**只取决于一件事**：它能否找到一份正确的 `compile_commands.json`（编译数据库）。

| 配置文件 | 作用 | 是否必需 |
| --- | --- | --- |
| `ros2_ws/compile_commands.json` | 编译数据库，提供 include 路径、宏定义、编译标准 | **必需** |
| `ros2_ws/.clangd` | clangd 原生配置，为数据库外的文件提供兜底 | 可选，建议保留 |
| `ros2_ws/.vscode/settings.json` | VSCode 扩展配置（路径、参数、冲突规避） | **必需** |
| `ros2_ws/scripts/merge_compile_commands.py` | 合并各包数据库为根目录单一文件 | **必需** |
| `ros2_ws/.vscode/c_cpp_properties.json` | Microsoft C/C++ 扩展用，clangd **不读** | 可留可删 |

---

## 2. 本次排障的三个根因

### 根因一：工作区根本没有编译数据库

`colcon build` 默认**不**生成 `compile_commands.json`。全盘搜索确认整个容器内不存在该文件。

CMake 只有在显式开启时才导出：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 根因二：数据库分散在每个包下，clangd 找不到

开启导出后，CMake 为**每个包**单独生成一份：

```
build/my_robot_commander_cpp/compile_commands.json
build/my_robot_interfaces/compile_commands.json
```

clangd 只在工作区根目录（或 `--compile-commands-dir` 指定的目录）查找一个 `compile_commands.json`。
因此需要把各包的文件合并成根目录的单一文件 —— 这就是 `scripts/merge_compile_commands.py` 的作用。

### 根因三：编译命令里没有 `-std=`，clangd 回退到 C++14

合并后 clangd 仍报大量错误：

```
no template named 'variant' in namespace 'std'
no template named 'is_convertible_v' in namespace 'std'
```

排查发现 CMake 生成的编译命令中**没有 `-std=` 标志**（`CXX_STANDARD` 目标属性未落到命令行），
clangd 于是使用默认标准 C++14，导致 C++17 特性全部无法解析，并级联出大量误报。

修复（双保险）：

1. 在 `src/my_robot_commander_cpp/CMakeLists.txt` 中显式声明标准，使 `-std=gnu++17` 进入编译命令：
   ```cmake
   if(NOT CMAKE_CXX_STANDARD)
     set(CMAKE_CXX_STANDARD 17)
   endif()
   set(CMAKE_CXX_STANDARD_REQUIRED ON)
   set(CMAKE_CXX_EXTENSIONS ON)
   ```
2. 合并脚本额外强制注入 `-std=c++17` 与 `-Wall -Wextra -Wpedantic`。

修复后：`All checks completed, 0 errors`。

---

## 3. 完整配置步骤（可复现）

### 步骤 1：确认 clangd 可用

```bash
which clangd
clangd --version
```

若容器内只有版本化名字（Ubuntu 默认），建立统一软链接：

```bash
ln -sf /usr/bin/clangd-14 /usr/bin/clangd
```

### 步骤 2：重建工作区并导出编译数据库

```bash
cd /workspace/ros2-moveit2-arm-control-tutorial/ros2_ws
. /opt/ros/humble/setup.sh          # 注意：sh 用 `.`，bash 用 `source`
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

验证：

```bash
ls -l build/*/compile_commands.json
```

### 步骤 3：合并为根目录单一数据库

```bash
python3 scripts/merge_compile_commands.py
```

输出示例：

```
merged   2 entries from build/my_robot_commander_cpp/compile_commands.json
merged  11 entries from build/my_robot_interfaces/compile_commands.json
wrote 13 entries to compile_commands.json
```

### 步骤 4：配置 VSCode

`ros2_ws/.vscode/settings.json` 关键项：

```json
{
  "clangd.path": "/usr/bin/clangd",
  "clangd.arguments": [
    "--compile-commands-dir=/workspace/ros2-moveit2-arm-control-tutorial/ros2_ws",
    "--background-index",
    "--completion-style=detailed",
    "--header-insertion=never"
  ],
  "C_Cpp.intelliSenseEngine": "disabled"
}
```

要点：

- `clangd.path` —— 显式指定可执行文件，避免扩展找不到 `clangd`。
- `--compile-commands-dir` —— 指向工作区根目录，clangd 在此查找数据库。
- `--background-index` —— 后台索引，缩短首次打开大文件时的卡顿。
- `--header-insertion=never` —— 不自动补 `#include`，避免引入意外依赖。
- `C_Cpp.intelliSenseEngine: disabled` —— 关闭 Microsoft C/C++ 扩展的 IntelliSense，
  避免两个语言服务器同时提供补全/跳转造成冲突与重复红波浪线。
  （若完全卸载了 Microsoft C/C++ 扩展，此项可省略。）

### 步骤 5：加载 clangd

VSCode 命令面板执行：

- `clangd: Restart language server` —— 修改配置后
- `Developer: Reload Window` —— 修改 `.vscode/settings.json` 或 `.clangd` 后（**必须**，扩展不会热重载设置）

首次索引可能需要数十秒，状态栏会显示 `clangd: indexed`。

---

## 4. 日常维护流程

### 场景 A：修改了 CMakeLists.txt（新增依赖、改编译选项）

```bash
. /opt/ros/humble/setup.sh
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
python3 scripts/merge_compile_commands.py
```

然后 VSCode 执行 `clangd: Restart language server`。

### 场景 B：新增了源文件

新文件不在旧数据库中，需要重新构建 + 合并（同上）。
在合并完成前，新文件的头文件解析会暂时失准。

### 场景 C：切换了容器 / 重新拉起 dev container

容器内路径若发生变化，数据库里的绝对路径会失效，必须在**容器内**重新执行步骤 2、3。

### 一条龙命令

```bash
. /opt/ros/humble/setup.sh && \
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
python3 scripts/merge_compile_commands.py
```

---

## 5. 常见故障排查

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 所有头文件都报 "file not found" | 找不到编译数据库 | 确认 `compile_commands.json` 在工作区根目录；确认 `--compile-commands-dir` 路径正确 |
| `no template named 'variant' in namespace 'std'` | clangd 回退到 C++14 | 编译命令缺 `-std=`；见根因三 |
| 头文件能解析但**没有任何诊断** | 只报了 error，warning 被静默 | 编译命令缺 `-Wall -Wextra`；合并脚本已注入 |
| 改了配置不生效 | 扩展不热重载设置 | `Developer: Reload Window` |
| 补全/跳转异常或出现两套红波浪线 | Microsoft C/C++ 扩展与 clangd 冲突 | `C_Cpp.intelliSenseEngine: disabled`，或卸载该扩展 |
| 索引特别慢 | 启用了 clang-tidy | 移除 `--clang-tidy` 参数（ROS2 头文件上噪声极大） |

### 排查命令

```bash
# 看 clangd 是否加载了数据库、诊断了什么
clangd --check=src/my_robot_commander_cpp/src/commander_template.cpp \
  --compile-commands-dir=$PWD 2>&1 | grep -E "Loaded compilation|All checks"

# 看数据库里某条命令实际带哪些标志
python3 -c "
import json
d=json.load(open('compile_commands.json'))
e=[x for x in d if 'commander_template' in x['file']][0]
print([t for t in e['arguments'] if t.startswith('-std=') or t.startswith('-W')])
"
```

> **注意**：`clangd --check` **只输出 error**，不会输出 warning。
> 验证 warning 是否生效，需要用 LSP 协议走 `textDocument/publishDiagnostics`，
> 或直接在编辑器中观察。（本次排障中曾用 `--check` 误判为"无诊断"。）

编辑器内查看日志：命令面板 `clangd: Open logs`。

---

## 6. 编译数据库生成原理

CMake 开启 `CMAKE_EXPORT_COMPILE_COMMANDS` 后，为每个 target 生成一份 JSON：

```json
{
  "file": "/abs/path/to/commander_template.cpp",
  "arguments": ["/usr/bin/c++", "-std=gnu++17", "-Wall", "-I/opt/ros/humble/include/...", "..."]
}
```

clangd 依据此文件还原每个源文件的真实编译上下文（头文件搜索路径、宏、`-std=`），
从而正确解析 ROS2 / MoveIt2 的头文件并给出准确诊断。

**关键**：该文件包含机器相关的绝对路径，因此不应提交到版本库。
仓库根目录 `.gitignore` 已包含 `compile_commands.json` 规则。

---

## 7. 相关文件清单

### 需要保留

| 文件 | 说明 |
| --- | --- |
| `ros2_ws/.vscode/settings.json` | VSCode 扩展配置 |
| `ros2_ws/.clangd` | clangd 原生配置 |
| `ros2_ws/scripts/merge_compile_commands.py` | 数据库合并脚本 |
| `ros2_ws/src/my_robot_commander_cpp/CMakeLists.txt` | C++17 标准声明（已修改） |

### 构建期生成，不提交

| 文件 | 说明 |
| --- | --- |
| `ros2_ws/compile_commands.json` | 合并后的编译数据库（已被 gitignore） |
| `ros2_ws/build/*/compile_commands.json` | 各包原始数据库（已在 gitignore） |
| `ros2_ws/.cache/clangd/` | clangd 索引缓存（建议加入 gitignore） |

### 可选删除

| 文件 | 说明 |
| --- | --- |
| `ros2_ws/.vscode/c_cpp_properties.json` | 仅 Microsoft C/C++ 扩展使用；clangd 忽略它。若保留该扩展则保留 |

---

## 8. 验证结果

本工作区实际验证记录：

```
colcon build → 5 packages finished（my_robot_description / my_robot_interfaces /
              my_robot_moveit_config / my_robot_commander_cpp / my_robot_bringup）

clangd --check commander_template.cpp → Loaded compilation database from
    /workspace/ros2-moveit2-arm-control-tutorial/ros2_ws/compile_commands.json
    All checks completed, 0 errors

LSP 实时诊断（编辑器实际路径）：
    test_moveit.cpp        → 2 diagnostics
        [WARNING] line 22: Implicit conversion from 'double' to 'int' changes value from 2.3 to 2
        [WARNING] line 22: Unused variable 'z'
    commander_template.cpp → 0 diagnostics（无误报）
```

`int z = 2.3;` 被正确诊断为 **WARNING**（隐式窄化转换），说明诊断链路、
警告标志、标准版本三者均已生效。

### `.clangd` 删与不删的实测对比

| 场景 | 保留 `.clangd` | 删除 `.clangd` |
| --- | --- | --- |
| 已在数据库中的文件（`commander_template.cpp`） | `0 errors` | `0 errors`（**无差异**） |
| 未编译的新建源文件 | 能解析 ROS2 头文件 | 满屏 "file not found" |

结论：**对已编译文件，`.clangd` 可安全删除**。它只对尚未进入编译数据库的新文件起兜底作用。
建议保留；若要删除，新建文件后需及时执行第 4 节的一次龙命令让文件进入数据库。

---

## 9. 附：`.clangd` 配置说明

```yaml
CompileFlags:
  CompilationDatabase: .      # 编译数据库所在目录
  Add:
    - -std=gnu++17            # 数据库外文件的兜底标准
    - -Wall
    - -Wextra
    - -Wpedantic

Diagnostics:
  Suppress:
    - "*:unknown-warning-option"
    - "ppcclint.c"             # ROS2 头文件噪声
  UnusedIncludes: None         # 不报告未使用的 include
  MissingIncludes: None        # 不报告缺失的 include

Index:
  Background: true             # 后台索引

InlayHints:
  Enabled: true
```

> `CompilationDatabase: .` 与 VSCode 设置中的 `--compile-commands-dir` 指向同一目录，
> 二者一致时不会冲突。

### 已知配置警告（clangd 14）

`clangd` 启动时会在日志中打印两条 config warning，均**不影响功能**，可忽略：

```
config warning at .clangd:23:2: Unknown Diagnostics key 'MissingIncludes'
config warning at :-1:-1: Invalid Background value 'true'. Valid values are Build, Skip.
```

含义：

- `MissingIncludes` —— 该键在 clangd 14 中不存在（它属于 clang-tidy 的检查项，
  不是 clangd 的 Diagnostics 配置），因此**从未生效**。缺失头文件的提示由 clang 自身的
  `ppc` 诊断提供，不受此键影响。
- `Index.Background` —— 合法值是 `Build` / `Skip`，`true` 会被解析为无效值并回退到默认行为。
  VSCode 扩展的 `--background-index` 参数已单独实现了后台索引，所以此处的取值不影响实际效果。

若想消除警告，可将 `Background: true` 改为 `Background: Build`，并删除 `MissingIncludes` 一行；
但这不是必须操作。
