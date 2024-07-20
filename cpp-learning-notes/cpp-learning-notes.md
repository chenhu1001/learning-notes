# cpp-learning-notes
cpp-learning-notes

# TCP拥塞控制算法、TCP和UDP区别、进程和线程区别
当然可以！下面是详细解释：

### TCP 拥塞控制算法

TCP（传输控制协议）是一种面向连接、可靠的传输层协议，旨在提供可靠的数据传输。为了确保网络稳定和高效，TCP使用了一些拥塞控制算法来管理网络流量。这些算法主要包括：

1. **慢启动（Slow Start）**：在连接建立初期，TCP发送方会缓慢增加其发送速率，以避免网络拥塞。初始的拥塞窗口（Congestion Window, cwnd）通常设为一个最大报文段（MSS）。每当接收到一个ACK时，拥塞窗口就加倍，从而指数级增长，直到达到一个阈值（ssthresh）。

2. **拥塞避免（Congestion Avoidance）**：当拥塞窗口达到ssthresh时，TCP进入拥塞避免阶段。在这个阶段，拥塞窗口线性增长，每收到一个ACK，拥塞窗口增加一个MSS/cwnd。

3. **快速重传（Fast Retransmit）**：如果TCP发送方在没有超时的情况下连续收到三个重复的ACK，表示可能有数据包丢失，此时会立即重传丢失的数据包，而不是等待重传超时。

4. **快速恢复（Fast Recovery）**：在快速重传之后，TCP进入快速恢复阶段，将ssthresh设置为当前cwnd的一半，并将cwnd设为ssthresh加上三个MSS，然后进行拥塞避免阶段的操作。

### TCP和UDP区别

TCP和UDP（用户数据报协议）都是传输层协议，但它们有显著的区别：

1. **连接性**：
   - **TCP**：面向连接的协议，在传输数据前需要建立连接（三次握手），在数据传输完毕后需断开连接（四次挥手）。
   - **UDP**：无连接的协议，传输数据前不需要建立连接，适合对实时性要求高的数据传输，如视频流和在线游戏。

2. **可靠性**：
   - **TCP**：提供可靠的数据传输，保证数据包按顺序到达并且没有丢失或重复，通过序号、确认应答（ACK）和重传机制实现。
   - **UDP**：不保证数据包的顺序，不提供重传机制，因此可能会出现数据包丢失或重复。

3. **流量控制和拥塞控制**：
   - **TCP**：有流量控制和拥塞控制机制，如慢启动、拥塞避免等。
   - **UDP**：没有流量控制和拥塞控制机制，更适合实时应用。

4. **速度**：
   - **TCP**：由于其可靠性和控制机制，传输速度较慢。
   - **UDP**：没有这些控制机制，传输速度较快。

### 进程和线程区别

1. **定义**：
   - **进程**：操作系统中资源分配的基本单位，每个进程都有独立的内存空间，包含代码、数据、文件句柄等。
   - **线程**：操作系统中CPU调度的基本单位，是进程中的一个执行路径。一个进程可以包含多个线程，这些线程共享进程的资源。

2. **资源共享**：
   - **进程**：不同进程间不共享内存空间，资源隔离，通信相对复杂，需要使用进程间通信（IPC）机制，如消息队列、共享内存、信号等。
   - **线程**：同一进程内的线程共享进程的所有资源，如内存、文件句柄，因此线程间通信相对简单，可以直接读写相同的变量。

3. **创建和销毁开销**：
   - **进程**：创建和销毁进程的开销较大，因为需要分配和回收独立的内存空间。
   - **线程**：创建和销毁线程的开销较小，因为线程间共享内存空间。

4. **独立性**：
   - **进程**：独立性较强，一个进程的崩溃不会影响其他进程。
   - **线程**：独立性较弱，同一进程内一个线程的崩溃可能会导致整个进程崩溃。

# 仿函数
在C++中，结构体和类之间的唯一差别是默认访问权限（结构体默认是public，类默认是private）。所以，你可以把结构体（`struct`）当作类（`class`）来使用。

在这个例子中，`struct Square` 是一个自定义函数对象，因为它定义了`operator()`成员函数。这个`operator()`的定义使得`Square`对象可以像函数一样被调用。这种特性使得它成为一个函数对象（或称仿函数）。

### 函数对象（仿函数）简介

函数对象（仿函数，Functors）是在C++中可以像函数一样被调用的对象。它们是通过在类或结构体中重载`operator()`来实现的。

### 详细解释

```cpp
// 自定义函数对象，计算平方
struct Square {
    // operator() 是一个成员函数，可以让 Square 对象像函数一样被调用
    int operator()(int x) const {
        return x * x;
    }
};
```

#### 解析：

- **struct Square**:
    - 定义了一个结构体`Square`。虽然使用的是`struct`，但它可以包含成员函数，与`class`几乎没有区别。

- **operator()(int x) const**:
    - 这是一个成员函数，它重载了函数调用运算符`()`。这意味着你可以创建`Square`类型的对象，并使用它们像函数一样进行调用。
    - **参数**：`int x`表示被调用时传入的参数。
    - **const**：该成员函数是`const`的，这意味着它不会修改对象的状态。

### 示例：使用自定义函数对象

```cpp
int main() {
    Square square; // 创建一个 Square 对象
    int result = square(5); // 调用 Square 对象，就像调用函数一样
    std::cout << "Square of 5 is: " << result << std::endl; // 输出：Square of 5 is: 25
    return 0;
}
```

#### 详细步骤：

1. **创建对象**：
    - `Square square;`：创建一个`Square`类型的对象`square`。

2. **调用对象**：
    - `int result = square(5);`：调用`square`对象，并传入参数`5`。这会调用`Square`结构体中的`operator()`成员函数，计算`5`的平方，返回结果`25`。

3. **输出结果**：
    - `std::cout << "Square of 5 is: " << result << std::endl;`：输出计算结果。

在原示例代码中，`Square`函数对象被传递给`std::transform`算法，作为对每个元素进行变换操作的函数。因为`Square`定义了`operator()`，所以可以像函数一样被调用，从而实现对每个元素进行平方计算的功能。

# STL
当然，可以举一个综合运用STL六大模块的例子。以下示例展示了如何使用STL的容器、迭代器、算法、函数对象、适配器和配接器来完成特定任务：

### 示例：使用STL计算整数序列的平方和并按降序排序

```cpp
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>
#include <iterator>
#include <numeric>

// 自定义配接器（这里我们使用STL默认的std::allocator）
template <typename T>
using Allocator = std::allocator<T>;

// 自定义函数对象，计算平方
struct Square {
    int operator()(int x) const {
        return x * x;
    }
};

int main() {
    // 1. 容器：使用vector存储整数
    std::vector<int, Allocator<int>> numbers = {1, 2, 3, 4, 5};

    // 2. 迭代器：使用vector的迭代器遍历元素
    std::cout << "Original numbers: ";
    for (auto it = numbers.begin(); it != numbers.end(); ++it) {
        std::cout << *it << " ";
    }
    std::cout << std::endl;

    // 3. 算法和函数对象：使用transform算法和自定义函数对象计算平方
    std::vector<int, Allocator<int>> squares(numbers.size());
    std::transform(numbers.begin(), numbers.end(), squares.begin(), Square());

    // 输出平方后的结果
    std::cout << "Squared numbers: ";
    std::copy(squares.begin(), squares.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;

    // 4. 算法：使用accumulate算法计算平方和
    int sum_of_squares = std::accumulate(squares.begin(), squares.end(), 0);
    std::cout << "Sum of squares: " << sum_of_squares << std::endl;

    // 5. 适配器：使用greater函数对象适配器对平方数进行降序排序
    std::sort(squares.begin(), squares.end(), std::greater<int>());

    // 输出排序后的结果
    std::cout << "Sorted squares in descending order: ";
    std::copy(squares.begin(), squares.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;

    return 0;
}
```
那我们一开始就提到，我们学的stack是一种容器适配器，所以它也是一样的，是用来进行转换的，对已有的容器进行转换。
简单的理解容器适配器，其就是将不适用的序列式容器（包括 vector、deque 和 list）变得适用。即通过封装某个序列式容器，并重新组合该容器中包含的成员函数，使其满足某些特定场景的需要。

### 代码解析

1. **容器**：
    - 使用`std::vector`存储整数序列。`std::vector`是一个动态数组，可以方便地进行元素的插入和删除操作。

2. **迭代器**：
    - 使用`std::vector`的迭代器遍历并输出原始整数序列。

3. **算法**和**函数对象**：
    - 使用`std::transform`算法将整数序列的每个元素平方。这里我们定义了一个自定义的函数对象`Square`，其重载了`operator()`用于计算平方值。

4. **算法**：
    - 使用`std::accumulate`算法计算平方后的整数序列的和。

5. **适配器**：
    - 使用`std::greater`适配器对平方数进行降序排序。`std::greater`是一个预定义的函数对象，用于比较两个值的大小。

6. **配接器**：
    - 使用`std::allocator`作为默认的内存分配器。虽然在这个例子中没有显示自定义配接器，但通过模板参数我们可以很容易地替换为其他自定义配接器。

通过这个示例，我们展示了如何综合运用STL的六大模块来完成一个具体的任务。

# union联合
联合（union）是一种节省空间的特殊的类，一个 union 可以有多个数据成员，但是在任意时刻只有一个数据成员可以有值。当某个成员被赋值后其他成员变为未定义状态。联合有如下特点：
• 默认访问控制符为 public
• 可以含有构造函数、析构函数
• 不能含有引用类型的成员
• 不能继承自其他类，不能作为基类
• 不能含有虚函数
• 匿名 union 在定义所在作用域可直接访问 union 成员
• 匿名 union 不能包含 protected 成员或 private 成员
• 全局匿名联合必须是静态（static）的

```
#include<iostream>

union UnionTest {
    UnionTest() : i(10) {};
    int i;
    double d;
};

static union {
    int i;
    double d;
};

int main() {
    UnionTest u;
    union {
        int i;
        double d;
    };
    std::cout << u.i << std::endl; // 输出 UnionTest 联合的 10
    ::i = 20;
    std::cout << ::i << std::endl; // 输出全局静态匿名联合的 20
    i = 30;
    std::cout << i << std::endl; // 输出局部匿名联合的 30
    return 0;
}
```

# 结构体位域
这段代码定义了一个名为 `RemoteCommand` 的结构体，用来表示遥控器的控制指令。让我解释一下每个成员的含义：

- `uint8_t unuse: 5;`：这是一个占用 5 位的无用字段，用来填充剩余的比特位。

- `uint8_t rcType: 1;`：这是一个 1 位的字段，表示遥控器的类型。0 表示挡位遥控，1 表示比例遥控。

- `uint8_t rcRelease: 1;`：这是一个 1 位的字段，表示是否释放遥控模式。1 表示释放遥控模式。

- `uint8_t rcApply: 1;`：这是一个 1 位的字段，表示是否申请遥控模式。1 表示申请遥控模式。

### 解释：

- 结构体中的每个字段都使用了位域（bit-field）的方式来定义，即指定字段的位数，这样可以更加灵活地利用内存空间。

- 使用 `uint8_t` 类型确保每个字段占用 1 字节（8 位）的空间，这是因为 C/C++ 中的位域的位数通常不能超过字段类型的位数。

### 示例用法：

```cpp
#include <cstdint>
#include <iostream>

typedef struct {
    uint8_t unuse: 5;
    uint8_t rcType: 1;
    uint8_t rcRelease: 1;
    uint8_t rcApply: 1;
} RemoteCommand;

int main() {
    RemoteCommand command;
    command.unuse = 7;       // 设置 unuse 字段的值为 7
    command.rcType = 1;      // 设置 rcType 字段的值为 1
    command.rcRelease = 0;   // 设置 rcRelease 字段的值为 0
    command.rcApply = 1;     // 设置 rcApply 字段的值为 1

    std::cout << "unuse: " << (int)command.unuse << std::endl;
    std::cout << "rcType: " << (int)command.rcType << std::endl;
    std::cout << "rcRelease: " << (int)command.rcRelease << std::endl;
    std::cout << "rcApply: " << (int)command.rcApply << std::endl;

    return 0;
}
```

在这个示例中，我们定义了一个 `RemoteCommand` 结构体，并创建了一个 `command` 变量来存储遥控器指令。通过设置各个字段的值，并使用 `(int)` 强制转换为整数类型以输出，可以看到每个字段的具体值。

这种位域的使用方式可以节省内存，特别是在嵌入式系统或者需要节省数据传输带宽的应用中，是一种常见的优化手段。

# #pragma pack(1)这作用，好处和不好的
`#pragma pack(1)` 是一个编译器指令，用于指定结构体（或类）的成员在内存中的对齐方式。具体作用、好处和不好的地方如下：

### 作用和好处：

1. **控制内存对齐**：
    - 默认情况下，结构体的成员在内存中会按照编译器设定的对齐规则进行对齐，以提高访问速度和内存访问的效率。例如，大多数编译器默认按照结构体成员的数据类型大小进行对齐。
    - 使用 `#pragma pack(1)` 可以取消或者减少对齐，让结构体的成员按照1字节对齐，从而节省内存，特别是在需要精确控制内存布局的情况下，例如在网络通信中发送数据结构。

2. **节省内存**：
    - 取消或减少对齐可以减少结构体在内存中的空间占用，尤其是对于那些包含大量小数据类型的结构体来说，能显著降低内存的使用量。

### 不好的地方：

1. **性能影响**：
    - 非对齐访问可能会导致性能下降，特别是在某些处理器架构上，对非对齐内存访问可能需要额外的处理或者多次访存操作，从而影响程序的执行效率。

2. **可移植性**：
    - 使用 `#pragma pack(1)` 改变了默认的对齐规则，可能会导致代码在不同的编译器或不同的平台上表现不一致，使得代码的可移植性变差。
    - 在多平台开发中，可能需要谨慎使用 `#pragma pack(1)`，或者考虑其他方式来保证数据结构的内存布局一致性。

### 示例：

```cpp
#pragma pack(1)
struct MyStruct {
    char a;
    int b;
};
#pragma pack()

int main() {
    std::cout << "Size of MyStruct: " << sizeof(MyStruct) << std::endl; // 输出结构体的大小
    return 0;
}
```

在这个示例中，结构体 `MyStruct` 的成员按照1字节对齐，因此其大小可能会比默认情况下小，节省了内存空间。然而，应注意到使用 `#pragma pack(1)` 可能会带来的性能损失和可移植性问题。

# GeographicLib::LocalCartesian这是什么库
`GeographicLib::LocalCartesian` 是 GeographicLib 库中的一个类，用于在地理坐标系和局部笛卡尔坐标系之间进行转换和计算。GeographicLib 是一个开源的地理计算库，用于执行各种地理和地球坐标系统的转换、距离计算、方向计算等操作。

具体来说，`LocalCartesian` 类提供了以下功能：

- 将地球上的地理坐标（如经纬度）转换为局部笛卡尔坐标系中的坐标。
- 将局部笛卡尔坐标系中的坐标转换为地理坐标（经纬度）。
- 计算两个点之间的距离。
- 计算点的方向和航向角。
- 处理局部地图和局部区域的地理数据。

### 示例用法：

```cpp
#include <GeographicLib/LocalCartesian.hpp>
#include <iostream>

int main() {
    // 创建一个 LocalCartesian 对象，使用 WGS84 椭球体参数
    GeographicLib::LocalCartesian localCartesian(0.0, 0.0, 0.0, GeographicLib::Geocentric::WGS84());

    // 设定基准点的地理坐标（纬度、经度、高度）
    double lat = 37.7749; // 纬度（San Francisco）
    double lon = -122.4194; // 经度（San Francisco）
    double h = 0.0; // 高度（海拔）

    // 转换地理坐标为局部笛卡尔坐标
    double x, y, z;
    localCartesian.Forward(lat, lon, h, x, y, z);

    // 输出转换后的局部笛卡尔坐标
    std::cout << "Local Cartesian Coordinates (x, y, z): " << x << ", " << y << ", " << z << std::endl;

    return 0;
}
```

在这个示例中，我们首先引入 `GeographicLib/LocalCartesian.hpp` 头文件，创建了一个 `LocalCartesian` 对象，并设定了一个地理坐标点（San Francisco 的经纬度），然后使用 `Forward` 方法将地理坐标转换为局部笛卡尔坐标系中的坐标，并输出转换后的坐标。

GeographicLib 提供了丰富的地理计算功能，适用于需要进行地理位置转换和计算的应用场景，如地图软件、导航系统、无人机控制等。

# Eigen::Vector3d point; 这是什么
`Eigen::Vector3d point;` 是使用 Eigen 库定义的一个向量对象 `Vector3d`，用于表示三维空间中的点或向量。

具体解释如下：

- **Eigen**：是一个开源的C++模板库，用于线性代数计算，特别适用于科学计算和机器学习中的数值运算。
- **Vector3d**：是 Eigen 库中的一种向量类型，表示具有三个双精度浮点数（double）分量的向量。它通常用于表示三维空间中的点坐标或向量。

### 示例用法：

```cpp
#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::Vector3d point(1.0, 2.0, 3.0); // 定义一个三维点 (1.0, 2.0, 3.0)

    // 访问向量的各个分量
    std::cout << "X: " << point.x() << ", Y: " << point.y() << ", Z: " << point.z() << std::endl;

    // 输出整个向量
    std::cout << "Point: " << point.transpose() << std::endl;

    return 0;
}
```

在这个示例中，`Eigen::Vector3d` 类型的 `point` 对象表示一个三维点 `(1.0, 2.0, 3.0)`。通过 `x()`、`y()`、`z()` 方法可以分别访问向量的 x、y、z 分量，通过 `transpose()` 方法将向量转置并输出。

Eigen 的向量和矩阵类型提供了丰富的数学运算功能，包括向量加法、减法、点积、叉积等，适用于各种线性代数和数值计算的应用场景。

# using 和 namespace alias（命名空间别名）有一些区别
`using` 和 `namespace alias`（命名空间别名）有一些区别，尽管它们都可以用来简化代码中的类型或命名空间的使用，但它们的作用和适用场景略有不同。

### `using` 语句

- **功能**：`using` 语句用于引入命名空间中的一个或多个成员到当前作用域，或者用于定义类型别名。
- **示例**：
  ```cpp
  using namespace std; // 引入整个命名空间 std 的所有成员
  using std::cout; // 引入命名空间 std 中的 cout 到当前作用域
  using age_t = int; // 定义类型别名 age_t 为 int
  ```

- **注意事项**：
    - 使用 `using namespace` 可能会引入命名冲突和名称隐藏问题，尤其是在大型项目中。
    - 最好避免在头文件中使用 `using namespace`，以免影响其他文件的命名空间。

### 命名空间别名

- **功能**：命名空间别名用于将一个命名空间定义为另一个简短的别名，使得在代码中可以使用别名来代替长命名空间。
- **示例**：
  ```cpp
  namespace pu = parameter_utils; // 将命名空间 parameter_utils 定义为 pu
  ```

- **优势**：
    - 命名空间别名可以有效地减少命名空间的重复书写，提高代码的可读性和简洁性。
    - 可以避免命名空间重命名时需要修改大量的代码。

### 区别总结

- **引入范围**：`using` 可以引入命名空间的所有成员或单个成员；命名空间别名只能将一个命名空间定义为一个别名。
- **使用场景**：`using` 更适合在局部作用域内引入少量成员或定义类型别名；命名空间别名更适合在整个文件或多个文件中使用相同的命名空间时，为其定义一个简短的别名。

综上所述，虽然它们在简化代码和提高可读性方面都有帮助，但在具体使用时需要根据具体情况选择合适的方式。

# #include <Eigen/Eigen> 这个库的作用和简单使用
`#include <Eigen/Eigen>` 是用于引入 Eigen 库的头文件，Eigen 是一个开源的C++模板库，用于线性代数的运算，包括矩阵和向量运算、矩阵分解、求解线性方程组等。它提供了高性能的数学运算，特别适合于科学计算和机器学习等领域的数值计算需求。

### 主要功能和特点：

1. **矩阵和向量运算**：
    - Eigen 提供了 `Matrix` 和 `Vector` 类型，支持常见的矩阵和向量操作，如加法、减法、乘法、转置等。

2. **线性代数运算**：
    - Eigen 支持多种线性代数运算，包括矩阵的逆、行列式计算、特征值和特征向量分解等。

3. **性能优化**：
    - Eigen 在设计上注重性能优化，使用表达式模板技术（expression templates），在编译时生成高效的代码，避免了临时对象的创建和拷贝，提高了运行效率。

4. **模块化设计**：
    - Eigen 模块化的设计使得用户可以按需引入所需的功能，例如 `Core` 模块包含了基本的矩阵和向量运算，而 `Sparse` 模块则用于处理稀疏矩阵。

### 简单使用示例：

```cpp
#include <iostream>
#include <Eigen/Eigen>

int main() {
    // 创建一个3x3的浮点数矩阵
    Eigen::Matrix3f matrix;
    
    // 初始化矩阵
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;
    
    // 输出矩阵
    std::cout << "Matrix:" << std::endl;
    std::cout << matrix << std::endl;
    
    // 计算矩阵的特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(matrix);
    if (solver.info() == Eigen::Success) {
        std::cout << "Eigenvalues:\n" << solver.eigenvalues() << std::endl;
        std::cout << "Eigenvectors:\n" << solver.eigenvectors() << std::endl;
    } else {
        std::cerr << "Eigen solver failed!" << std::endl;
        return 1;
    }
    
    return 0;
}
```

在上述示例中，我们首先包含了 `<Eigen/Eigen>` 头文件，然后创建了一个3x3的浮点数矩阵 `Eigen::Matrix3f`，并初始化它。接着，我们使用 `Eigen::SelfAdjointEigenSolver` 类计算了矩阵的特征值和特征向量。最后，将结果输出到标准输出流中。

这只是 Eigen 库的一个简单示例，它还提供了更多丰富的功能和高级的数学运算，可以根据具体的需求进行更复杂的使用。

# std::mutex mtx的简单使用
`std::mutex` 是C++标准库中用于实现互斥锁的类，用于保护共享资源，避免多个线程同时访问导致的竞态条件。以下是 `std::mutex` 的简单使用示例：

```cpp
#include <iostream>
#include <thread>
#include <mutex>

std::mutex mtx; // 全局互斥量

void print_thread_id(int id) {
    mtx.lock(); // 线程尝试获取互斥量的锁
    std::cout << "Thread ID: " << id << std::endl;
    mtx.unlock(); // 释放互斥量的锁
}

int main() {
    std::thread t1(print_thread_id, 1);
    std::thread t2(print_thread_id, 2);

    t1.join();
    t2.join();

    return 0;
}
```

在上述示例中，`std::mutex mtx` 是一个全局的互斥量对象。在 `print_thread_id` 函数中，每个线程首先调用 `mtx.lock()` 来获取互斥量的锁，以确保在输出线程ID时不会被其他线程打断。完成输出后，通过 `mtx.unlock()` 来释放互斥量的锁，使得其他线程可以继续获取锁并执行其临界区代码。

注意事项：
- 使用 `std::lock_guard` 或 `std::unique_lock` 可以更安全和方便地管理互斥量的锁定和解锁，以避免忘记释放锁或异常情况下未能正确释放锁的问题。
- 确保互斥量的锁定和解锁操作配对使用，避免死锁和资源泄露的风险。
- 在实际的多线程程序中，应当根据具体需求和逻辑合理地设计和使用互斥量，以确保线程安全和程序正确性。

# 虚函数、虚基类、纯虚函数的区别
在C++中，虚函数（virtual function）、虚基类（virtual base class）、以及纯虚函数（pure virtual function）是面向对象编程中重要的概念，它们的区别如下：

1. **虚函数（virtual function）**：
    - 虚函数是用 `virtual` 关键字声明的类成员函数。
    - 虚函数允许在派生类中重新定义（override），即子类可以根据需要重新实现虚函数。
    - 当通过基类指针或引用调用虚函数时，将根据实际对象的类型来调用适当的函数。这种行为称为动态绑定或后期绑定（dynamic binding），允许在运行时确定调用的函数版本。

2. **虚基类（virtual base class）**：
    - 虚基类用于解决多继承时的菱形继承问题（diamond inheritance problem）。
    - 当一个类被声明为虚基类时，无论派生类如何多重继承，虚基类在派生类中只有一份实例。
    - 虚基类在继承链中的位置由虚基类列表指定，例如 `class Derived : virtual public Base`。

3. **纯虚函数（pure virtual function）**：
    - 纯虚函数是通过在声明中使用 `= 0` 来定义的虚函数。
    - 包含纯虚函数的类称为抽象类，不能直接实例化对象。
    - 派生类必须实现（覆盖）抽象类中的所有纯虚函数才能实例化，否则它们也将成为抽象类。

总结起来，虚函数允许运行时多态，虚基类解决了多继承的问题，而纯虚函数则定义了接口，强制派生类实现特定的行为。这些概念在面向对象编程中非常重要，特别是在实现复杂的类层次结构和接口规范时。

# #include <cstdint> 这个的作用
`#include <cstdint>` 是C++标准库中的一个头文件，它提供了固定大小的整数类型（如`int8_t`, `uint16_t`等）的定义。在C++11标准之前，C++语言中没有直接定义固定大小的整数类型，因此 `<cstdint>` 头文件的引入填补了这一空白。这些固定大小的整数类型确保了在不同平台上的一致性，使得编写跨平台的代码更加可靠和方便。

例如，你可以使用 `<cstdint>` 中定义的类型来明确指定变量的大小和符号性，而不依赖于具体的平台或编译器的实现。

# 解决g++没有安装的问题
```
yum install gcc-c++
```

# C++类型转换
在C++中，除了 `static_cast<int>(a)` 之外，还有其他几种类型转换方式。类型转换可以分为隐式转换和显式转换（也称为强制转换）。显式转换有四种主要方式，每种方式有不同的用途和适用场景。以下是这些转换方式的介绍和示例：

### 1. C风格强制转换（C-Style Cast）

这是C语言的传统转换方式，形式为 `(type)expression`。它可以进行几乎任何类型的转换，包括隐式和显式转换。

```cpp
int a = 5;
double b = (double)a;
```

### 2. `static_cast`

`static_cast` 用于在相关类型之间进行转换，例如基本数据类型之间的转换、枚举类型与整数之间的转换、指针类型之间的转换等。它在编译时进行类型检查，但不适用于所有类型的转换（如不同继承关系的类型）。

```cpp
int a = 5;
double b = static_cast<double>(a);
```

### 3. `dynamic_cast`

`dynamic_cast` 用于将基类指针或引用转换为派生类指针或引用。它在运行时进行类型检查，确保转换是安全的，仅适用于包含虚函数的多态类型。如果转换失败，指针类型返回 `nullptr`，引用类型抛出 `bad_cast` 异常。

```cpp
class Base {
    virtual void func() {}
};
class Derived : public Base {};
Base* base = new Derived();
Derived* derived = dynamic_cast<Derived*>(base);
if (derived) {
    // 转换成功，使用 derived
}
```

### 4. `const_cast`

`const_cast` 用于在相同类型的 `const` 和非 `const` 之间转换。它是唯一可以用于移除 `const` 或添加 `const` 的转换方式。

```cpp
const int a = 5;
int* b = const_cast<int*>(&a);
*b = 10; // 修改了 const 对象，可能导致未定义行为
```

### 5. `reinterpret_cast`

`reinterpret_cast` 用于在几乎任何类型之间进行低级别的转换，通常是指针或整数类型之间的转换。它没有类型安全性检查，结果可能与平台相关，通常用于系统编程和底层代码。

```cpp
int a = 5;
void* b = reinterpret_cast<void*>(&a);
int* c = reinterpret_cast<int*>(b);
```

### 示例代码

以下示例展示了上述所有转换方式的使用：

```cpp
#include <iostream>
#include <stdexcept>

class Base {
public:
    virtual void func() {}
};

class Derived : public Base {};

int main() {
    int a = 5;

    // C风格强制转换
    double b = (double)a;
    std::cout << "C-style cast: " << b << "\n";

    // static_cast
    double c = static_cast<double>(a);
    std::cout << "static_cast: " << c << "\n";

    // dynamic_cast
    Base* base = new Derived();
    Derived* derived = dynamic_cast<Derived*>(base);
    if (derived) {
        std::cout << "dynamic_cast: Successful\n";
    } else {
        std::cout << "dynamic_cast: Failed\n";
    }

    // const_cast
    const int d = 5;
    int* e = const_cast<int*>(&d);
    *e = 10; // 修改了 const 对象，可能导致未定义行为
    std::cout << "const_cast: " << *e << "\n";

    // reinterpret_cast
    int f = 5;
    void* g = reinterpret_cast<void*>(&f);
    int* h = reinterpret_cast<int*>(g);
    std::cout << "reinterpret_cast: " << *h << "\n";

    return 0;
}
```

### 注意事项

- 使用 `reinterpret_cast` 和 `const_cast` 时要非常小心，因为它们可能导致未定义行为或类型安全问题。
- `dynamic_cast` 适用于有虚函数的类，通常用于安全的向下转换。
- `static_cast` 是大多数类型转换的首选，因为它在编译时进行类型检查，比C风格强制转换更安全。
