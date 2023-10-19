### Q: CMakeLists.txt这个文件是干啥的？

这个文件是一个**CMakeLists.txt**文件，它用来配置和生成一个名为**libMultiRobotPlanning**的项目。这个项目是一个用于多机器人路径规划的C++库，它包含了以下几个方面的内容：

- **依赖库**：这个项目需要使用**Boost**和**yaml-cpp**两个库，它们分别用于提供程序选项和解析YAML文件。
- **文档生成**：这个项目使用**Doxygen**来生成API文档，它根据源代码中的注释来生成HTML格式的文档。
- **代码质量**：这个项目使用**clang-tidy**和**clang-format**两个工具来检查和格式化代码，以保证代码的规范性和可读性。
- **测试运行**：这个项目使用**python3 unittest**来运行测试用例，以验证代码的正确性和功能性。
- **编译选项**：这个项目使用C++14标准，并开启了所有的警告选项。它还导出了一个编译数据库，供clang-tidy使用。
- **头文件目录**：这个项目将include目录作为头文件的搜索路径，该目录下包含了所有的库头文件，如a_star.hpp, cbs.hpp等。
- **示例程序**：这个项目提供了一些示例程序，展示了如何使用这个库来进行多机器人路径规划。每个示例程序都有一个对应的可执行文件，如a_star, cbs, ecbs等。每个可执行文件都链接了Boost和yaml-cpp两个库。

: https://github.com/whoenig/libMultiRobotPlanning : https://www.boost.org/ : https://github.com/jbeder/yaml-cpp : http://www.doxygen.nl/ : https://clang.llvm.org/extra/clang-tidy/ : https://clang.llvm.org/docs/ClangFormat.html : https://docs.python.org/3/library/unittest.html