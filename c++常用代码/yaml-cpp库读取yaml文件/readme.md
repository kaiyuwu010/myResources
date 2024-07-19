#运行指令
g++ -o yaml yaml.cpp -I ./dependency/include -L ./dependency/lib -lyaml-cpp

-I 表示包含头文件的目录路径（不是文件路径）
-L 表示包含的库文件的目录路径
-l 表示包含的库的名称，例如库文件libyaml-cpp.a就应该写为-lyaml-cpp