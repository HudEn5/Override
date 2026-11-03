# VEXcode 主构建文件，版本 2019_03_26_01

# 是否显示编译器详细输出
VERBOSE = 0

# 引入工具链配置
include vex/mkenv.mk

# 默认程序名；VEX 扩展或命令行可通过 P 指定名称。
PROJECT := $(if $(strip $(P)),$(P),Bartmoss)

# 收集工程的 C++ 和 C 源文件
SRC_C  = $(wildcard src/*.cpp)
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp)
SRC_C += $(wildcard src/*/*.c)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# 收集源文件依赖的头文件
SRC_H  = $(wildcard include/*.h)

# 额外的构建依赖
SRC_A  = makefile

# 工程头文件目录
INC_F  = include

# 默认构建目标
all: $(BUILD)/$(PROJECT).bin

# 引入构建规则
include vex/mkrules.mk
