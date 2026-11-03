# VEXcode 构建环境配置，版本 2022_06_26_01

# 用于处理含空格路径的辅助宏
sp = $() $()
qs = $(subst ?, ,$1)
sq = $(subst $(sp),?,$1)

# 默认平台和构建目录
PLATFORM  = vexv5
BUILD     = build

# Clang 内置头文件版本
ifneq ("$(origin HEADERS)", "command line")
HEADERS = 8.0.0
endif

# 从应用传入的工程名称
ifeq ("$(origin P)", "command line")
PROJECT  := $(P)
else
PROJECT  := $(call qs,$(notdir $(call sq,${CURDIR})))
endif

# 检查工程名称是否包含空白字符
ifneq (1,$(words $(PROJECT)))
$(error Project name cannot contain whitespace: $(PROJECT))
endif

# 从应用传入的 SDK 路径
# 未通过命令行指定时，使用环境变量
ifeq ("$(origin T)", "command line")
VEX_SDK_PATH = $(T)
endif
# 仍未设置时使用默认路径
VEX_SDK_PATH ?= ${HOME}/sdk

# 应用传入的浮点打印选项（当前版本未使用）
ifeq ("$(origin PRINTF_FLOAT)", "command line")
PRINTF_FLAG = -u_printf_float
endif

# 从应用传入的详细输出选项
ifeq ("$(origin V)", "command line")
BUILD_VERBOSE=$(V)
endif

# 应用未指定时，允许主构建文件设置详细输出选项
ifndef VERBOSE
BUILD_VERBOSE ?= 0
else
BUILD_VERBOSE ?= $(VERBOSE)
endif

# 根据详细输出选项决定是否回显命令
ifeq ($(BUILD_VERBOSE),0)
Q = @
else
Q =
endif

# 编译和链接工具
CC      = clang
CXX     = clang
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size
LINK    = arm-none-eabi-ld
ARCH    = arm-none-eabi-ar
ECHO    = @echo
DEFINES = -DVexV5

# 各操作系统使用的宏
ifeq ($(OS),Windows_NT)
$(info windows build for platform $(PLATFORM))
SHELL = cmd.exe
MKDIR = md "$(@D)" 2> nul || :
RMDIR = rmdir /S /Q
CLEAN = $(RMDIR) $(BUILD) 2> nul || :
else
# 检测当前类 Unix 系统及架构
UNAME := $(shell sh -c 'uname -sm 2>/dev/null || Unknown')
$(info unix build for platform $(PLATFORM) on $(UNAME))
MKDIR = mkdir -p "$(@D)" 2> /dev/null || :
RMDIR = rm -rf
CLEAN = $(RMDIR) $(BUILD) 2> /dev/null || :
endif

# 工具链头文件和库目录
TOOL_INC  = -I"$(VEX_SDK_PATH)/$(PLATFORM)/clang/$(HEADERS)/include" -I"$(VEX_SDK_PATH)/$(PLATFORM)/gcc/include/c++/4.9.3"  -I"$(VEX_SDK_PATH)/$(PLATFORM)/gcc/include/c++/4.9.3/arm-none-eabi/armv7-ar/thumb" -I"$(VEX_SDK_PATH)/$(PLATFORM)/gcc/include"
TOOL_LIB  = -L"$(VEX_SDK_PATH)/$(PLATFORM)/gcc/libs"

# 编译器选项
CFLAGS_CL = -target thumbv7-none-eabi -fshort-enums -Wno-unknown-attributes -U__INT32_TYPE__ -U__UINT32_TYPE__ -D__INT32_TYPE__=long -D__UINT32_TYPE__='unsigned long'
CFLAGS_V7 = -march=armv7-a -mfpu=neon -mfloat-abi=softfp
CFLAGS    = ${CFLAGS_CL} ${CFLAGS_V7} -Os -Wall -Werror=return-type -ansi -std=gnu99 $(DEFINES)
CXX_FLAGS = ${CFLAGS_CL} ${CFLAGS_V7} -Os -Wall -Werror=return-type -fno-rtti -fno-threadsafe-statics -fno-exceptions  -std=gnu++11 -ffunction-sections -fdata-sections $(DEFINES)

# 链接器选项
LNK_FLAGS = -nostdlib -T "$(VEX_SDK_PATH)/$(PLATFORM)/lscript.ld" -R "$(VEX_SDK_PATH)/$(PLATFORM)/stdlib_0.lib" -Map="$(BUILD)/$(PROJECT).map" --gc-section -L"$(VEX_SDK_PATH)/$(PLATFORM)" ${TOOL_LIB}

# 预留的静态库名称
PROJECTLIB = lib$(PROJECT)
ARCH_FLAGS = rcs

# 链接所需的库
LIBS =  --start-group -lv5rt -lstdc++ -lc -lm -lgcc --end-group

# 头文件搜索路径
INC += $(addprefix -I, ${INC_F})
INC += -I"$(VEX_SDK_PATH)/$(PLATFORM)/include"
INC += ${TOOL_INC}
