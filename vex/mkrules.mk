# VEXcode 构建规则，版本 2019_03_26_01

# 编译 C 源文件
$(BUILD)/%.o: %.c $(SRC_H)
	$(Q)$(MKDIR)
	$(ECHO) "CC  $<"
	$(Q)$(CC) $(CFLAGS) $(INC) -c -o $@ $<

# 编译 C++ 源文件
$(BUILD)/%.o: %.cpp $(SRC_H) $(SRC_A)
	$(Q)$(MKDIR)
	$(ECHO) "CXX $<"
	$(Q)$(CXX) $(CXX_FLAGS) $(INC) -c -o $@ $<

# 链接生成可执行文件
$(BUILD)/$(PROJECT).elf: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LINK) $(LNK_FLAGS) -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

# 生成机器人可下载的二进制文件
$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(Q)$(OBJCOPY) -O binary $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin

# 生成静态库
$(BUILD)/$(PROJECTLIB).a: $(OBJ)
	$(Q)$(ARCH) $(ARCH_FLAGS) $@ $^

# 清理工程构建产物
clean:
	$(info clean project)
	$(Q)$(CLEAN)
