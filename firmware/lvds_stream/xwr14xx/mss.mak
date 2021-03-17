###################################################################################
# LVDS Stream: MSS Makefile
###################################################################################
.PHONY: mssTest mssTestClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c $(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/test/common
vpath %.c $(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/test/lvds_stream
vpath %.c $(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/test/lvds_stream/xwr14xx

###################################################################################
# The mmWave Unit Test requires additional libraries
###################################################################################
MSS_LVDS_STREAM_STD_LIBS = $(R4F_COMMON_STD_LIB)						\
			-llibesm_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT) 			\
   			-llibpinmux_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT) 		\
   			-llibuart_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
   			-llibcrc_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
   			-llibmailbox_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
   			-llibadcbuf_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		\
   			-llibedma_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		    \
   			-llibcbuff_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)		    \
   			-llibmmwavelink_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)    \
   			-llibmmwave_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)        \
   			-llibcli_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)			\
   			-llibhsiheader_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)

MSS_LVDS_STREAM_LOC_LIBS = $(R4F_COMMON_LOC_LIB)						\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/esm/lib				\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/pinmux/lib			\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/uart/lib			\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/crc/lib		        \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/mailbox/lib	        \
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwavelink/lib      \
			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/adcbuf/lib  		\
			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/cbuff/lib  			\
	        -i$(MMWAVE_SDK_INSTALL_PATH)/ti/drivers/edma/lib    		\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/control/mmwave/lib			\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/cli/lib				\
   			-i$(MMWAVE_SDK_INSTALL_PATH)/ti/utils/hsiheader/lib

###################################################################################
# Unit Test Files
###################################################################################
MSS_LVDS_STREAM_UNIT_TEST_CFG		 = $(MMWAVE_SDK_DEVICE_TYPE)/mss.cfg
MSS_LVDS_STREAM_UNIT_TEST_CMD		 = $(MMWAVE_SDK_INSTALL_PATH)/ti/platform/$(MMWAVE_SDK_DEVICE_TYPE)
MSS_LVDS_STREAM_UNIT_TEST_CONFIGPKG	 = $(MMWAVE_SDK_DEVICE_TYPE)/mss_configPkg_$(MMWAVE_SDK_DEVICE_TYPE)
MSS_LVDS_STREAM_UNIT_TEST_MAP		 = $(MMWAVE_SDK_DEVICE_TYPE)/$(MMWAVE_SDK_DEVICE_TYPE)_lvds_stream_mss.map
MSS_LVDS_STREAM_UNIT_TEST_OUT		 = $(MMWAVE_SDK_DEVICE_TYPE)/$(MMWAVE_SDK_DEVICE_TYPE)_lvds_stream_mss.$(R4F_EXE_EXT)
MSS_LVDS_STREAM_UNIT_TEST_BIN		 = $(MMWAVE_SDK_DEVICE_TYPE)/$(MMWAVE_SDK_DEVICE_TYPE)_lvds_stream_mss.bin
MSS_LVDS_STREAM_UNIT_TEST_APP_CMD	 = $(MMWAVE_SDK_DEVICE_TYPE)/mss_linker.cmd
MSS_LVDS_STREAM_UNIT_TEST_SOURCES	 = mss.c 					\
									   framework_core.c 		\
									   framework_ipc.c 			\
									   framework_ipc_local.c 	\
									   framework_listlib.c 		\
									   framework_xwr14xx.c		\
									   lvds_stream.c			\
									   cli.c
MSS_LVDS_STREAM_UNIT_TEST_DEPENDS	 = $(addprefix $(PLATFORM_OBJDIR)/, $(MSS_LVDS_STREAM_UNIT_TEST_SOURCES:.c=.$(R4F_DEP_EXT)))
MSS_LVDS_STREAM_UNIT_TEST_OBJECTS	 = $(addprefix $(PLATFORM_OBJDIR)/, $(MSS_LVDS_STREAM_UNIT_TEST_SOURCES:.c=.$(R4F_OBJ_EXT)))

###################################################################################
# RTSC Configuration:
###################################################################################
mssTestFullRTSC: $(R4_CFG)
	@echo 'Configuring RTSC packages...'
	$(XS) --xdcpath="$(XDCPATH)" xdc.tools.configuro $(R4F_XSFLAGS) -o $(MSS_LVDS_STREAM_UNIT_TEST_CONFIGPKG) $(MSS_LVDS_STREAM_UNIT_TEST_CFG)
	@echo 'Finished configuring packages'
	@echo ' '

###################################################################################
# Build Unit Test:
###################################################################################
mssTest: BUILD_CONFIGPKG=$(MSS_LVDS_STREAM_UNIT_TEST_CONFIGPKG)
mssTest: R4F_CFLAGS += --cmd_file=$(BUILD_CONFIGPKG)/compiler.opt
mssTest: buildDirectories mssTestFullRTSC $(MSS_LVDS_STREAM_UNIT_TEST_OBJECTS)
	$(R4F_LD) $(R4F_LDFLAGS) $(MSS_LVDS_STREAM_LOC_LIBS) $(MSS_LVDS_STREAM_STD_LIBS) 					\
	-l$(MSS_LVDS_STREAM_UNIT_TEST_CONFIGPKG)/linker.cmd --map_file=$(MSS_LVDS_STREAM_UNIT_TEST_MAP) 	\
	$(MSS_LVDS_STREAM_UNIT_TEST_OBJECTS) $(PLATFORM_R4F_LINK_CMD) $(MSS_LVDS_STREAM_UNIT_TEST_APP_CMD) 	\
	$(R4F_LD_RTS_FLAGS) -o $(MSS_LVDS_STREAM_UNIT_TEST_OUT)
	@echo '******************************************************************************'
	@echo 'Built the XWR14xx LVDS Stream '
	@echo '******************************************************************************'

###################################################################################
# Cleanup Unit Test:
###################################################################################
mssTestClean:
	@echo 'Cleaning the XWR14xx LVDS Stream objects'
	@$(DEL) $(MSS_LVDS_STREAM_UNIT_TEST_OBJECTS) $(MSS_LVDS_STREAM_UNIT_TEST_OUT) $(MSS_LVDS_STREAM_UNIT_TEST_BIN)
	@$(DEL) $(MSS_LVDS_STREAM_UNIT_TEST_MAP) $(MSS_LVDS_STREAM_UNIT_TEST_DEPENDS)
	@echo 'Cleaning the XWR14xx LVDS Stream RTSC package'
	@$(DEL) $(MSS_LVDS_STREAM_UNIT_TEST_CONFIGPKG)
	@$(DEL) $(PLATFORM_OBJDIR)

###################################################################################
# Dependency handling
###################################################################################
-include $(MSS_LVDS_STREAM_UNIT_TEST_DEPENDS)

