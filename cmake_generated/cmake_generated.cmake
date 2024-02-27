# This is converter generated file, and shall not be touched by user
#
# It is always re-generated if converter script is called multiple times
# Use CMakeLists.txt to apply user changes
cmake_minimum_required(VERSION 3.22)

# Core MCU flags, CPU, instruction set and FPU setup
set(cpu_PARAMS ${cpu_PARAMS}
    -mthumb

    # -mcpu, -mfloat, -mfloat-abi config
    -mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)

# Linker script
set(linker_script_SRC ${linker_script_SRC} ${CMAKE_CURRENT_SOURCE_DIR}/board/linker_scripts/link.lds)

# Sources
set(sources_SRCS ${sources_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/applications/main.c
	${CMAKE_CURRENT_SOURCE_DIR}/applications/usb_init.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/osal/usb_osal_rtthread.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/port/dwc2/usb_glue_st.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/demo/cdc_acm_template.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/class/cdc/usbd_cdc.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/third_party/rt-thread-5.0/msh_cmd.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/core/usbd_core.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/port/dwc2/usb_dc_dwc2.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/common/cctype.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/common/cstdio.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/common/cstdlib.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/common/cstring.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/common/ctime.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/common/cwchar.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/newlib/syscalls.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/libcpu/arm/common/atomic_arm.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/libcpu/arm/common/div0.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/libcpu/arm/common/showmem.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m7/context_gcc.S
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m7/cpu_cache.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m7/cpuport.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/audio/audio.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/audio/audio_pipe.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/i2c/i2c-bit-ops.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/i2c/i2c_core.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/i2c/i2c_dev.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/ipc/completion.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/ipc/dataqueue.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/ipc/pipe.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/ipc/ringblk_buf.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/ipc/ringbuffer.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/ipc/waitqueue.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/ipc/workqueue.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/misc/pin.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/misc/rt_drv_pwm.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/mtd/mtd_nor.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/rtc/alarm.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/rtc/rtc.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/sdio/block_dev.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/sdio/gpt.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/sdio/mmc.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/sdio/mmcsd_core.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/sdio/sd.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/sdio/sdio.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/serial/serial_v2.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/spi/qspi_core.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/spi/sfud/src/sfud.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/spi/sfud/src/sfud_sfdp.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/spi/spi_core.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/spi/spi_dev.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/spi/spi_flash_sfud.c
	${CMAKE_CURRENT_SOURCE_DIR}/board/CubeMX_Config/Src/stm32h7xx_hal_msp.c
	${CMAKE_CURRENT_SOURCE_DIR}/board/board.c
	#${CMAKE_CURRENT_SOURCE_DIR}/board/port/audio/drv_sound.c
	#${CMAKE_CURRENT_SOURCE_DIR}/board/port/audio/drv_wm8978.c
	${CMAKE_CURRENT_SOURCE_DIR}/board/port/drv_qspi_flash.c
	${CMAKE_CURRENT_SOURCE_DIR}/board/port/drv_spi_flash.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h750xx.s
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_common.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_flash/drv_flash_h7.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_gpio.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_pwm.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_qspi.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_rtc.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_sdmmc.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_soft_i2c.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_spi.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_tim.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_usart_v2.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/fal/src/fal.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/fal/src/fal_flash.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/fal/src/fal_rtt.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/fal/src/fal_partition.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/fal/samples/porting/fal_flash_sfud_port.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/filesystems/devfs/devfs.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/filesystems/elmfat/dfs_elm.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/filesystems/elmfat/ff.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/filesystems/elmfat/ffunicode.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/src/dfs.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/src/dfs_file.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/src/dfs_fs.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/src/dfs_posix.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/finsh/shell.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/finsh/msh.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/finsh/msh_parse.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/finsh/cmd.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/finsh/msh_file.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/scalfact.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/dequant.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/huffman.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/stproc.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/dqchan.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/arm/asmmisc_gcc.s
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/mp3tabs.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/subband.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/imdct.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/hufftabs.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/mp3dec.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/trigtabs.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/dct32.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/buffers.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/bitstream.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real/arm/asmpoly_thumb2_gcc.s
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/clock.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/components.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/device.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/idle.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/ipc.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/irq.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/kservice.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/memheap.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/mempool.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/object.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/scheduler_up.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/thread.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/src/timer.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sai_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sram.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2s.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cryp_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cryp.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2s_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_sdmmc.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_usart.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_lptim.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_delayblock.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_crc_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_nor.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_comp.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_crc.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cec.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_qspi.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sai.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/CMSIS/Device/ST/STM32H7xx/Source/Templates/system_stm32h7xx.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/mp3player-latest/src/mp3_player.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/mp3player-latest/src/mp3_tag.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/mp3player-latest/src/mp3_player_cmd.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/optparse-latest/optparse.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/posix/io/stdio/libc.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/utilities/ulog/ulog.c
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/utilities/ulog/backend/console_be.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/wavplayer-latest/src/wavplayer.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/wavplayer-latest/src/wavplayer_cmd.c
	${CMAKE_CURRENT_SOURCE_DIR}/packages/wavplayer-latest/src/wavhdr.c
)

# Include directories
set(include_c_DIRS ${include_c_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/applications
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/common
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/core
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/class/cdc
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/class/msc
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/class/hid
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/class/audio
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/class/video
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/class/wireless
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/class/dfu
	${CMAKE_CURRENT_SOURCE_DIR}/packages/CherryUSB-latest/osal
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/common/include
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/compilers/newlib
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/libcpu/arm/common
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/libcpu/arm/cortex-m7
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/audio
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/include
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/spi
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/drivers/spi/sfud/inc
	${CMAKE_CURRENT_SOURCE_DIR}/board
	${CMAKE_CURRENT_SOURCE_DIR}/board/CubeMX_Config/Inc
	${CMAKE_CURRENT_SOURCE_DIR}/board/port
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/config
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/CMSIS/Include
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/HAL_Drivers/drv_flash
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/fal/inc
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/include
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/filesystems/devfs
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/dfs/dfs_v1/filesystems/elmfat
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/finsh
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/pub
	${CMAKE_CURRENT_SOURCE_DIR}/packages/helix-latest/real
	${CMAKE_CURRENT_SOURCE_DIR}/.
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/include
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/STM32H7xx_HAL_Driver/Inc
	${CMAKE_CURRENT_SOURCE_DIR}/libraries/STM32H7xx_HAL/CMSIS/Device/ST/STM32H7xx/Include
	${CMAKE_CURRENT_SOURCE_DIR}/packages/mp3player-latest
	${CMAKE_CURRENT_SOURCE_DIR}/packages/mp3player-latest/inc
	${CMAKE_CURRENT_SOURCE_DIR}/packages/optparse-latest
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/posix/io/poll
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/posix/io/stdio
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/libc/posix/ipc
	${CMAKE_CURRENT_SOURCE_DIR}/rt-thread/components/utilities/ulog
	${CMAKE_CURRENT_SOURCE_DIR}/packages/wavplayer-latest
	${CMAKE_CURRENT_SOURCE_DIR}/packages/wavplayer-latest/inc
)
set(include_cxx_DIRS ${include_cxx_DIRS})
set(include_asm_DIRS ${include_asm_DIRS})

# Symbols definition
set(symbols_c_SYMB ${symbols_c_SYMB}
    "CONFIG_USB_DWC2_PORT=FS_PORT"
	"RT_USING_LIBC"
	"STM32H750xx"
	"USE_HAL_DRIVER"
	"_POSIX_C_SOURCE=1"
	"__RTTHREAD__"
)
set(symbols_cxx_SYMB ${symbols_cxx_SYMB})
set(symbols_asm_SYMB ${symbols_asm_SYMB}
    "DEBUG"
)

# Link directories
set(link_DIRS ${link_DIRS}{{sr:link_DIRS}})

# Link libraries
set(link_LIBS ${link_LIBS}
    c
    m
)

# Compiler options
set(compiler_OPTS ${compiler_OPTS})

# Linker options
set(linker_OPTS ${linker_OPTS})
