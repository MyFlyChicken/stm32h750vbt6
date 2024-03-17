#include "board.h"
#include "drv_common.h"
#include "drv_sound.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_sai.h"

#define DBG_TAG "drv.sound"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define CODEC_I2C_NAME ("i2c1")

#define TX_DMA_FIFO_SIZE (2048)

struct drv_sai _sai3_b = {0};

struct stm32_audio
{
    struct rt_i2c_bus_device* i2c_bus;
    struct rt_audio_device    audio;
    struct rt_audio_configure replay_config;
    int                       replay_volume;
    rt_uint8_t*               tx_fifo;
    rt_bool_t                 startup;
};
struct stm32_audio _stm32_audio_play = {0};

/**
 * @brief 
 * 
 * @details 参数分别对应sample_rate, PLL3M PLL3N PLL3P MCKDIV
 *          PLL3M=25 将PLL3的输入始终分频为25/25=1
 *          PLL3N    将PLL3输入分频后再进行倍频
 *          PLL3P    SAI3_B时钟源为PLL3P，这里设置PLL3P
 *          MCKDIV   samplerate = SAI时钟源 / (MCKDIV * 256 *(OSR+1))根据这个公式可以计算出对应值
 */
const rt_uint32_t SAI_PSC_TBL[][5] =
    {
        {AUDIO_FREQUENCY_008K, 25, 204, 10, 10},
        {AUDIO_FREQUENCY_011K, 25, 2, 52},
        {AUDIO_FREQUENCY_016K, 25, 38, 2},
        {AUDIO_FREQUENCY_032K, 25, 1, 52},
        {AUDIO_FREQUENCY_044K, 25, 226, 2, 10},
        {AUDIO_FREQUENCY_048K, 25, 246, 2, 10},
        {AUDIO_FREQUENCY_096K, 25, 295, 4, 3},
        {AUDIO_FREQUENCY_192K, 25, 295, 2, 3},
};

void SAIA_samplerate_set(rt_uint32_t freq)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
    int                      i;

    /* check frequence */
    for (i = 0; i < (sizeof(SAI_PSC_TBL) / sizeof(SAI_PSC_TBL[0])); i++)
    {
        if ((freq) == SAI_PSC_TBL[i][0])
            break;
    }
    if (i == (sizeof(SAI_PSC_TBL) / sizeof(SAI_PSC_TBL[0])))
    {
        LOG_E("Can not support this frequence: %d.", freq);
        return;
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI3;
    PeriphClkInitStruct.PLL3.PLL3M           = SAI_PSC_TBL[i][1];
    PeriphClkInitStruct.PLL3.PLL3N           = SAI_PSC_TBL[i][2];
    PeriphClkInitStruct.PLL3.PLL3P           = SAI_PSC_TBL[i][3];
    PeriphClkInitStruct.PLL3.PLL3Q           = 2;
    PeriphClkInitStruct.PLL3.PLL3R           = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE         = RCC_PLL3VCIRANGE_2;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL      = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN       = 0;
    PeriphClkInitStruct.Sai23ClockSelection  = RCC_SAI23CLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_SAI_DISABLE(&_sai3_b.hsai);
    _sai3_b.hsai.Init.AudioFrequency = freq;
    HAL_SAI_Init(&_sai3_b.hsai);
    __HAL_SAI_ENABLE(&_sai3_b.hsai);
}

void SAIA_channels_set(rt_uint16_t channels)
{
    if (channels == 2)
    {
        _sai3_b.hsai.Init.MonoStereoMode = SAI_STEREOMODE;
    }
    else
    {
        _sai3_b.hsai.Init.MonoStereoMode = SAI_MONOMODE;
    }
    __HAL_SAI_DISABLE(&_sai3_b.hsai);
    HAL_SAI_Init(&_sai3_b.hsai);
    __HAL_SAI_ENABLE(&_sai3_b.hsai);
}

void SAIA_samplebits_set(rt_uint16_t samplebits)
{
    switch (samplebits)
    {
    case 16:
        _sai3_b.hsai.Init.DataSize = SAI_DATASIZE_16;
        break;
    case 24:
        _sai3_b.hsai.Init.DataSize = SAI_DATASIZE_24;
        break;
    case 32:
        _sai3_b.hsai.Init.DataSize = SAI_DATASIZE_32;
        break;
    default:
        _sai3_b.hsai.Init.DataSize = SAI_DATASIZE_16;
        break;
    }
    __HAL_SAI_DISABLE(&_sai3_b.hsai);
    HAL_SAI_Init(&_sai3_b.hsai);
    __HAL_SAI_ENABLE(&_sai3_b.hsai);
}

void SAIA_config_set(struct rt_audio_configure config)
{
    SAIA_channels_set(config.channels);
    SAIA_samplerate_set(config.samplerate);
    SAIA_samplebits_set(config.samplebits);
}

rt_err_t SAI3B_pin_init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    /* Enable SAI3 clock */
    __HAL_RCC_SAI3_CLK_ENABLE();

    /* Configure GPIOs used for SAI1 */
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**SAI3_B_Block_B GPIO Configuration
    PD8     ------> SAI3_SCK_B
    PD9     ------> SAI3_SD_B
    PD10     ------> SAI3_FS_B
    PD14     ------> SAI3_MCLK_B
    */
    GPIO_Initure.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14;
    GPIO_Initure.Mode      = GPIO_MODE_AF_PP;
    GPIO_Initure.Pull      = GPIO_PULLUP;
    GPIO_Initure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Initure.Alternate = GPIO_AF10_SAI2;

    HAL_GPIO_Init(GPIOI, &GPIO_Initure);

    return RT_EOK;
}

/* initial sai A */
rt_err_t SAI3B_config_init(void)
{
    /* Initialize SAI */
    __HAL_SAI_RESET_HANDLE_STATE(&_sai3_b.hsai);

    _sai3_b.hsai.Instance = SAI3_Block_B;

    __HAL_SAI_DISABLE(&_sai3_b.hsai);

    _sai3_b.hsai.Init.AudioMode      = SAI_MODEMASTER_TX;
    _sai3_b.hsai.Init.Synchro        = SAI_ASYNCHRONOUS;
    _sai3_b.hsai.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
    _sai3_b.hsai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
    _sai3_b.hsai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
    _sai3_b.hsai.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
    _sai3_b.hsai.Init.Protocol       = SAI_FREE_PROTOCOL;
    _sai3_b.hsai.Init.DataSize       = SAI_DATASIZE_16;
    _sai3_b.hsai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
    _sai3_b.hsai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;

    //TODO frame 必须将FrameLength设置为64 ActiveFrameLength设置为32 否则会有很大的底噪。具体原因尚不明确
    _sai3_b.hsai.FrameInit.FrameLength       = 64;
    _sai3_b.hsai.FrameInit.ActiveFrameLength = 32;
    _sai3_b.hsai.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
    _sai3_b.hsai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
    _sai3_b.hsai.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

    //slot
    _sai3_b.hsai.SlotInit.FirstBitOffset = 0;
    _sai3_b.hsai.SlotInit.SlotSize       = SAI_SLOTSIZE_32B;
    _sai3_b.hsai.SlotInit.SlotNumber     = 2;
    _sai3_b.hsai.SlotInit.SlotActive     = (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1);

    if (HAL_OK != HAL_SAI_Init(&_sai3_b.hsai))
    {
        Error_Handler();
    };
    __HAL_SAI_ENABLE(&_sai3_b.hsai);

    return RT_EOK;
}

rt_err_t SAI3B_tx_dma(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_LINKDMA(&_sai3_b.hsai, hdmatx, _sai3_b.hdma);

    /* Select the DMA instance to be used for the transfer : DMA2_Stream3 */
    _sai3_b.hdma.Instance = DMA2_Stream3;

    _sai3_b.hdma.Init.Request   = DMA_REQUEST_SAI3_B;
    _sai3_b.hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    _sai3_b.hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    _sai3_b.hdma.Init.MemInc    = DMA_MINC_ENABLE;

    _sai3_b.hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    _sai3_b.hdma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;

    _sai3_b.hdma.Init.Mode          = DMA_CIRCULAR;
    _sai3_b.hdma.Init.Priority      = DMA_PRIORITY_HIGH;
    _sai3_b.hdma.Init.FIFOMode      = DMA_FIFOMODE_ENABLE;
    _sai3_b.hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    _sai3_b.hdma.Init.MemBurst      = DMA_MBURST_SINGLE;
    _sai3_b.hdma.Init.PeriphBurst   = DMA_PBURST_SINGLE;

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&_sai3_b.hdma);

    /* Configure the DMA Stream */
    if (HAL_OK != HAL_DMA_Init(&_sai3_b.hdma))
    {
        Error_Handler();
    }

    __HAL_DMA_DISABLE(&_sai3_b.hdma);

    __HAL_DMA_ENABLE_IT(&_sai3_b.hdma, DMA_IT_TC);
    __HAL_DMA_CLEAR_FLAG(&_sai3_b.hdma, DMA_FLAG_TCIF3_7);
    /* set nvic */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    return RT_EOK;
}

void DMA2_Stream3_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(_sai3_b.hsai.hdmatx);
    rt_interrupt_leave();
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    rt_audio_tx_complete(&_stm32_audio_play.audio);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef* hsai)
{
    rt_audio_tx_complete(&_stm32_audio_play.audio);
}

rt_err_t sai_init(void)
{
    SAI3B_pin_init();
    SAI3B_tx_dma();
    SAI3B_config_init();

    return RT_EOK;
}

static rt_err_t stm32_player_getcaps(struct rt_audio_device* audio, struct rt_audio_caps* caps)
{
    rt_err_t            result   = RT_EOK;
    struct stm32_audio* st_audio = (struct stm32_audio*)audio->parent.user_data;

    LOG_D("%s:main_type: %d, sub_type: %d", __FUNCTION__, caps->main_type, caps->sub_type);

    switch (caps->main_type)
    {
    case AUDIO_TYPE_QUERY: /* query the types of hw_codec device */
    {
        switch (caps->sub_type)
        {
        case AUDIO_TYPE_QUERY:
            caps->udata.mask = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_MIXER;
            break;

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }

    case AUDIO_TYPE_OUTPUT: /* Provide capabilities of OUTPUT unit */
    {
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM:
            caps->udata.config.channels   = st_audio->replay_config.channels;
            caps->udata.config.samplebits = st_audio->replay_config.samplebits;
            caps->udata.config.samplerate = st_audio->replay_config.samplerate;
            break;

        case AUDIO_DSP_SAMPLERATE:
            caps->udata.config.samplerate = st_audio->replay_config.samplerate;
            break;

        case AUDIO_DSP_CHANNELS:
            caps->udata.config.channels = st_audio->replay_config.channels;
            break;

        case AUDIO_DSP_SAMPLEBITS:
            caps->udata.config.samplebits = st_audio->replay_config.samplebits;
            break;

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }

    case AUDIO_TYPE_MIXER: /* report the Mixer Units */
    {
        switch (caps->sub_type)
        {
        case AUDIO_MIXER_QUERY:
            caps->udata.mask = AUDIO_MIXER_VOLUME | AUDIO_MIXER_LINE;
            break;

        case AUDIO_MIXER_VOLUME:
            caps->udata.value = st_audio->replay_volume;
            break;

        case AUDIO_MIXER_LINE:
            break;

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }

    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}

static rt_err_t stm32_player_configure(struct rt_audio_device* audio, struct rt_audio_caps* caps)
{
    rt_err_t            result   = RT_EOK;
    struct stm32_audio* st_audio = (struct stm32_audio*)audio->parent.user_data;

    LOG_D("%s:main_type: %d, sub_type: %d", __FUNCTION__, caps->main_type, caps->sub_type);

    switch (caps->main_type)
    {
    case AUDIO_TYPE_MIXER: {
        switch (caps->sub_type)
        {
        case AUDIO_MIXER_MUTE: {
            /* set mute mode */
            //wm8978_mute_enabled(_stm32_audio_play.i2c_bus, RT_FALSE);
            break;
        }

        case AUDIO_MIXER_VOLUME: {
            int volume = caps->udata.value;

            st_audio->replay_volume = volume;
            /* set mixer volume */
            //wm8978_set_volume(_stm32_audio_play.i2c_bus, volume);

            break;
        }

        default:
            result = -RT_ERROR;
            break;
        }

        break;
    }

    case AUDIO_TYPE_OUTPUT: {
        switch (caps->sub_type)
        {
        case AUDIO_DSP_PARAM: {
            struct rt_audio_configure config = caps->udata.config;

            st_audio->replay_config.samplerate = config.samplerate;
            st_audio->replay_config.samplebits = config.samplebits;
            st_audio->replay_config.channels   = config.channels;

            SAIA_config_set(config);
            break;
        }

        case AUDIO_DSP_SAMPLERATE: {
            st_audio->replay_config.samplerate = caps->udata.config.samplerate;
            SAIA_samplerate_set(caps->udata.config.samplerate);
            break;
        }

        case AUDIO_DSP_CHANNELS: {
            st_audio->replay_config.channels = caps->udata.config.channels;
            SAIA_channels_set(caps->udata.config.channels);
            break;
        }

        case AUDIO_DSP_SAMPLEBITS: {
            st_audio->replay_config.samplebits = caps->udata.config.samplebits;
            SAIA_samplebits_set(caps->udata.config.samplebits);
            break;
        }

        default:
            result = -RT_ERROR;
            break;
        }
        break;
    }

    default:
        break;
    }

    return result;
}

static rt_err_t stm32_player_init(struct rt_audio_device* audio)
{
    /* initialize wm8978 */
    //_stm32_audio_play.i2c_bus = (struct rt_i2c_bus_device*)rt_device_find(CODEC_I2C_NAME);

    sai_init();
    //wm8978_init(_stm32_audio_play.i2c_bus);
    return RT_EOK;
}

static rt_err_t stm32_player_start(struct rt_audio_device* audio, int stream)
{
    if (stream == AUDIO_STREAM_REPLAY)
    {
        HAL_SAI_Transmit_DMA(&_sai3_b.hsai, _stm32_audio_play.tx_fifo, TX_DMA_FIFO_SIZE / 2);
        //wm8978_player_start(_stm32_audio_play.i2c_bus);
    }

    return RT_EOK;
}

static rt_err_t stm32_player_stop(struct rt_audio_device* audio, int stream)
{
    if (stream == AUDIO_STREAM_REPLAY)
    {
        HAL_SAI_DMAStop(&_sai3_b.hsai);
    }

    return RT_EOK;
}

static void stm32_player_buffer_info(struct rt_audio_device* audio, struct rt_audio_buf_info* info)
{
    /**
     *               TX_FIFO
     * +----------------+----------------+
     * |     block1     |     block2     |
     * +----------------+----------------+
     *  \  block_size  /
     */
    info->buffer      = _stm32_audio_play.tx_fifo;
    info->total_size  = TX_DMA_FIFO_SIZE;
    info->block_size  = TX_DMA_FIFO_SIZE / 2;
    info->block_count = 2;
}
static struct rt_audio_ops _p_audio_ops =
    {
        .getcaps     = stm32_player_getcaps,
        .configure   = stm32_player_configure,
        .init        = stm32_player_init,
        .start       = stm32_player_start,
        .stop        = stm32_player_stop,
        .transmit    = RT_NULL,
        .buffer_info = stm32_player_buffer_info,
};

int rt_hw_sound_init(void)
{
    rt_uint8_t*        tx_fifo;
    struct rt_memheap* memheap;
#if 1
    /* player */
    memheap = sys_memheap_get(MEMHEAP_AXI_SRAM);
    if (memheap != RT_NULL)
    {
        tx_fifo = rt_memheap_alloc(memheap, TX_DMA_FIFO_SIZE);
        if (tx_fifo == RT_NULL)
        {
            return -RT_ENOMEM;
        }
    }
    else
    {
        return -RT_ENOMEM;
    }

#else
    rt_align(4) static rt_uint8_t buffer[2048] __attribute__((at(0x24001000)));
    tx_fifo = buffer;
#endif

    rt_memset(tx_fifo, 0, TX_DMA_FIFO_SIZE);
    _stm32_audio_play.tx_fifo = tx_fifo;

    /* register sound device */
    _stm32_audio_play.audio.ops = &_p_audio_ops;
    rt_audio_register(&_stm32_audio_play.audio, "sound0", RT_DEVICE_FLAG_WRONLY, &_stm32_audio_play);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_sound_init);
