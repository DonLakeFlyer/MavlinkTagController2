#pragma once

#include <cstdint>
#include <vector>

bool channelizerTuner(
        const uint32_t              sampleRateHz, 
        const uint32_t              nChannels, 
        const std::vector<uint32_t> freqListHz,
        uint32_t&                   bestCenterHz,
        std::vector<uint32_t>&      channelBins);