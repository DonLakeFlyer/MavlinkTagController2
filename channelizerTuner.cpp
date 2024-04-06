#include "channelizerTuner.h"
#include "log.h"

#include <algorithm>
#include <numeric>
#include <functional>

bool channelizerTuner(
        const uint32_t              sampleRateHz, 
        const uint32_t              nChannels, 
        const std::vector<uint32_t> rawFreqListHz,
        uint32_t&                   bestCenterHz,
        std::vector<uint32_t>&      channelBins)
{
    uint32_t freqMaxHz          = *std::max_element(rawFreqListHz.begin(), rawFreqListHz.end());
    uint32_t freqMinHz          = *std::min_element(rawFreqListHz.begin(), rawFreqListHz.end());
    uint32_t fullBwHz           = sampleRateHz;
    uint32_t halfBwHz           = fullBwHz / 2;
    uint32_t channelBwHz        = sampleRateHz / nChannels;
    uint32_t halfChannelBwHz    = channelBwHz / 2;

    // First make sure all the requested frequencies can fit within the available bandwidth
    if (freqMaxHz - freqMinHz > sampleRateHz) {
        logError() << "Requested frequencies are too far apart to fit within the available bandwidth";
        return false;
    }

    // Remove duplicates
    std::vector<uint32_t> cleanedFreqListHz = rawFreqListHz;
    std::sort(cleanedFreqListHz.begin(), cleanedFreqListHz.end());
    cleanedFreqListHz.erase(std::unique(cleanedFreqListHz.begin(), cleanedFreqListHz.end()), cleanedFreqListHz.end());

    // If there is only a single requested frequency, then we just position it within the center of a
    // channel which is nearest to center.
    if (cleanedFreqListHz.size() == 1) {
        auto singleFreqHz   = cleanedFreqListHz[0];
        auto channelBin     = nChannels / 2;

        if (nChannels % 2 == 0) {
            bestCenterHz = singleFreqHz + halfChannelBwHz;
        } else {
            bestCenterHz = singleFreqHz;
        }

        channelBins.resize(1);
        channelBins[0] = channelBin + 1;

        return true;
    }


    // We determine best possible center frequencies by looping through possible choices and assigning
    // weights to each.

    // Generate the list of center channel choices to test against
    std::vector<uint32_t> testCentersHz;
    uint32_t stepSizeHz = 1000;
    uint32_t nextPossibleHz = freqMinHz + halfChannelBwHz;
    while (nextPossibleHz <= freqMaxHz - halfChannelBwHz) {
        testCentersHz.push_back(nextPossibleHz);
        nextPossibleHz += stepSizeHz;
    }
    logDebug() << "testCentersHz:" << testCentersHz;

    std::vector<bool>                   usableTestCenters;
    std::vector<uint32_t>               channelUsageCountsForTestCenter;
    std::vector<uint32_t>               distanceFromCenterAverages;
    std::vector<std::vector<uint32_t>>  channelBinsArray;
    
    std::vector<bool>                   isFreqInShoulder;
    std::vector<std::vector<uint32_t>>  channelUsageCountsArray;
    channelUsageCountsForTestCenter.resize(nChannels);

    for (auto testCenterHz : testCentersHz) {
        std::for_each(channelUsageCountsForTestCenter.begin(), channelUsageCountsForTestCenter.end(), [](uint32_t& n) { n = 0; });

        std::cout << "Channels edges: ";
        int nextChannelStartFreqHz = testCenterHz - halfBwHz;
        for (uint32_t i=0; i<nChannels; i++) {
            std::cout << nextChannelStartFreqHz << " ";
            nextChannelStartFreqHz += channelBwHz;
        }
        std::cout << std::endl;

        std::vector<uint32_t> distanceFromCenters;
        std::vector<uint32_t> channelBins;

        for (auto requestedFreqHz : cleanedFreqListHz) {
            // Adjust the requested frequency to be zero-based withing the available total bandwidth
            int32_t adjustedFreqHz = requestedFreqHz - (testCenterHz - halfBwHz);

            // Find the frequency position within the channel bandwidth
            auto hzPositionWithinChannel = adjustedFreqHz % channelBwHz;

            // Find the distance from the center of the channel
            uint32_t distanceFromCenter = abs((int)hzPositionWithinChannel - (int)halfChannelBwHz);

            distanceFromCenters.push_back(distanceFromCenter);

            // Keep track of the number of frequencies which are within each channel bandwidth
            auto channelBucket = adjustedFreqHz / channelBwHz;

            logDebug() << "requestedFreqHz:" << requestedFreqHz << 
                "testCenterHz:" << testCenterHz << 
                "adjustedFreqHz:" << adjustedFreqHz << 
                "halfBwHz:" << halfBwHz <<
                "channelBwHz:" << channelBwHz <<
                "hzPositionWithinChannel:" << hzPositionWithinChannel <<
                "distanceFromCenter:" << distanceFromCenter <<
                "channelBins:" << channelBins <<
                "channelBucket:" << channelBucket;

            channelBins.push_back(channelBucket + 1);

            channelUsageCountsForTestCenter[channelBucket]++;
        }

        // Find the average of the distances from the center of each channel
        auto averageDistanceFromCenter = std::accumulate(distanceFromCenters.begin(), distanceFromCenters.end(), 0) / distanceFromCenters.size();
        distanceFromCenterAverages.push_back(averageDistanceFromCenter);

        channelBinsArray.push_back(channelBins);
    }

    // Find the best center frequency

    uint32_t smallestDistanceFromCenterAverage = channelBwHz;
    uint32_t bestTestCenterHz = 0;
    for (size_t i = 0; i < testCentersHz.size(); i++) {
        auto testCenterHz = testCentersHz[i];

        // We can't use a center frequency if it has more than one frequency in the same channel
        if (std::any_of(channelUsageCountsForTestCenter.begin(), channelUsageCountsForTestCenter.end(), [](uint32_t n) { return n > 1; })) {
            continue;
        }

        // We can't use a center frequency if it has a requested frequency within the shoulder of the channel
        double  shoulderPercent = 0.925;
        auto    centerBwHz      = channelBwHz * shoulderPercent;
        if (std::any_of(distanceFromCenterAverages.begin(), distanceFromCenterAverages.end(), 
                std::bind([](uint32_t n, uint32_t centerBwHz) { return n > centerBwHz; }, std::placeholders::_1, centerBwHz))) {
            continue;
        }

        if (distanceFromCenterAverages[i] < smallestDistanceFromCenterAverage) {
            smallestDistanceFromCenterAverage = distanceFromCenterAverages[i];
            bestTestCenterHz = testCenterHz;
            channelBins = channelBinsArray[i];
        }
    }

    if (bestTestCenterHz != 0) {
        bestCenterHz = bestTestCenterHz;
        return false;
    }

    return bestTestCenterHz != 0;
}