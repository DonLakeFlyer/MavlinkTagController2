#!/usr/bin/env python3
# -*- coding: utf-8 -*-    %
#----------------------------------------------------------------------------
# Created By  : Michael W. Shafer
# Created Date: 2023-01-02
# version ='1.0'
# ---------------------------------------------------------------------------
"""
TUNER.PY Selects the radio center frequency well. This functions tries
to pick the radio center frequency that minimizes the number of tags in
each channel and then minimizes the edge cost. The channel edge cost is
defined such that tags within 7.5% of the channel edges received a cost
that is higher the closer they are to the edge. Cost is defined as
infinite for being located at the channel edge. Cost is minimize in order
to locate tag frequencies away from of each channel's shoulder rolloff.
The program also provides various outputs associate with the channels
that are all defined below in the outputs section. The program will raise
an exception if the frequencies between the input tag definitions is larger
than the radio bandwidth.
INPUTS:
    Fs              Positive scalar of number of samples per second (Hz)
                    of the  radio.
    nChannels       Number of channels the radio data is split into.
    tagFreqVecMHz   A vector of tag transmission frequencies in MHz
 OUTPUTS:
    radioFc         Optimal radio center frequency in MHz
    channelFcVec    The center frequencies of each channel (ascending) in
                    MHz
    tagChannelNum   The index of the channel that each tag should be
                    present in
    tagChannelEdgeWarning
                    A logical list that highlights tags that are
                    outside of the 90% of the bandwidth of their
                    channel to highlight tags that might be on the
                    rolloff shoulders of a channel.
    multipleTagsInChannelWarning
                    A single logical to warn calling
                    function that there will be multiple tags in a single
                    channel.
EXAMPLES:
Fs = 192000
nChannels = 48
tagFreqVecMHz = np.array([149.922727, 149.928381, 149.934261, 149.957471, 149.997192, 150.009001, 150.025413, 150.077912, 150.087777])
[radioFc,channelFcVec, tagChannelNum, tagChannelEdgeWarning, multipleTagsInChannelWarning] = tuner(Fs, nChannels, tagFreqVecMHz)
    """
# ---------------------------------------------------------------------------
import numpy as np
from scipy import interpolate
from time import sleep
# ---------------------------------------------------------------------------

def tuner(Fs, nChannels, tagFreqVecMHz):

    # bwFlatFrac    = 0.9;
    # tagFreqVecMHz = tagFreqVecMHz(:);%Force column vector
    # tagFreqVecMHz = sort(tagFreqVecMHz);
    # nTags         = numel(tagFreqVecMHz);
    # channelBWHz   = Fs/nChannels;
    # channelBWMHz  = channelBWHz*10^-6;
    # fNyq          = Fs/2;
    # fMin          = min(tagFreqVecMHz);
    # fMax          = max(tagFreqVecMHz);

    bwFlatFrac    = 0.85 #Definition for channel edge warning

    tagFreqVecMHz = np.around(tagFreqVecMHz, 6)[:, np.newaxis] #make column vector

    tagFreqVecMHzSorted = np.sort(tagFreqVecMHz , 0)
    
    sortedInds    = np.argsort(tagFreqVecMHz , 0) 

    inputFreqOrder = np.argsort(sortedInds , 0 )

    nTags         = np.size(tagFreqVecMHzSorted)

    channelBWHz   = Fs/nChannels

    channelBWMHz  = channelBWHz*10**-6

    shoulderWidthMHz = channelBWMHz*(1-bwFlatFrac)

    fNyq          = Fs/2;

    fMin          = np.amin(tagFreqVecMHzSorted);

    fMax          = np.amax(tagFreqVecMHzSorted);
 
    # if fMax-fMin > Fs
    #     error('UAV-RT: Max difference in tag frequencies must be less than the radio sample rate input Fs.')
    # end

    if fMax - fMin > Fs:
        raise Exception("UAV-RT: Max difference in tag frequencies must be less than the radio sample rate input Fs.")

    # %Determine the channel center frequecies away from baseband
    # if mod(nChannels,2)~=0
    #     channelVec = 1/10^6 * ( (-channelBWHz * floor(nChannels/2) ) : ...
    #                             channelBWHz : fNyq ).';
    # else
    #     channelVec = 1/10^6 * (-fNyq : channelBWHz : (fNyq - channelBWHz)).';
    # end
    # radioBWUpper = abs(max(channelVec))+channelBWMHz/2;
    # radioBWLower = abs(min(channelVec))+channelBWMHz/2;

    if nChannels%2 != 0:
        channelVec = 10**-6*np.arange(-channelBWHz*np.floor(nChannels/2),fNyq+channelBWHz,channelBWHz)[:, np.newaxis]

    else:
        channelVec = 10**-6*np.arange(-fNyq, fNyq, channelBWHz)[:, np.newaxis]

    radioBWUpper = abs(np.amax(channelVec))+channelBWMHz/2

    radioBWLower = abs(np.amin(channelVec))+channelBWMHz/2

    # %Generate a list of potential radio center frequency options. Only use
    # %those options that can include all tags in the bandwidth
    # fCentOptions = round((fMin:0.0001:fMax).',6);%Round to nearest integer Hz
    # fCentOptionsUpperFreq = fCentOptions+radioBWUpper;
    # fCentOptionsLowerFreq = fCentOptions-radioBWLower;
    # fCentOptionsValid = fCentOptionsLowerFreq <= fMin & fCentOptionsUpperFreq >= fMax;
    # fCentOptions = fCentOptions(fCentOptionsValid);
    # nValidCentFreqs = numel(fCentOptions);
    if fMin != fMax:
        fCentOptions      = np.around(np.arange(fMin, fMax , 0.0001), 6)[:, np.newaxis]
    else:
        fCentOptions      = fMax

    fCentOptionsUpperFreq = fCentOptions + radioBWUpper

    fCentOptionsLowerFreq = fCentOptions - radioBWLower

    fCentOptionsValid     = np.logical_and(fCentOptionsLowerFreq <= fMin, fCentOptionsUpperFreq >= fMax)

    fCentOptions          = np.extract(fCentOptionsValid, fCentOptions)[:, np.newaxis]

    nValidCentFreqs       = np.size(fCentOptions)

    # %Here we will investigate each center frequency option and calculate the
    # %distance to the channel edges for each tag.
    # freqError = zeros(size(fCentOptions));
    # freqDistToChannelEdge = zeros(nValidCentFreqs, nTags);
    # for i = 1:nValidCentFreqs
    #     centerFreqs = fCentOptions(i) + channelVec;
    #     nearestCenterFreq = interp1(centerFreqs,centerFreqs,tagFreqVecMHzSorted,'nearest','extrap');
    #     freqErrorAllTags = nearestCenterFreq - tagFreqVecMHzSorted;
    #     freqDistToChannelEdge(i,:) = (channelBWMHz/2 - abs(freqErrorAllTags)).';
    #     freqError(i) = sum(abs(nearestCenterFreq - tagFreqVecMHzSorted));
    # end
    # %Deal with numerical precision. Round to integer Hz
    # freqDistToChannelEdge = round(freqDistToChannelEdge,6);


    freqDistToChannelEdge = np.zeros((nValidCentFreqs, nTags))

    for i in range(0, nValidCentFreqs):
        centerFreqs           = np.squeeze(fCentOptions[i] + channelVec)

        nearestCenterFreqFunc = interpolate.interp1d(centerFreqs, centerFreqs, kind = 'nearest', bounds_error = False, fill_value = 'extrapolate')

        nearestCenterFreq     = nearestCenterFreqFunc( tagFreqVecMHzSorted )

        freqErrorAllTags      = nearestCenterFreq - tagFreqVecMHzSorted

        freqDistToChannelEdge[i,:] = np.transpose(channelBWMHz/2 - abs(freqErrorAllTags))

    # %Deal with numerical precision. Round to integer Hz
    freqDistToChannelEdge     = np.around(freqDistToChannelEdge, decimals = 6);

    # %Flag those tags near a channel edge and assign a cost. Zero cost if not
    # %near edge.
    # nearChannelEdgeLogic  = freqDistToChannelEdge < channelBWMHz/2*(1-bwFlatFrac);
    # channelEdgeCostAllTag = freqDistToChannelEdge;
    # channelEdgeCostAllTag(~nearChannelEdgeLogic) = 0;
    # channelEdgeCost       = sum(channelEdgeCostAllTag,2);

    nearChannelEdgeLogic  = freqDistToChannelEdge < shoulderWidthMHz/2

    channelEdgeCostAllTag = np.abs( (shoulderWidthMHz/2*( freqDistToChannelEdge**-1 ) ) - 1)

    channelEdgeCostAllTag[~nearChannelEdgeLogic] = 0

    channelEdgeCost       = np.sum(channelEdgeCostAllTag, axis = 1)[:, np.newaxis]

    # %Ideally, we'd choose the center frequency that minimizes cost, but
    # %we also need to minimize the number of tags in each channel. Having one
    # %tag per channel is the prioity. Minimize the number of tags per
    # %channel first and then of those that minimize, pick the one that minimize
    # %the tag near channel edge cost.

    # %Here we calculate the number of tags per channel for each of the different
    # %center frequency options.
    # tagsPerChannelMat = zeros(nChannels, nValidCentFreqs);
    # for j = 1:nValidCentFreqs
    #     centerFreqs = fCentOptions(j) + channelVec;
    #     fEdgeLower  = centerFreqs-channelBWMHz;
    #     fEdgeUpper  = centerFreqs+channelBWMHz;
    #     inBandLogic = zeros(nChannels, nTags);
    #     for i = 1:nTags
    #       inBandLogic(:,i) = ( tagFreqVecMHzSorted(i) > fEdgeLower ) & ...
    #                          ( tagFreqVecMHzSorted(i) <= fEdgeUpper );
    #     end
    #     tagsPerChannelMat(:,j) = sum(inBandLogic, 2);
    # end
    # tagsPerChannelMat(tagsPerChannelMat==0) = NaN;
    # tagPerChannelMean = mean(tagsPerChannelMat,1,'omitnan').';

    tagsPerChannelMat = np.zeros((nChannels, nValidCentFreqs))

    for j in range(0, nValidCentFreqs):
        centerFreqs = fCentOptions[j] + channelVec

        fEdgeLower  = centerFreqs - channelBWMHz/2

        fEdgeUpper  = centerFreqs + channelBWMHz/2

        inBandLogic = np.zeros( (nChannels, nTags) )

        for i in range(0, nTags):
            inBandLogic[:,i] = np.squeeze(np.logical_and(( tagFreqVecMHzSorted[i] > fEdgeLower ) , ( tagFreqVecMHzSorted[i] <= fEdgeUpper )))

        tagsPerChannelMat[:, j] = np.sum(inBandLogic, axis = 1)

    tagsPerChannelMat[tagsPerChannelMat == 0 ] = float('NaN')

    tagPerChannelMean = np.nanmean(tagsPerChannelMat, axis = 0)[:, np.newaxis];


    # %Now build a list of the channel edge cost options and the tags per channel
    # %cost. We want to minimize the number of tags per channel first and then
    # %from that minimize the channel edge cost.
    # acceptableFreqLogic   = false( nValidCentFreqs , 1 );
    # channelEdgeCostList   = sort(unique(channelEdgeCost));
    # tagPerChannelCost     = tagPerChannelMean;
    # tagPerChannelCostList = sort(unique(tagPerChannelCost));
    # tick1 = 1;
    # tick2 = 1;
    # while ~any(acceptableFreqLogic)
    #     currTagPerChannelCost = tagPerChannelCostList(tick2);
    #     %Of all those options whose tagsPerChannelMean equal to the minimum in that
    #     %list, pick the ones that has the lowest minFreqError.
    #     tagPerChannelLogic = tagPerChannelMean == currTagPerChannelCost;

    #     while ~any(acceptableFreqLogic)
    #         currChannelEdgeCost = channelEdgeCostList(tick1);
    #         channelEdgeCostLogic = channelEdgeCost == currChannelEdgeCost;
    #         acceptableFreqLogic    = tagPerChannelLogic & channelEdgeCostLogic;
    #         tick1 = tick1 + 1;
    #     end
    #     tick2 = tick2 + 1;
    # end

    acceptableFreqLogic   = np.zeros(( nValidCentFreqs , 1 ), dtype=bool)

    channelEdgeCostList   = np.sort(np.unique(channelEdgeCost))

    tagPerChannelCost     = tagPerChannelMean

    tagPerChannelCostList = np.sort(np.unique(tagPerChannelCost))

    tick1 = 0

    tick2 = 0

    while ~np.any(acceptableFreqLogic):
        currTagPerChannelCost = tagPerChannelCostList[tick2]
        #Of all those options whose tagsPerChannelMean equal to the minimum in that
        #list, pick the ones that has the lowest minFreqError.

        tagPerChannelLogic   = tagPerChannelMean == currTagPerChannelCost;

        while ~np.any(acceptableFreqLogic):
            currChannelEdgeCost  = channelEdgeCostList[tick1];

            channelEdgeCostLogic = channelEdgeCost == currChannelEdgeCost

            acceptableFreqLogic  = np.logical_and(tagPerChannelLogic, channelEdgeCostLogic)

            tick1 = tick1 + 1

        tick2 = tick2 + 2

    # %There may be multiple options where tags per channel is minimized, as is
    # %tag channel edge cost. They are all acceptable options. Of all the
    # %acceptable options, choose that closes to the centroid of the tag list
    # acceptableFreqs        = fCentOptions(acceptableFreqLogic);
    # freqInds               = 1:numel(fCentOptions);
    # acceptableFreqsInds    = freqInds(acceptableFreqLogic);
    # [~,indSel]             = min( abs( acceptableFreqs - mean(tagFreqVecMHzSorted) ) );
    # selectedOptionInd      = acceptableFreqsInds( indSel );

    acceptableFreqs        = fCentOptions[acceptableFreqLogic]

    freqInds               = np.arange(0,np.size(fCentOptions))[:, np.newaxis]

    acceptableFreqsInds    = freqInds[acceptableFreqLogic]

    indSel                 = np.argmin( abs( acceptableFreqs - np.mean(tagFreqVecMHzSorted) ) )

    selectedOptionInd      = acceptableFreqsInds[ indSel ]

    # %Begin calculation of the output parameters
    # radioFc        = fCentOptions(selectedOptionInd);
    # channelFcVec   = radioFc + channelVec;
    # tagChannelFreq = interp1(channelFcVec, channelFcVec, tagFreqVecMHzSorted,...
    #                          'nearest','extrap');
    # %How far off from channel center freq?
    # tagFreqOffestMHz = tagFreqVecMHzSorted - tagChannelFreq;
    # %Determine which channel each tag will be in
    # tagChannelNum = zeros(nTags,1);
    # for i = 1:nTags
    #     tagChannelNum(i) = find(channelFcVec == tagChannelFreq(i));
    # end
    # tagChannelEdgeWarning        = tagFreqOffestMHz > bwFlatFrac*channelBWMHz/2;
    # multipleTagsInChannelWarning = numel(tagChannelNum) ~= numel(unique(tagChannelNum));
    # channelEdgeFreqs = unique([channelFcVec+channelBWHz/2*10^-6;channelFcVec-channelBWHz/2*10^-6]);

    radioFc        = fCentOptions[selectedOptionInd]

    channelFcVec   = radioFc + channelVec
    # Update channelFcVec to align with the channelizer channel order.
    channelFcVec    = np.roll(channelFcVec, int(-np.ceil(nChannels/2)))

    tagChannelFreqFunc = interpolate.interp1d(np.squeeze(channelFcVec), np.squeeze(channelFcVec),  kind = 'nearest', bounds_error = False, fill_value = 'extrapolate')

    tagChannelFreq = tagChannelFreqFunc(tagFreqVecMHzSorted)

    # %How far off from channel center freq?
    tagFreqOffestMHz = tagFreqVecMHzSorted - tagChannelFreq

    # %Determine which channel each tag will be in
    tagChannelNum = np.zeros((nTags,1))

    for i in range(0, nTags):
        tagChannelNum[i] = np.flatnonzero(channelFcVec == tagChannelFreq[i])

    tagChannelEdgeWarning        = np.abs(tagFreqOffestMHz) > channelBWMHz/2-shoulderWidthMHz/2 #bwFlatFrac*channelBWMHz/2

    multipleTagsInChannelWarning = np.size(tagChannelNum) != np.size(np.unique(tagChannelNum))

    fEdgeLower  = np.around(channelFcVec - channelBWMHz/2*10**-6, decimals = 2)

    fEdgeUpper  = np.around(channelFcVec + channelBWMHz/2*10**-6, decimals = 2)

    channelEdgeFreqs = np.unique(np.concatenate((fEdgeLower, fEdgeUpper), axis = 0))[:, np.newaxis]

    #Sort outputs to match input frequency list
    tagChannelEdgeWarning = tagChannelEdgeWarning[inputFreqOrder, 0]
    
    tagChannelNum         = 1 + tagChannelNum[inputFreqOrder, 0]

#     # %Determine the channel center frequecies away from baseband
#     # if mod(nChannels,2)~=0
#     #     channelVec = 1/10^6*(-Fs/nChannels*floor(nChannels/2):Fs/nChannels:Fs/2).';
#     # else
#     #     channelVec = 1/10^6*(-Fs/2: Fs/nChannels: Fs/2-Fs/nChannels).';
#     # end
#     #Determine the channel center frequecies away from baseband
#     freqStep = Fs/nChannels
#     if nChannels%2 != 0:
#         channelVec = 10**-6*np.arange(-freqStep*np.floor(nChannels/2),Fs/2+freqStep,freqStep)[:, np.newaxis]
#     else:
#         channelVec = 10**-6*np.arange(-Fs/2, Fs/2, freqStep)[:, np.newaxis]




    # Note: radioFc will return as a list unless you specify which element you want to return.
    
    return radioFc[0],channelFcVec, tagChannelNum, tagChannelEdgeWarning, multipleTagsInChannelWarning

tuner(300000, 4, np.array([146.0]))

