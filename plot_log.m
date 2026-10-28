%% Plot STM32 ILC trials from concatenated serial-monitor output
% The text file can contain multiple PRNT blocks pasted back-to-back.
% Each block is enclosed by DATA_BEGIN / DATA_END markers.
%
% Firmware log columns:
%   sample,errorX_mm,correctionX_next_mm,errorY_mm,correctionY_next_mm
% DATA_BEGIN now also contains ILC=MI or ILC=PD and active tuning parameters.
%
% Firmware error convention: error = reference - measured position.
% Thus: estimated measured position = reference - error.
%
% The correction columns contain the ILC signal generated for the NEXT trial.
% This script reconstructs the signal actually applied on each trial by using
% the previous same-command block's correctionX_next / correctionY_next.
% The first trial after a command/profile switch is assumed to have u_ILC = 0.
%
% SIST is accepted as a legacy command name, but is always displayed as SINX.
% Supported references: SINX, SINY, CIRC, and PETAL.

clc;
clear;
close all;

[fileName, filePath] = uigetfile( ...
    {'*.txt;*.log;*.csv', 'Serial log files (*.txt, *.log, *.csv)'; ...
     '*.*', 'All files (*.*)'}, ...
    'Select the concatenated STM32 serial log');

if isequal(fileName, 0)
    fprintf('No file selected.\n');
    return;
end

logText = fileread(fullfile(filePath, fileName));
loglines = regexp(logText, '\r\n|\n|\r', 'split');

emptyTrial = struct( ...
    'command', '', ...
    'Fs', NaN, ...
    'R', NaN, ...
    'freqHz', NaN, ...
    'rampS', NaN, ...
    'petalNum', NaN, ...
    'petalTurn', NaN, ...
    'ilcMode', 'UNKNOWN', ...
    'beta', NaN, ...
    'ilcFcHz', NaN, ...
    'kpX', NaN, ...
    'kdX', NaN, ...
    'kpY', NaN, ...
    'kdY', NaN, ...
    'sample', [], ...
    't', [], ...
    'errorX', [], ...
    'correctionXNext', [], ...
    'errorY', [], ...
    'correctionYNext', [], ...
    'xReference', [], ...
    'yReference', [], ...
    'xMeasured', [], ...
    'yMeasured', [], ...
    'ilcAppliedX', [], ...
    'ilcAppliedY', []);
trials = repmat(emptyTrial, 0, 1);

isReadingBlock = false;
currentTrial = emptyTrial;
rowBuffer = [];
rowCount = 0;
expectedRows = 0;

for lineIndex = 1:numel(loglines)
    lineText = strtrim(loglines{lineIndex});

    if strncmp(lineText, 'DATA_BEGIN,', 11)
        % Retain rows from a prior block if the copied text was truncated.
        if isReadingBlock && rowCount > 0
            currentTrial = finishTrial(currentTrial, rowBuffer(1:rowCount, :));
            trials(end + 1, 1) = currentTrial;
        end

        metadata = parseDataBeginHeader(lineText);

        if ~metadata.valid
            warning('Skipping malformed DATA_BEGIN line %d: %s', lineIndex, lineText);
            isReadingBlock = false;
            continue;
        end

        currentTrial = emptyTrial;
        currentTrial.command = canonicalCommand(metadata.command);
        expectedRows = metadata.samples;
        currentTrial.Fs = metadata.Fs;
        currentTrial.R = metadata.R;
        currentTrial.freqHz = metadata.freqHz;
        currentTrial.rampS = metadata.rampS;
        currentTrial.petalNum = metadata.petalNum;
        currentTrial.petalTurn = metadata.petalTurn;
        currentTrial.ilcMode = metadata.ilcMode;
        currentTrial.beta = metadata.beta;
        currentTrial.ilcFcHz = metadata.ilcFcHz;
        currentTrial.kpX = metadata.kpX;
        currentTrial.kdX = metadata.kdX;
        currentTrial.kpY = metadata.kpY;
        currentTrial.kdY = metadata.kdY;
        rowBuffer = NaN(expectedRows, 5);
        rowCount = 0;
        isReadingBlock = true;
        continue;
    end

    if strncmp(lineText, 'DATA_END', 8)
        if isReadingBlock
            if rowCount > 0
                if rowCount ~= expectedRows
                    warning('Block %s contained %d rows; header expected %d.', ...
                        currentTrial.command, rowCount, expectedRows);
                end
                currentTrial = finishTrial(currentTrial, rowBuffer(1:rowCount, :));
                trials(end + 1, 1) = currentTrial;
            else
                warning('Empty data block for command %s.', currentTrial.command);
            end
        end
        isReadingBlock = false;
        currentTrial = emptyTrial;
        rowBuffer = [];
        rowCount = 0;
        expectedRows = 0;
        continue;
    end

    if isReadingBlock && ~isempty(regexp(lineText, '^\d+\s*,', 'once'))
        values = sscanf(lineText, '%f,%f,%f,%f,%f');
        if numel(values) == 5
            if rowCount < size(rowBuffer, 1)
                rowCount = rowCount + 1;
                rowBuffer(rowCount, :) = values(:).';
            else
                warning('Ignoring extra data row in %s at line %d.', ...
                    currentTrial.command, lineIndex);
            end
        end
    end
end

% Accept a final block even if the text ends before DATA_END.
if isReadingBlock && rowCount > 0
    warning('The final data block has no DATA_END marker; using the rows found.');
    currentTrial = finishTrial(currentTrial, rowBuffer(1:rowCount, :));
    trials(end + 1, 1) = currentTrial;
end

if isempty(trials)
    error(['No DATA_BEGIN/DATA_END blocks were found. Use firmware that prints ' ...
           'the marked PRNT blocks, then paste the full dumps into one text file.']);
end

%% Assign zero-based iteration numbers within each consecutive command/configuration sequence
% Iteration 0 is the first (no-ILC) run after the command/reference configuration
% changes. Separate sequences ensure different reference types are not mixed in plots.
numTrials = numel(trials);
Command = cell(numTrials, 1);
TrialNumber = zeros(numTrials, 1);
SequenceID = zeros(numTrials, 1);
SampleCount = zeros(numTrials, 1);
sequenceCounter = 0;

for kTrial = 1:numTrials
    Command{kTrial} = canonicalCommand(trials(kTrial).command);
    trials(kTrial).command = Command{kTrial};
    SampleCount(kTrial) = numel(trials(kTrial).sample);

    sameSequence = false;
    if kTrial > 1
        previous = trials(kTrial - 1);
        current = trials(kTrial);
        sameReference = strcmp(Command{kTrial}, Command{kTrial - 1}) && ...
            isequaln([current.R, current.freqHz, current.rampS, current.Fs, ...
                      current.petalNum, current.petalTurn], ...
                     [previous.R, previous.freqHz, previous.rampS, previous.Fs, ...
                      previous.petalNum, previous.petalTurn]);
        sameILCSettings = strcmp(current.ilcMode, previous.ilcMode) && ...
            isequaln([current.beta, current.ilcFcHz, current.kpX, current.kdX, ...
                      current.kpY, current.kdY], ...
                     [previous.beta, previous.ilcFcHz, previous.kpX, previous.kdX, ...
                      previous.kpY, previous.kdY]);
        sameSequence = sameReference && sameILCSettings;
    end

    if ~sameSequence
        sequenceCounter = sequenceCounter + 1;
        TrialNumber(kTrial) = 0;
    else
        TrialNumber(kTrial) = TrialNumber(kTrial - 1) + 1;
    end
    SequenceID(kTrial) = sequenceCounter;
end

%% Reconstruct the reference and measured output
for kTrial = 1:numTrials
    t = trials(kTrial).t;
    omega = 2*pi*trials(kTrial).freqHz;

    switch trials(kTrial).command
        case 'SINX'
            trials(kTrial).xReference = trials(kTrial).R * sin(omega*t);
            trials(kTrial).yReference = zeros(size(t));

        case 'SINY'
            trials(kTrial).xReference = zeros(size(t));
            trials(kTrial).yReference = trials(kTrial).R * sin(omega*t);

        case 'CIRC'
            % The logged circular segment begins after the approach ramp.
            phase = omega * (trials(kTrial).rampS + t);
            trials(kTrial).xReference = trials(kTrial).R * sin(phase);
            trials(kTrial).yReference = trials(kTrial).R * cos(phase);

        case 'PETAL'
            % Match the STM32 petal reference over the logged learning window.
            % The firmware uses theta = petalTurn*(rampS + t)*petal_freq,
            % with petal_freq = 2*pi*freqHz.
            theta = trials(kTrial).petalTurn .* ...
                (trials(kTrial).rampS + t) .* omega;
            petalEnvelope = cos(trials(kTrial).petalNum .* theta);
            trials(kTrial).xReference = trials(kTrial).R .* petalEnvelope .* cos(theta);
            trials(kTrial).yReference = trials(kTrial).R .* petalEnvelope .* sin(theta);

        otherwise
            warning('Unsupported command %s. Reference will be NaN.', trials(kTrial).command);
            trials(kTrial).xReference = NaN(size(t));
            trials(kTrial).yReference = NaN(size(t));
    end

    % Firmware convention: e = reference - measured position.
    trials(kTrial).xMeasured = trials(kTrial).xReference - trials(kTrial).errorX;
    trials(kTrial).yMeasured = trials(kTrial).yReference - trials(kTrial).errorY;
end

%% Reconstruct the ILC signal applied during each iteration
% PRNT contains the correction profile generated AFTER the corresponding trial.
% Therefore, iteration n uses the correction_next profile printed after iteration n-1.
% Iteration 0 has zero applied ILC. Do not carry corrections across sequence changes.
for kTrial = 1:numTrials
    trials(kTrial).ilcAppliedX = zeros(size(trials(kTrial).sample));
    trials(kTrial).ilcAppliedY = zeros(size(trials(kTrial).sample));

    if kTrial > 1 && SequenceID(kTrial) == SequenceID(kTrial - 1)
        prevX = trials(kTrial - 1).correctionXNext;
        prevY = trials(kTrial - 1).correctionYNext;
        nCopy = min(numel(trials(kTrial).sample), numel(prevX));
        trials(kTrial).ilcAppliedX(1:nCopy) = prevX(1:nCopy);
        nCopy = min(numel(trials(kTrial).sample), numel(prevY));
        trials(kTrial).ilcAppliedY(1:nCopy) = prevY(1:nCopy);
    end
end

%% Calculate summary metrics
RMS_X_Full_mm = NaN(numTrials, 1);
RMS_Y_Full_mm = NaN(numTrials, 1);
RMS_X_Steady_mm = NaN(numTrials, 1);
RMS_Y_Steady_mm = NaN(numTrials, 1);
PeakILC_X_Next_mm = NaN(numTrials, 1);
PeakILC_Y_Next_mm = NaN(numTrials, 1);

for kTrial = 1:numTrials
    RMS_X_Full_mm(kTrial) = sqrt(mean(trials(kTrial).errorX.^2));
    RMS_Y_Full_mm(kTrial) = sqrt(mean(trials(kTrial).errorY.^2));

    % Firmware uses 1-based sample 1000 for the steady-state RMS start.
    % The logged sample index is zero-based, so sample index 999 is included.
    steadyMask = trials(kTrial).sample >= 999;
    if any(steadyMask)
        RMS_X_Steady_mm(kTrial) = sqrt(mean(trials(kTrial).errorX(steadyMask).^2));
        RMS_Y_Steady_mm(kTrial) = sqrt(mean(trials(kTrial).errorY(steadyMask).^2));
    end

    PeakILC_X_Next_mm(kTrial) = max(abs(trials(kTrial).correctionXNext));
    PeakILC_Y_Next_mm(kTrial) = max(abs(trials(kTrial).correctionYNext));
end

%% Print reference and ILC settings as a command-window table (not on figures)
sequenceList = unique(SequenceID, 'stable');
numSequences = numel(sequenceList);
SettingsSequence = zeros(numSequences, 1);
SettingsCommand = cell(numSequences, 1);
ReferenceSignal = cell(numSequences, 1);
R_mm = NaN(numSequences, 1);
Frequency_Hz = NaN(numSequences, 1);
Sampling_Hz = NaN(numSequences, 1);
Ramp_s = NaN(numSequences, 1);
Petal_Num = NaN(numSequences, 1);
Petal_Turn = NaN(numSequences, 1);
ILC_Mode = cell(numSequences, 1);
Beta = NaN(numSequences, 1);
ILC_fc_Hz = NaN(numSequences, 1);
KPX = NaN(numSequences, 1);
KDX = NaN(numSequences, 1);
KPY = NaN(numSequences, 1);
KDY = NaN(numSequences, 1);
LoggedTrials = zeros(numSequences, 1);

for iSeq = 1:numSequences
    seqID = sequenceList(iSeq);
    groupIndices = find(SequenceID == seqID);
    firstIndex = groupIndices(1);
    tr = trials(firstIndex);

    SettingsSequence(iSeq) = iSeq;
    SettingsCommand{iSeq} = tr.command;
    ReferenceSignal{iSeq} = referenceDescriptionForCommand(tr.command);
    R_mm(iSeq) = tr.R;
    Frequency_Hz(iSeq) = tr.freqHz;
    Sampling_Hz(iSeq) = tr.Fs;
    Ramp_s(iSeq) = tr.rampS;
    Petal_Num(iSeq) = tr.petalNum;
    Petal_Turn(iSeq) = tr.petalTurn;
    ILC_Mode{iSeq} = tr.ilcMode;
    Beta(iSeq) = tr.beta;
    ILC_fc_Hz(iSeq) = tr.ilcFcHz;
    KPX(iSeq) = tr.kpX;
    KDX(iSeq) = tr.kdX;
    KPY(iSeq) = tr.kpY;
    KDY(iSeq) = tr.kdY;
    LoggedTrials(iSeq) = numel(groupIndices);
end

settingsTable = table(SettingsSequence, SettingsCommand, ReferenceSignal, ...
    R_mm, Frequency_Hz, Sampling_Hz, Ramp_s, Petal_Num, Petal_Turn, ...
    ILC_Mode, Beta, ILC_fc_Hz, KPX, KDX, KPY, KDY, LoggedTrials, ...
    'VariableNames', {'Sequence','Command','ReferenceSignal','R_mm','Frequency_Hz', ...
    'Sampling_Hz','Ramp_s','Petal_Num','Petal_Turn','ILC_Mode','Beta','ILC_fc_Hz', ...
    'KPX','KDX','KPY','KDY','LoggedTrials'});

fprintf('Reference and ILC settings for each logged sequence:\n');
disp(settingsTable);

fprintf('Reference and ILC settings for each logged sequence:\n\n');
for iSeq = 1:numSequences
    fprintf('--- Sequence %d ---\n', iSeq);
    fprintf('  %-16s : %s\n',  'Command',         SettingsCommand{iSeq});
    fprintf('  %-16s : %s\n',  'ReferenceSignal', ReferenceSignal{iSeq});
    fprintf('  %-16s : %.3f\n','R_mm',            R_mm(iSeq));
    fprintf('  %-16s : %.3f\n','Frequency_Hz',    Frequency_Hz(iSeq));
    fprintf('  %-16s : %.1f\n','Sampling_Hz',     Sampling_Hz(iSeq));
    fprintf('  %-16s : %.3f\n','Ramp_s',          Ramp_s(iSeq));
    fprintf('  %-16s : %.3f\n','Petal_Num',       Petal_Num(iSeq));
    fprintf('  %-16s : %.3f\n','Petal_Turn',      Petal_Turn(iSeq));
    fprintf('  %-16s : %s\n',  'ILC_Mode',        ILC_Mode{iSeq});
    fprintf('  %-16s : %.3f\n','Beta',            Beta(iSeq));
    fprintf('  %-16s : %.3f\n','ILC_fc_Hz',       ILC_fc_Hz(iSeq));
    fprintf('  %-16s : %.4f\n','KPX',             KPX(iSeq));
    fprintf('  %-16s : %.4f\n','KDX',             KDX(iSeq));
    fprintf('  %-16s : %.4f\n','KPY',             KPY(iSeq));
    fprintf('  %-16s : %.4f\n','KDY',             KDY(iSeq));
    fprintf('  %-16s : %d\n',  'LoggedTrials',    LoggedTrials(iSeq));
    fprintf('\n');
end

fprintf(['Iteration 0 is treated as the no-ILC baseline within each sequence. ' ...
         'Settings are shown in the table above, not in plot titles.\n\n']);

summaryTable = table(Command, TrialNumber, SampleCount, ...
    RMS_X_Full_mm, RMS_Y_Full_mm, RMS_X_Steady_mm, RMS_Y_Steady_mm, ...
    PeakILC_X_Next_mm, PeakILC_Y_Next_mm, ...
    'VariableNames', {'Command','Iteration','SampleCount', ...
    'RMS_X_Full_mm','RMS_Y_Full_mm','RMS_X_Steady_mm','RMS_Y_Steady_mm', ...
    'PeakILC_X_Next_mm','PeakILC_Y_Next_mm'});
disp(summaryTable);

%% Generate six all-iteration figures plus three best-iteration figures per sequence
% Reference and ILC settings are printed in settingsTable, not displayed on figures.
% RMS convergence is plotted as lines with markers (not bars).
for sequenceIndex = 1:numel(sequenceList)
    seqID = sequenceList(sequenceIndex);
    groupIndices = find(SequenceID == seqID);
    firstIndex = groupIndices(1);
    commandName = Command{firstIndex};
    nGroup = numel(groupIndices);
    sequenceTitle = commandName;
    clrs = [1 0 0; lines(nGroup-1)];

    %% Figure 1: X and Y positions as subplots
    figure('Name', sprintf('%s positions', commandName, sequenceIndex), ...
        'Color', 'w');
    subplot(2, 1, 1); hold on; grid on;
    plot(trials(firstIndex).t, trials(firstIndex).xReference, 'k--', ...
        'LineWidth', 1.6, 'DisplayName', 'Reference');
    for jj = 1:nGroup
        ii = groupIndices(jj);
        plot(trials(ii).t, trials(ii).xMeasured, 'Color', clrs(jj, :), ...
            'DisplayName', sprintf('Iteration %d', TrialNumber(ii)));
    end
    title('X position'); xlabel('Time (s)'); ylabel('X (mm)');
    legend('Location', 'eastoutside');

    subplot(2, 1, 2); hold on; grid on;
    plot(trials(firstIndex).t, trials(firstIndex).yReference, 'k--', ...
        'LineWidth', 1.6, 'DisplayName', 'Reference');
    for jj = 1:nGroup
        ii = groupIndices(jj);
        plot(trials(ii).t, trials(ii).yMeasured, 'Color', clrs(jj, :), ...
            'DisplayName', sprintf('Iteration %d', TrialNumber(ii)));
    end
    title('Y position'); xlabel('Time (s)'); ylabel('Y (mm)');
    legend('Location', 'eastoutside');

    %% Figure 2: X and Y tracking errors as subplots
    figure('Name', sprintf('%s errors', commandName, sequenceIndex), ...
        'Color', 'w');
    subplot(2, 1, 1); hold on; grid on;
    for jj = 1:nGroup
        ii = groupIndices(jj);
        plot(trials(ii).t, trials(ii).errorX, 'Color', clrs(jj, :), ...
            'DisplayName', sprintf('Iteration %d', TrialNumber(ii)));
    end
    title('X tracking error'); xlabel('Time (s)'); ylabel('e_X (mm)');
    legend('Location', 'eastoutside');

    subplot(2, 1, 2); hold on; grid on;
    for jj = 1:nGroup
        ii = groupIndices(jj);
        plot(trials(ii).t, trials(ii).errorY, 'Color', clrs(jj, :), ...
            'DisplayName', sprintf('Iteration %d', TrialNumber(ii)));
    end
    title('Y tracking error'); xlabel('Time (s)'); ylabel('e_Y (mm)');
    legend('Location', 'eastoutside');

    %% Figure 3: 2D reference and reconstructed trajectories
    figure('Name', sprintf('%s 2D trajectory', commandName, sequenceIndex), ...
        'Color', 'w');
    hold on; grid on; axis equal;
    plot(trials(firstIndex).xReference, trials(firstIndex).yReference, 'k--', ...
        'LineWidth', 1.6, 'DisplayName', 'Reference');
    for jj = 1:nGroup
        ii = groupIndices(jj);
        plot(trials(ii).xMeasured, trials(ii).yMeasured, 'Color', clrs(jj, :), ...
            'DisplayName', sprintf('Iteration %d', TrialNumber(ii)));
    end
    title('2D trajectory'); xlabel('X (mm)'); ylabel('Y (mm)');
    legend('Location', 'eastoutside');

    %% Figure 4: Full-trial RMS convergence (line plot, preserved style)
    figure('Name', sprintf('%s full-trial RMS', commandName, sequenceIndex), ...
        'Color', 'w');
    hold on; grid on;
    plot(TrialNumber(groupIndices), RMS_X_Full_mm(groupIndices), '-o', ...
        'LineWidth', 1.4, 'DisplayName', 'X RMS');
    plot(TrialNumber(groupIndices), RMS_Y_Full_mm(groupIndices), '-s', ...
        'LineWidth', 1.4, 'DisplayName', 'Y RMS');
    title('Full-trial RMS tracking error');
    xlabel('Iteration'); ylabel('RMS error (mm)');
    xticks(TrialNumber(groupIndices));
    legend('Location', 'best');

    %% Figure 5: Steady-state RMS convergence (line plot, preserved style)
    figure('Name', sprintf('%s steady-state RMS', commandName, sequenceIndex), ...
        'Color', 'w');
    hold on; grid on;
    plot(TrialNumber(groupIndices), RMS_X_Steady_mm(groupIndices), '-o', ...
        'LineWidth', 1.4, 'DisplayName', 'X RMS');
    plot(TrialNumber(groupIndices), RMS_Y_Steady_mm(groupIndices), '-s', ...
        'LineWidth', 1.4, 'DisplayName', 'Y RMS');
    title(sprintf('Steady-state RMS tracking error (samples %d-%d)', ...
        ILC_RMS_START_SAMPLE_LOCAL(), max(trials(firstIndex).sample) + 1));
    xlabel('Iteration'); ylabel('RMS error (mm)');
    xticks(TrialNumber(groupIndices));
    legend('Location', 'best');

    %% Figure 6: Applied ILC signal u_ILC for X and Y
    figure('Name', sprintf('%s applied ILC signal', commandName, sequenceIndex), ...
        'Color', 'w');
    subplot(2, 1, 1); hold on; grid on;
    for jj = 1:nGroup
        ii = groupIndices(jj);
        plot(trials(ii).t, trials(ii).ilcAppliedX, 'Color', clrs(jj, :), ...
            'DisplayName', sprintf('Iteration %d', TrialNumber(ii)));
    end
    title('X-axis ILC signal applied to the reference');
    xlabel('Time (s)'); ylabel('u_{ILC,X} (mm)');
    legend('Location', 'eastoutside');

    subplot(2, 1, 2); hold on; grid on;
    for jj = 1:nGroup
        ii = groupIndices(jj);
        plot(trials(ii).t, trials(ii).ilcAppliedY, 'Color', clrs(jj, :), ...
            'DisplayName', sprintf('Iteration %d', TrialNumber(ii)));
    end
    title('Y-axis ILC signal applied to the reference');
    xlabel('Time (s)'); ylabel('u_{ILC,Y} (mm)');
    legend('Location', 'eastoutside');

    %% Select the best iteration using combined steady-state RMS across X and Y.
    groupSteadyCombinedRMS = sqrt((RMS_X_Steady_mm(groupIndices).^2 + ...
                                   RMS_Y_Steady_mm(groupIndices).^2) / 2.0);
    validBestCandidates = find(isfinite(groupSteadyCombinedRMS));
    if isempty(validBestCandidates)
        warning('No finite steady-state RMS values for %s sequence %d; skipping Figures 7-9.', ...
            commandName, sequenceIndex);
    else
        [~, bestLocalCandidate] = min(groupSteadyCombinedRMS(validBestCandidates));
        bestLocalIndex = validBestCandidates(bestLocalCandidate);
        bestIndex = groupIndices(bestLocalIndex);
        bestIteration = TrialNumber(bestIndex);
        baselineIndex = groupIndices(1);  % Iteration 0 in this sequence.

        %% Figure 7: X and Y positions: reference, iteration 0, and best iteration
        figure('Name', sprintf('%s positions - best iteration %d', ...
            commandName, bestIteration, sequenceIndex), 'Color', 'w');

        subplot(2, 1, 1); hold on; grid on;
        plot(trials(firstIndex).t, trials(firstIndex).xReference, 'k--', ...
            'LineWidth', 1.6, 'DisplayName', 'Reference');
        plot(trials(baselineIndex).t, trials(baselineIndex).xMeasured, 'Color', clrs(1, :), ...
            'LineWidth', 1.2, 'DisplayName', 'Iteration 0');
        if bestIndex ~= baselineIndex
            plot(trials(bestIndex).t, trials(bestIndex).xMeasured, 'Color', clrs(2, :), ...
                'LineWidth', 1.2, 'DisplayName', sprintf('Iteration %d (best)', bestIteration));
        else
            % Avoid drawing the same profile twice when iteration 0 is best.
            set(findobj(gca, 'DisplayName', 'Iteration 0'), 'DisplayName', 'Iteration 0 (best)');
        end
        title('X position'); xlabel('Time (s)'); ylabel('X (mm)');
        legend('Location', 'bestoutside');

        subplot(2, 1, 2); hold on; grid on;
        plot(trials(firstIndex).t, trials(firstIndex).yReference, 'k--', ...
            'LineWidth', 1.6, 'DisplayName', 'Reference');
        plot(trials(baselineIndex).t, trials(baselineIndex).yMeasured, 'Color', clrs(1, :), ...
            'LineWidth', 1.2, 'DisplayName', 'Iteration 0');
        if bestIndex ~= baselineIndex
            plot(trials(bestIndex).t, trials(bestIndex).yMeasured, 'Color', clrs(2, :), ...
                'LineWidth', 1.2, 'DisplayName', sprintf('Iteration %d (best)', bestIteration));
        else
            % Do not draw the same iteration twice when iteration 0 is best.
            set(findobj(gca, 'DisplayName', 'Iteration 0'), 'DisplayName', 'Iteration 0 (best)');
        end
        title('Y position'); xlabel('Time (s)'); ylabel('Y (mm)');
        legend('Location', 'bestoutside');

        %% Figure 8: X and Y errors: zero-error reference, iteration 0, and best iteration
        figure('Name', sprintf('%s errors - best iteration %d', ...
            commandName, bestIteration, sequenceIndex), 'Color', 'w');

        subplot(2, 1, 1); hold on; grid on;
        plot(trials(baselineIndex).t, trials(baselineIndex).errorX, 'Color', clrs(1, :), ...
            'LineWidth', 1.2, 'DisplayName', 'Iteration 0');
        if bestIndex ~= baselineIndex
            plot(trials(bestIndex).t, trials(bestIndex).errorX, 'Color', clrs(2, :), ...
                'LineWidth', 1.2, 'DisplayName', sprintf('Iteration %d (best)', bestIteration));
        else
            set(findobj(gca, 'DisplayName', 'Iteration 0'), 'DisplayName', 'Iteration 0 (best)');
        end
        title('X tracking error'); xlabel('Time (s)'); ylabel('e_X (mm)');
        legend('Location', 'bestoutside');

        subplot(2, 1, 2); hold on; grid on;
        plot(trials(baselineIndex).t, trials(baselineIndex).errorY, 'Color', clrs(1, :), ...
            'LineWidth', 1.2, 'DisplayName', 'Iteration 0');
        if bestIndex ~= baselineIndex
            plot(trials(bestIndex).t, trials(bestIndex).errorY, 'Color', clrs(2, :), ...
                'LineWidth', 1.2, 'DisplayName', sprintf('Iteration %d (best)', bestIteration));
        else
            set(findobj(gca, 'DisplayName', 'Iteration 0'), 'DisplayName', 'Iteration 0 (best)');
        end
        title('Y tracking error'); xlabel('Time (s)'); ylabel('e_Y (mm)');
        legend('Location', 'bestoutside');

        %% Figure 9: 2D trajectory: reference, iteration 0, and best iteration
        figure('Name', sprintf('%s 2D trajectory - best iteration %d', ...
            commandName, bestIteration), 'Color', 'w');
        hold on; grid on; axis equal;
        plot(trials(firstIndex).xReference, trials(firstIndex).yReference, 'k--', ...
            'LineWidth', 1.6, 'DisplayName', 'Reference');
        plot(trials(baselineIndex).xMeasured, trials(baselineIndex).yMeasured, 'Color', clrs(1, :), ...
            'LineWidth', 1.2, 'DisplayName', 'Iteration 0');
        if bestIndex ~= baselineIndex
            plot(trials(bestIndex).xMeasured, trials(bestIndex).yMeasured, 'Color', clrs(2, :), ...
                'LineWidth', 1.2, 'DisplayName', sprintf('Iteration %d (best)', bestIteration));
        else
            set(findobj(gca, 'DisplayName', 'Iteration 0'), 'DisplayName', 'Iteration 0 (best)');
        end
        title('2D trajectory'); xlabel('X (mm)'); ylabel('Y (mm)');
        legend('Location', 'best');

        %% Figure 10: 2D trajectory: reference, iteration 0, and best iteration (SS)
        figure('Name', sprintf('%s 2D trajectory (SS) - best iteration %d', ...
            commandName, bestIteration), 'Color', 'w');
        hold on; grid on; axis equal;
        plot(trials(firstIndex).xReference(1000:5000), trials(firstIndex).yReference(1000:5000), 'k--', ...
            'LineWidth', 1.6, 'DisplayName', 'Reference');
        plot(trials(baselineIndex).xMeasured(1000:5000), trials(baselineIndex).yMeasured(1000:5000), 'Color', clrs(1, :), ...
            'LineWidth', 1.2, 'DisplayName', 'Iteration 0');
        if bestIndex ~= baselineIndex
            plot(trials(bestIndex).xMeasured(1000:5000), trials(bestIndex).yMeasured(1000:5000), 'Color', clrs(2, :), ...
                'LineWidth', 1.2, 'DisplayName', sprintf('Iteration %d (best)', bestIteration));
        else
            set(findobj(gca, 'DisplayName', 'Iteration 0'), 'DisplayName', 'Iteration 0 (best)');
        end
        title('2D trajectory steady state'); xlabel('X (mm)'); ylabel('Y (mm)');
        legend('Location', 'best');

    end
end

%% Local helper functions
function outTrial = finishTrial(inTrial, rows)
    outTrial = inTrial;
    outTrial.sample = rows(:, 1);
    outTrial.errorX = rows(:, 2);
    outTrial.correctionXNext = rows(:, 3);
    outTrial.errorY = rows(:, 4);
    outTrial.correctionYNext = rows(:, 5);
    outTrial.t = outTrial.sample ./ outTrial.Fs;
end

function commandName = canonicalCommand(rawCommand)
    commandName = upper(strtrim(rawCommand));
    if strcmp(commandName, 'SIST')
        commandName = 'SINX';
    elseif strcmp(commandName, 'PETL')
        commandName = 'PETAL';
    end
end

function metadata = parseDataBeginHeader(lineText)
    % Parse key=value fields independent of their order. Supports both new
    % headers (with ILC mode/settings) and earlier logs without those fields.
    metadata = struct( ...
        'valid', false, ...
        'command', '', ...
        'samples', NaN, ...
        'Fs', NaN, ...
        'R', NaN, ...
        'freqHz', NaN, ...
        'rampS', NaN, ...
        'petalNum', NaN, ...
        'petalTurn', NaN, ...
        'ilcMode', 'UNKNOWN', ...
        'beta', NaN, ...
        'ilcFcHz', NaN, ...
        'kpX', NaN, ...
        'kdX', NaN, ...
        'kpY', NaN, ...
        'kdY', NaN);

    pairs = regexp(lineText, '([A-Za-z][A-Za-z0-9_]*)=([^,]*)', 'tokens');
    for iPair = 1:numel(pairs)
        key = lower(strtrim(pairs{iPair}{1}));
        value = strtrim(pairs{iPair}{2});
        switch key
            case 'command'
                metadata.command = value;
            case 'samples'
                metadata.samples = str2double(value);
            case 'fs_hz'
                metadata.Fs = str2double(value);
            case 'r_mm'
                metadata.R = str2double(value);
            case 'freq_hz'
                metadata.freqHz = str2double(value);
            case 'ramp_s'
                metadata.rampS = str2double(value);
            case 'petal_num'
                metadata.petalNum = str2double(value);
            case 'petal_turn'
                metadata.petalTurn = str2double(value);
            case 'ilc'
                metadata.ilcMode = upper(value);
            case 'beta'
                metadata.beta = str2double(value);
            case 'fc_hz'
                metadata.ilcFcHz = str2double(value);
            case 'kpx'
                metadata.kpX = str2double(value);
            case 'kdx'
                metadata.kdX = str2double(value);
            case 'kpy'
                metadata.kpY = str2double(value);
            case 'kdy'
                metadata.kdY = str2double(value);
        end
    end

    metadata.valid = ~isempty(metadata.command) && ...
        isfinite(metadata.samples) && metadata.samples >= 1 && ...
        isfinite(metadata.Fs) && metadata.Fs > 0 && ...
        isfinite(metadata.R) && isfinite(metadata.freqHz) && ...
        isfinite(metadata.rampS);
end

function description = referenceDescriptionForCommand(commandName)
    switch commandName
        case 'SINX'
            description = 'X: R*sin(2*pi*f*t); Y: 0';
        case 'SINY'
            description = 'X: 0; Y: R*sin(2*pi*f*t)';
        case 'CIRC'
            description = 'X: R*sin(2*pi*f*(t+ramp)); Y: R*cos(2*pi*f*(t+ramp))';
        case 'PETAL'
            description = ['theta=turn*2*pi*f*(t+ramp); X=R*cos(num*theta)*cos(theta); ' ...
                           'Y=R*cos(num*theta)*sin(theta)'];
        otherwise
            description = 'Reference equation unavailable';
    end
end

function startSample = ILC_RMS_START_SAMPLE_LOCAL()
    % Mirror the firmware's current 1-based start sample (1000 by default).
    startSample = 1000;
end
