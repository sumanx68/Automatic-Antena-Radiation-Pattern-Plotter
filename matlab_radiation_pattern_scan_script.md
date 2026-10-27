function PNA_arduino_scan()
% Group-12 automatic radiation pattern scan
%   Arduino (az/el, your sketch)  +  Keysight PNA-L N5232B over LAN (SCPI, port 5025)
%
% Based on sir's PNA_mandi.m (tcpip + CALC:DATA? FDATA + binblockread), but:
%   - the Arduino port stays open (no open/close every step)
%   - MATLAB tells the Arduino "go AZ EL", waits until the move is finished,
%     waits a settle time, triggers ONE fresh PNA sweep and stores it
%   - after the full 360 deg azimuth scan it saves CSV/MAT and draws the 3D pattern
%
% Uses the same legacy functions as sir's code (serial, tcpip, binblockread),
% so it needs the Instrument Control Toolbox. Run it from the Editor or type PNA_arduino_scan.
%
% Arduino sketch: set AUTO_RUN 0 and Serial 115200 (as in your sketch).

%% ======================= USER SETTINGS =======================
% --- Arduino link ---
cfg.arduinoLink    = 'serial';        % 'serial' = USB on this laptop | 'tcp' = serial_tcp_bridge.py
cfg.comPort        = 'COM_PORT_PLACEHOLDER'; % Arduino port (check Device Manager, e.g., 'COM3')
cfg.baud           = 115200;          % must match Serial.begin in the sketch
cfg.arduinoIP      = '192.168.X.X';   % only for 'tcp'
cfg.arduinoTcpPort = 2000;            % only for 'tcp'

% --- PNA link (from sir's code) ---
cfg.pnaIP          = 'XXX.XXX.XXX.XXX'; % TODO: Enter your PNA IP address here
cfg.pnaPort        = 5025;
cfg.pnaMeas        = 'CH1_S11_1';     % measurement name on the PNA. For a radiation pattern between
                                      % two antennas create an S21 trace and use its name, e.g. 'CH1_S21_1'

% --- what to measure ---
cfg.mode           = 'scan';          % 'scan' (az x el) | 'azcut' | 'elcut'
cfg.azStep         = 5;               % deg
cfg.azMax          = 360;             % az goes 0 .. azMax-azStep
cfg.elStep         = 5;               % deg
cfg.elMin          = 0;
cfg.elMax          = 180;
cfg.fixedEL        = 90;              % used by 'azcut'
cfg.fixedAZ        = 0;               % used by 'elcut'
cfg.patternFreqHz  = [];              % frequency to plot, e.g. 2.4e9. [] = centre of the PNA sweep

cfg.settleS        = 0.3;             % wait after each move before the PNA sweep (s)
cfg.floorDB        = -40;             % lowest level shown in the plots
cfg.echoArduino    = false;           % true = print every line the Arduino sends
cfg.loadFile       = '';              % e.g. 'pna_scan_20261025_101500.mat' = only re-plot saved data
%% =============================================================

% ---- re-plot only ----
if ~isempty(cfg.loadFile)
    S = load(cfg.loadFile);
    fIdx = pickFreq(S.freq, cfg.patternFreqHz);
    plotPattern(S.A, S.azL, S.elL, S.freq, fIdx, cfg);
    return;
end

% ---- connect ----
try delete(instrfindall); catch, end               %#ok<*CTCH>   same as sir's code
ard = openArduino(cfg);
try
    PNA = openPNA(cfg);
catch err
    closeAll(ard, []);
    rethrow(err);
end
cleaner = onCleanup(@() closeAll(ard, PNA)); %#ok<NASGU>

freq = getFreqAxis(PNA);
nF   = numel(freq);
fIdx = pickFreq(freq, cfg.patternFreqHz);
fprintf('PNA sweep: %d points, %.4f - %.4f GHz, plotting %.4f GHz\n', ...
        nF, freq(1)/1e9, freq(end)/1e9, freq(fIdx)/1e9);

% ---- list of angles ----
azList = 0:cfg.azStep:(cfg.azMax - cfg.azStep);
elList = cfg.elMin:cfg.elStep:cfg.elMax;
switch cfg.mode
    case 'scan'
        pts = zeros(0, 2);
        for ia = 1:numel(azList)
            if mod(ia, 2) == 1, els = elList; else, els = fliplr(elList); end   % serpentine
            pts = [pts; repmat(azList(ia), numel(els), 1), els(:)];             %#ok<AGROW>
        end
    case 'azcut'
        pts = [azList(:), repmat(cfg.fixedEL, numel(azList), 1)];
    case 'elcut'
        pts = [repmat(cfg.fixedAZ, numel(elList), 1), elList(:)];
    otherwise
        error('Unknown cfg.mode: %s', cfg.mode);
end
nPts = size(pts, 1);
fprintf('%d measurement points.\n', nPts);

% ---- start pose ----
input('Put the antenna at the START pose (AZ=0, EL=0) and press Enter... ', 's');
fprintf(ard, 'zero');
waitForLine(ard, {'[ OK ]', 'OK zero'}, 5, [], cfg.echoArduino);

% ---- live figure ----
fig = figure('Name', 'PNA radiation scan (live)', 'NumberTitle', 'off');
setappdata(fig, 'abort', false);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'ABORT', 'FontWeight', 'bold', ...
          'BackgroundColor', [1 0.45 0.45], 'Position', [10 10 90 30], ...
          'Callback', @(~, ~) setappdata(fig, 'abort', true));
ax = axes('Parent', fig, 'Position', [0.12 0.2 0.72 0.7]);
hold(ax, 'on'); grid(ax, 'on');
hs = scatter(ax, 0, 0, 50, 0, 'filled');
xlabel(ax, 'Azimuth (deg)'); ylabel(ax, 'Elevation (deg)');
xlim(ax, [-5 365]); ylim(ax, [-5 185]); colorbar(ax);

% ---- scan ----
A    = nan(nF, nPts);      % one PNA sweep per column (like sir's A matrix)
vals = nan(nPts, 1);
tStart = tic;
k = 0;
for p = 1:nPts
    if ~ishandle(fig) || getappdata(fig, 'abort')
        fprintf('[MATLAB] aborted by user at point %d/%d\n', p, nPts);
        break;
    end
    az = pts(p, 1);  el = pts(p, 2);

    moveTo(ard, az, el, fig, cfg.echoArduino);          % blocks until the Arduino prints [POS ]
    if ~ishandle(fig) || getappdata(fig, 'abort'), fprintf('[MATLAB] aborted\n'); break; end
    pause(cfg.settleS);

    %%% START: data recording from the PNA after each angular step
    y = readPNA(PNA);
    %%% END
    if numel(y) ~= nF
        warning('PNA returned %d points, expected %d', numel(y), nF);
        y = y(1:min(end, nF));  y(end+1:nF) = NaN;
    end
    k = p;
    A(:, p)  = y;
    vals(p)  = y(fIdx);

    fprintf('[%4d/%d] AZ %6.1f  EL %6.1f  %8.2f  (%.0f s)\n', p, nPts, az, el, vals(p), toc(tStart));
    set(hs, 'XData', pts(1:p, 1), 'YData', pts(1:p, 2), 'CData', vals(1:p));
    title(ax, sprintf('%.4f GHz   point %d / %d', freq(fIdx)/1e9, p, nPts));
    drawnow limitrate;
end

% ---- back to start pose ----
try
    moveTo(ard, 0, 0, [], cfg.echoArduino);
catch
    fprintf('[MATLAB] could not return to 0,0\n');
end

if k == 0
    fprintf('No data recorded.\n');
    return;
end
A = A(:, 1:k);
azL = pts(1:k, 1);
elL = pts(1:k, 2);

% ---- save ----
base = ['pna_scan_' datestr(now, 'yyyymmdd_HHMMSS')];
save([base '.mat'], 'A', 'azL', 'elL', 'freq', 'cfg');
if exist('writematrix', 'file'), writematrix(A, [base '_A.csv']); else, csvwrite([base '_A.csv'], A); end
fid = fopen([base '_pattern.csv'], 'w');
fprintf(fid, 'az_deg,el_deg,value_at_%.6fGHz\n', freq(fIdx)/1e9);
fprintf(fid, '%g,%g,%g\n', [azL, elL, A(fIdx, :).'].');
fclose(fid);
fprintf('\nSaved %s.mat, %s_A.csv (rows = frequency, columns = angle points) and %s_pattern.csv\n', base, base, base);

% ---- plot ----
plotPattern(A, azL, elL, freq, fIdx, cfg);
end

%% ======================= helper functions =======================
function ard = openArduino(cfg)
if strcmp(cfg.arduinoLink, 'tcp')
    ard = tcpip(cfg.arduinoIP, cfg.arduinoTcpPort);
    set(ard, 'Terminator', 'LF', 'Timeout', 5);
    fopen(ard);
    pause(0.5);                        % a network bridge does not reset the Uno
else
    ard = serial(cfg.comPort, 'BaudRate', cfg.baud);
    set(ard, 'Terminator', 'LF', 'Timeout', 5);
    fopen(ard);
    pause(2.2);                        % opening the port resets the Uno
end
fprintf(ard, 'x');  pause(0.4);        % cancel the sketch's 5 s auto-run if it started
fprintf(ard, 'x');  pause(0.4);
flushinput(ard);
fprintf('Arduino connected (%s)\n', cfg.arduinoLink);
end

function PNA = openPNA(cfg)
PNA = tcpip(cfg.pnaIP, cfg.pnaPort);
set(PNA, 'InputBufferSize', 200000);
set(PNA, 'Timeout', 30);
set(PNA, 'ByteOrder', 'littleEndian');
fopen(PNA);
fprintf(PNA, '*IDN?');         fprintf('PNA: %s', fscanf(PNA));
fprintf(PNA, 'CALC1:PAR:CAT?');  fprintf('Measurements on PNA: %s', fscanf(PNA));
fprintf(PNA, ['CALC1:PAR:SEL "' cfg.pnaMeas '"']);
fprintf(PNA, 'FORM REAL,64');
fprintf(PNA, 'FORM:BORD SWAP');
fprintf(PNA, 'INIT1:CONT OFF');     % one fresh sweep per angle; restored to ON at the end
end

function freq = getFreqAxis(PNA)
fprintf(PNA, 'SENS1:FREQ:STAR?'); f1 = str2double(fscanf(PNA));
fprintf(PNA, 'SENS1:FREQ:STOP?'); f2 = str2double(fscanf(PNA));
fprintf(PNA, 'SENS1:SWE:POIN?');  np = round(str2double(fscanf(PNA)));
freq = linspace(f1, f2, np).';         % assumes a linear frequency sweep
end

function y = readPNA(PNA)
flushinput(PNA);
fprintf(PNA, 'INIT1:IMM;*OPC?');         % take one new sweep ...
fscanf(PNA);                            % ... and wait until it is complete
fprintf(PNA, 'CALC1:DATA? FDATA');        % same read as sir's code
y = binblockread(PNA, 'double');
fread(PNA, 1);                          % trailing newline
y = y(:);
end

function moveTo(ard, az, el, fig, echo)
fprintf(ard, 'go %g %g', az, el);
waitForLine(ard, {'[POS ]', 'POS,'}, 120, fig, echo);   % printed by the sketch when the move is done
end

function ln = waitForLine(ard, pats, tmo, fig, echo)
t0 = tic;
sentX = false;
while toc(t0) < tmo
    if ~isempty(fig) && ~sentX && (~ishandle(fig) || getappdata(fig, 'abort'))
        fprintf(ard, 'x');  sentX = true;          % stop the motors, keep waiting for [POS ]
    end
    if ard.BytesAvailable > 0
        s = fgetl(ard);
        if ischar(s)
            s = strtrim(s);
            if echo && ~isempty(s), fprintf('    [Arduino] %s\n', s); end
            if ~isempty(strfind(s, '[ERR')) || strncmp(s, 'ERR', 3)
                error('Arduino error: %s', s);
            end
            for q = 1:numel(pats)
                if ~isempty(strfind(s, pats{q})), ln = s; return; end
            end
        end
    else
        pause(0.005);
    end
end
error('Timeout waiting for the Arduino (%s). Check COM port, baud rate and that the sketch is running.', pats{1});
end

function closeAll(ard, PNA)
try, if strcmp(ard.Status, 'open'), fprintf(ard, 'x'); end, catch, end      % stop motors
try, if strcmp(PNA.Status, 'open'), fprintf(PNA, 'INIT1:CONT ON'); end, catch, end
try, fclose(ard);  delete(ard);  catch, end
try, fclose(PNA);  delete(PNA);  catch, end
fprintf('[MATLAB] connections closed\n');
end

function idx = pickFreq(freq, target)
if isempty(target)
    idx = round(numel(freq) / 2);
else
    [~, idx] = min(abs(freq - target));
end
end

function plotPattern(A, azL, elL, freq, fIdx, cfg)
P = A(fIdx, :).';                       % value at the chosen frequency (dB, as formatted on the PNA)
bad = isnan(P);
P = P - max(P);                         % normalise: peak = 0 dB
P = max(P, cfg.floorDB);
P(bad) = NaN;
[~, ipk] = max(P);
fprintf('Peak at AZ = %.1f deg, EL = %.1f deg  (%.4f GHz)\n', azL(ipk), elL(ipk), freq(fIdx)/1e9);
ttl = sprintf('%.4f GHz', freq(fIdx)/1e9);

azU = unique(azL);  elU = unique(elL);

if numel(elU) == 1                                   % azimuth cut
    [a, i] = sort(azL);  d = P(i);
    a = [a; a(1) + 360];  d = [d; d(1)];
    figure('Name', 'Azimuth cut', 'NumberTitle', 'off');
    polarplot(deg2rad(a), d, 'LineWidth', 1.6);
    rlim([cfg.floorDB 0]);  title(['Azimuth cut at EL = ' num2str(elU) ' deg, ' ttl]);
    return;
elseif numel(azU) == 1                                   % elevation cut
    [e, i] = sort(elL);
    figure('Name', 'Elevation cut', 'NumberTitle', 'off');
    polarplot(deg2rad(e), P(i), 'LineWidth', 1.6);
    thetalim([0 180]);  rlim([cfg.floorDB 0]);
    title(['Elevation cut at AZ = ' num2str(azU) ' deg, ' ttl]);
    return;
end

% ---- full scan: grid ----
[~, ~, ia] = unique(azL);
[~, ~, ie] = unique(elL);
M = nan(numel(elU), numel(azU));
M(sub2ind(size(M), ie, ia)) = P;
azG = azU;
if azG(1) == 0 && azG(end) < 360                     % close the sphere
    azG = [azG; 360];  M = [M, M(:, 1)];
end
[AZ, EL] = meshgrid(azG, elU);

% 3D pattern (radius = normalised level, EL = angle from the z axis)
R = (M - cfg.floorDB) / (-cfg.floorDB);
X = R .* sind(EL) .* cosd(AZ);
Y = R .* sind(EL) .* sind(AZ);
Z = R .* cosd(EL);
f1 = figure('Name', '3D radiation pattern', 'NumberTitle', 'off');
surf(X, Y, Z, M);  shading interp;  axis equal;  colorbar;  caxis([cfg.floorDB 0]);
xlabel('X'); ylabel('Y'); zlabel('Z (EL = 0)');  view(135, 25);
title(['3D radiation pattern, dB, ' ttl]);

% AZ-EL map
f2 = figure('Name', 'AZ-EL map', 'NumberTitle', 'off');
imagesc(azG, elU, M);  axis xy;  colorbar;  caxis([cfg.floorDB 0]);
xlabel('Azimuth (deg)');  ylabel('Elevation (deg)');  title(['AZ-EL map, dB, ' ttl]);

% cuts through the peak
iEl = find(elL == elL(ipk));
[a, i] = sort(azL(iEl));  d = P(iEl(i));
f3 = figure('Name', 'Azimuth cut at peak', 'NumberTitle', 'off');
polarplot(deg2rad([a; a(1) + 360]), [d; d(1)], 'LineWidth', 1.6);
rlim([cfg.floorDB 0]);  title(sprintf('Azimuth cut at EL = %g deg, %s', elL(ipk), ttl));

iAz = find(azL == azL(ipk));
[e, i] = sort(elL(iAz));
f4 = figure('Name', 'Elevation cut at peak', 'NumberTitle', 'off');
polarplot(deg2rad(e), P(iAz(i)), 'LineWidth', 1.6);
thetalim([0 180]);  rlim([cfg.floorDB 0]);
title(sprintf('Elevation cut at AZ = %g deg, %s', azL(ipk), ttl));

try
    base = ['pna_scan_' datestr(now, 'yyyymmdd_HHMMSS')];
    saveas(f1, [base '_3D.png']);  saveas(f2, [base '_map.png']);
    saveas(f3, [base '_azcut.png']);  saveas(f4, [base '_elcut.png']);
catch
end
end