%% safe_tune_controller.m
%  Automated position-planner tuning sweep with safety time limit and resume.
%
%  Sweeps K_TRACK (pixel-error → metre gain) and K_APEX (velocity look-ahead
%  for apex cutting) inside mpc_racing_controller.m, runs a short simulation
%  for each combo, and logs cross-track RMSE results to tuning_history.csv.
%
%  USAGE
%    1. Open parrotMinidroneCompetition.prj in MATLAB.
%    2. Ensure flightControlSystem.slx is correctly wired (MPC block in
%       Path Planning is a POSITION planner; controlModePosVsOrient = 1).
%    3. Run:  safe_tune_controller
%
%  RESUME
%    If interrupted, simply re-run — already-tested combos are skipped.
%
%  OUTPUTS
%    tuning_history.csv  – one row per combination (K_TRACK, K_APEX, RMSE, ...)
%    error_log.txt       – any simulation failures with full error message

%% ── CONFIGURATION ────────────────────────────────────────────────────────
K_TRACK_values     = [0.0025, 0.0038, 0.0060];   % pixel→metre gain (0.0038 = orig)
K_APEX_values      = [0.00, 0.05, 0.10, 0.15];    % apex/velocity preview gain [s]
TIME_LIMIT_MINUTES = 50;                          % hard cutoff (< 60 min)
SIM_STOP_TIME      = '20';                        % seconds per simulation run
TOP_MODEL          = 'parrotMinidroneCompetition';

scriptDir       = fileparts(mfilename('fullpath'));
CONTROLLER_FILE = fullfile(scriptDir, 'controller', 'mpc_racing_controller.m');
CSV_FILE        = fullfile(scriptDir, 'tuning_history.csv');
ERROR_LOG       = fullfile(scriptDir, 'error_log.txt');

%% ── BUILD FULL PARAMETER GRID ────────────────────────────────────────────
[Kt_grid, Ka_grid] = meshgrid(K_TRACK_values, K_APEX_values);
Kt_list = Kt_grid(:);
Ka_list = Ka_grid(:);
total   = numel(Kt_list);

%% ── VALIDATE CONTROLLER FILE EXISTS ─────────────────────────────────────
if ~isfile(CONTROLLER_FILE)
    error('safe_tune:noFile', ...
          'Controller not found:\n  %s\nCheck scriptDir is correct.', ...
          CONTROLLER_FILE);
end

%% ── RESUME: LOAD ALREADY-TESTED COMBINATIONS ─────────────────────────────
tested = containers.Map('KeyType', 'char', 'ValueType', 'logical');

if isfile(CSV_FILE)
    fprintf('Found %s — loading previously tested combinations...\n', CSV_FILE);
    try
        prev = readtable(CSV_FILE, 'Delimiter', ',');
        for i = 1:height(prev)
            tested(st_makeKey(prev.K_TRACK(i), prev.K_APEX(i))) = true;
        end
        fprintf('  Skipping %d already-tested combination(s).\n\n', tested.Count);
    catch ME
        fprintf('  Warning: could not parse CSV (%s). Starting fresh.\n\n', ME.message);
    end
else
    % Create CSV with header row
    fid = fopen(CSV_FILE, 'w');
    if fid == -1
        error('safe_tune:csvCreate', 'Cannot create %s — check write permissions.', CSV_FILE);
    end
    fprintf(fid, 'K_TRACK,K_APEX,RMSE_x,RMSE_y,RMSE_total,sim_time_s,status\n');
    fclose(fid);
    fprintf('Created results file: %s\n\n', CSV_FILE);
end

%% ── BACKUP ORIGINAL CONTROLLER CODE ─────────────────────────────────────
originalCode = fileread(CONTROLLER_FILE);

%% ── INITIALISE SIMULATION WORKSPACE ─────────────────────────────────────
fprintf('Initialising workspace...\n');
try
    % Override visualisation to "workspace" mode for speed (no 3-D window)
    assignin('base', 'VSS_VISUALIZATION', 1);
    startVars;
    assignin('base', 'VSS_VISUALIZATION', 1);   % override in case startVars reset it
    fprintf('  startVars OK\n');
catch ME
    fprintf('  Warning: startVars failed (%s). Using existing workspace.\n', ME.message);
end

% Load Simulink model without opening the GUI window
try
    if ~bdIsLoaded(TOP_MODEL)
        load_system(TOP_MODEL);
        fprintf('  Loaded model: %s\n\n', TOP_MODEL);
    else
        fprintf('  Model already loaded: %s\n\n', TOP_MODEL);
    end
catch ME
    error('safe_tune:noModel', 'Cannot load "%s": %s', TOP_MODEL, ME.message);
end

%% ── MAIN TUNING LOOP ─────────────────────────────────────────────────────
startTime = datetime('now');
completed = 0;
skipped   = 0;

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  POSITION-PLANNER TUNING SWEEP  —  %d combinations total\n', total);
fprintf('  Time limit : %d min  |  Sim duration : %s s each\n', ...
        TIME_LIMIT_MINUTES, SIM_STOP_TIME);
fprintf('═══════════════════════════════════════════════════════════\n\n');

for idx = 1:total

    %% ── Time-limit guard ──────────────────────────────────────────────────
    elapsed = minutes(datetime('now') - startTime);
    if elapsed > TIME_LIMIT_MINUTES
        fprintf('\n[TIMEOUT]  %.1f min elapsed — stopping after %d/%d combinations.\n', ...
                elapsed, completed, total);
        break;
    end

    K_track = Kt_list(idx);
    K_apex  = Ka_list(idx);
    key     = st_makeKey(K_track, K_apex);

    %% ── Skip already-tested ───────────────────────────────────────────────
    if isKey(tested, key)
        skipped = skipped + 1;
        continue;
    end

    elapsed = minutes(datetime('now') - startTime);
    fprintf('[%2d/%d]  K_TRACK=%-8g  K_APEX=%-5g  (%.1f min elapsed)\n', ...
            idx, total, K_track, K_apex, elapsed);

    %% ── Patch K_TRACK and K_APEX in controller file ──────────────────────
    patchedCode = regexprep(originalCode, ...
        'K_TRACK\s*=\s*[\d\.eE+\-]+\s*;', sprintf('K_TRACK = %g;', K_track));
    patchedCode = regexprep(patchedCode, ...
        'K_APEX\s*=\s*[\d\.eE+\-]+\s*;',  sprintf('K_APEX  = %g;', K_apex));

    fid = fopen(CONTROLLER_FILE, 'w');
    if fid == -1
        st_logError(ERROR_LOG, K_track, K_apex, 'Cannot open controller file for writing');
        fprintf('  → SKIPPED (file write error)\n\n');
        continue;
    end
    fwrite(fid, patchedCode, 'char');
    fclose(fid);

    %% ── Force MATLAB to reload the patched controller ────────────────────
    clear mpc_racing_controller

    %% ── Run simulation ────────────────────────────────────────────────────
    rmse_x     = NaN;
    rmse_y     = NaN;
    rmse_total = NaN;
    status     = 'OK';
    simStart   = datetime('now');

    try
        simOut = sim(TOP_MODEL, ...
                     'StopTime',        SIM_STOP_TIME, ...
                     'SimulationMode',  'normal');
        sim_time_s = seconds(datetime('now') - simStart);

        %% ── Extract cross-track RMSE from logged signals ─────────────────
        % Attempt 1: logsout structure (VSS_VISUALIZATION = 1)
        try
            logs = simOut.get('logsout');
            if ~isempty(logs)
                try
                    sig_x  = logs.getElement('x_error');
                    rmse_x = rms(sig_x.Values.Data(:));
                catch
                    % signal not in logsout — try workspace next
                end
                try
                    sig_y  = logs.getElement('y_error');
                    rmse_y = rms(sig_y.Values.Data(:));
                catch
                    % signal not in logsout — try workspace next
                end
            end
        catch
            % logsout not available in this simOut
        end

        % Attempt 2: base workspace variables logged by model
        if isnan(rmse_x)
            try
                xe     = evalin('base', 'x_error');
                rmse_x = rms(xe(:));
            catch
                % not in workspace either
            end
        end
        if isnan(rmse_y)
            try
                ye     = evalin('base', 'y_error');
                rmse_y = rms(ye(:));
            catch
                % not in workspace either
            end
        end

        if ~isnan(rmse_x) && ~isnan(rmse_y)
            rmse_total = sqrt(rmse_x^2 + rmse_y^2);
            fprintf('  → RMSE  x=%.4f  y=%.4f  total=%.4f  [%.0f s]\n', ...
                    rmse_x, rmse_y, rmse_total, sim_time_s);
        else
            fprintf('  → Sim OK  [%.0f s]  (no error signal found in output)\n', ...
                    sim_time_s);
        end

    catch ME
        sim_time_s = seconds(datetime('now') - simStart);
        status     = 'ERROR';
        errMsg     = strtrim(ME.message);
        if numel(errMsg) > 120
            errMsg = [errMsg(1:120) '...'];
        end
        fprintf('  → FAILED [%.0f s]: %s\n', sim_time_s, errMsg);
        st_logError(ERROR_LOG, K_track, K_apex, ME.message);
    end

    %% ── Restore original controller (always, even after errors) ──────────
    fid = fopen(CONTROLLER_FILE, 'w');
    fwrite(fid, originalCode, 'char');
    fclose(fid);
    clear mpc_racing_controller

    %% ── Append result to CSV ─────────────────────────────────────────────
    fid = fopen(CSV_FILE, 'a');
    fprintf(fid, '%g,%g,%.6f,%.6f,%.6f,%.2f,%s\n', ...
            K_track, K_apex, rmse_x, rmse_y, rmse_total, sim_time_s, status);
    fclose(fid);

    tested(key) = true;
    completed   = completed + 1;
    fprintf('  [Completed %d/%d | Elapsed %.1f min]\n\n', ...
            completed, total, minutes(datetime('now') - startTime));
end

%% ── SAFETY NET: Restore controller after any loop exit ───────────────────
fid = fopen(CONTROLLER_FILE, 'w');
fwrite(fid, originalCode, 'char');
fclose(fid);
clear mpc_racing_controller

%% ── FINAL SUMMARY ────────────────────────────────────────────────────────
totalElapsed = minutes(datetime('now') - startTime);
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('  SWEEP COMPLETE\n');
fprintf('  Tested this run  : %d\n', completed);
fprintf('  Skipped (done)   : %d\n', skipped);
fprintf('  Total elapsed    : %.1f minutes\n', totalElapsed);
fprintf('  Results file     : %s\n', CSV_FILE);
fprintf('═══════════════════════════════════════════════════════════\n');

%% ── FIND AND PRINT BEST PARAMETERS ──────────────────────────────────────
try
    results = readtable(CSV_FILE, 'Delimiter', ',');
    ok_mask = strcmp(results.status, 'OK') & ~isnan(results.RMSE_total);
    if any(ok_mask)
        ok_results       = results(ok_mask, :);
        [~, best_idx]    = min(ok_results.RMSE_total);
        best             = ok_results(best_idx, :);
        fprintf('\n  BEST PARAMETERS (lowest RMSE_total):\n');
        fprintf('    K_TRACK    = %g\n',   best.K_TRACK);
        fprintf('    K_APEX     = %g\n',   best.K_APEX);
        fprintf('    RMSE_x     = %.4f\n', best.RMSE_x);
        fprintf('    RMSE_y     = %.4f\n', best.RMSE_y);
        fprintf('    RMSE_total = %.4f\n', best.RMSE_total);
        fprintf('\n  Set these in mpc_racing_controller.m (K_TRACK, K_APEX),\n');
        fprintf('  then run the full simulation to validate.\n\n');
    else
        fprintf('\n  No successful simulations with RMSE data.\n');
        fprintf('  Check %s and fix wiring issues first.\n\n', ERROR_LOG);
    end
catch ME
    fprintf('\n  Could not read results: %s\n\n', ME.message);
end

%% ══════════════════════════════════════════════════════════════════════════
%  LOCAL HELPER FUNCTIONS
%  (defined at file scope — visible only within this script)
%% ══════════════════════════════════════════════════════════════════════════

function key = st_makeKey(K_track, K_apex)
%ST_MAKEKEY  Unique string key for a (K_TRACK, K_APEX) parameter pair.
    key = sprintf('Kt%.6f_Ka%.6f', K_track, K_apex);
end

function st_logError(errorFile, K_track, K_apex, msg)
%ST_LOGERROR  Append one error entry to the error log file.
    fid = fopen(errorFile, 'a');
    if fid ~= -1
        fprintf(fid, '[%s]  K_TRACK=%g  K_APEX=%g\n  %s\n\n', ...
                char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), K_track, K_apex, msg);
        fclose(fid);
    end
end
