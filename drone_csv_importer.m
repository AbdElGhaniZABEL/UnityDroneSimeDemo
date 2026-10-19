function drone_csv_importer(csvFile, outVar, stepSize)

    % INPUT PARAMETERS:
    %   csvFile  - String/Char. The relative or absolute path to the CSV deliverable.
    %   outVar   - String/Char. Target workspace variable name for the output table.
    %   stepSize - Integer. Downsampling interval to optimize memory bandwidth.
    
    % --- 1. Input Validation & Default Parameters ---
    if nargin < 3 || isempty(stepSize)
        stepSize = 1; % Default to full resolution
    end
    if nargin < 2 || isempty(outVar)
        outVar = 'droneData'; % Default workspace variable
    end
    if nargin < 1 || isempty(csvFile)
        error('Actionable feedback: You must provide a valid CSV filename to execute this pipeline.');
    end

    fprintf('Initiating data ingestion protocol for: %s\n', csvFile);

    % --- 2. Sanitize Schema Headers ---
    fid = fopen(csvFile, 'r');
    if fid == -1
        error('File I/O Error: Could not open the file. Please verify path integrity.');
    end
    headerLine = fgetl(fid);
    fclose(fid);
    
    % Strip trailing/inline semicolons and segment by comma
    headerLine = strrep(headerLine, ';', ''); 
    fileHeaders = strtrim(split(headerLine, ',')); 

    % --- 3. Ingest Raw Data ---
    fprintf('Parsing datasets and aligning paradigms...\n');
    % Force comma delimiter to bypass edge-case parsing failures
    T_raw = readtable(csvFile, 'ReadVariableNames', false, 'HeaderLines', 1, 'Delimiter', ',');
    
    % --- 4. Data Cleansing Loop ---
    % Failsafe to convert cell/string arrays corrupted by semicolons back to doubles
    for c = 1:width(T_raw)
        colData = T_raw{:,c};
        if iscell(colData) || isstring(colData)
            cleanedStr = strrep(string(colData), ';', '');
            T_raw.(c) = str2double(cleanedStr); 
        end
    end
    
    % Finalize the schema with sanitized, unique MATLAB-compliant headers
    T_raw.Properties.VariableNames = matlab.lang.makeUniqueStrings(fileHeaders);

    % --- 5. Downsample / Step Size Application ---
    if stepSize > 1
        fprintf('Downsampling data with step size %d to optimize bandwidth...\n', stepSize);
        T_final = T_raw(1:stepSize:end, :);
    else
        T_final = T_raw;
    end
    

    T_final.RotPitch = rad2deg(unwrap(deg2rad(T_final.RotPitch)));
    T_final.RotYaw = rad2deg(unwrap(deg2rad(T_final.RotYaw)));
    
    T_final.DesRotPitch = rad2deg(unwrap(deg2rad(T_final.DesRotPitch)));
    T_final.DesRotYaw = rad2deg(unwrap(deg2rad(T_final.DesRotYaw)));
    
% Recompute roll from quaternions (avoids gimbal-lock flip at ±90°)
qw = T_final.RotQW; qx = T_final.RotQX;
qy = T_final.RotQY; qz = T_final.RotQZ;

T_final.RotRoll = rad2deg(unwrap(atan2(2*(qw.*qx + qy.*qz), ...
                                       1 - 2*(qx.^2 + qy.^2))));

% Desired roll — same formula (assumes DesRotQX/Y/Z/W exist)
if all(ismember({'DesRotQX','DesRotQY','DesRotQZ','DesRotQW'}, T_final.Properties.VariableNames))
    dw = T_final.DesRotQW; dx = T_final.DesRotQX;
    dy = T_final.DesRotQY; dz = T_final.DesRotQZ;
    T_final.DesRotRoll = rad2deg(unwrap(deg2rad(T_final.DesRotRoll)));
else
    % No desired quaternions available — fall back to unwrapping the existing column
    T_final.DesRotRoll = rad2deg(unwrap(deg2rad(T_final.DesRotRoll)));
end
    % --- 6. Push Deliverable to Workspace ---
    assignin('base', outVar, T_final);
    
    fprintf('Mission accomplished: %d rows and %d columns successfully pushed to workspace variable "%s".\n', ...
        height(T_final), width(T_final), outVar);
end