function [s_record] = BT_readRecData(x_fileName)
% function [s_record] = BT_readRecData(x_fileName)
%
% Récupère des données enregistrées dans un fichier au format binaire défini dans REC
% le format "1" est défini dans "REC/doc/recorder.doc-16"
% x_filename - char[] : nom du fichier avec chemin et extension
% s_record - structure
%   s_record.m_var - structure[]
%       s_record.m_var(i).m_name - char[]: nom de la i-ème variable
%       s_record.m_var(i).m_data - double[1XN] : données de la i-ème variable
%   s_record.m_freq - frequence de l'enregistrement
%   s_record.m_format - format de l'enregistrement
%
%see also BT_getRecData, BT_readRecData, BT_getRecFreq, bt_getRecVarList, bt_plotSignals

% initial value (in case of error)
s_record = [];

% l_error is initialized to 0, and is set to 1 when an error occurs
l_error = 0;

% open record file for read
l_fid = fopen(x_fileName, 'r');
if l_fid == -1
    disp('cannot open file');
    l_error = 1;
end

% read the binary format version (l_format)
if l_error == 0
    [l_format,l_count] = fread(l_fid,1,'uint');
    if l_count ~= 1
        disp('problem while reading format');
        l_error = 1;
    end
end

% check the format version
if l_error == 0
    % for the moment, "1", "2" and "3" are supported
    if ((l_format~=1)&(l_format~=2)&(l_format~=3))
        disp(['invalid binary format :' num2str(l_format)]);
        l_error = 1;
    else
        s_record.m_format = l_format;    
    end
end

% read the format #3 informations

if l_error==0
    
    if l_format==3
        % read the arm version
        % read variable name size
        [l_varNameSize,l_count] = fread(l_fid,1,'uint');
        
        if l_count ~= 1
            disp('problem while reading var name size');
            l_error = 1;
        else

            % read variable name (char)
            [l_char,l_count] = fread(l_fid,l_varNameSize,'char');
            s_record.m_armVersion=char(l_char');
            
            if l_count ~= l_varNameSize
                disp('problem while reading var name');
                l_error = 1;
            end

        end
        
        % read the Val3 version
        % read variable name size
        [l_varNameSize,l_count] = fread(l_fid,1,'uint');
        
        if l_count ~= 1
            disp('problem while reading var name size');
            l_error = 1;
        else

            % read variable name (char)
            [l_char,l_count] = fread(l_fid,l_varNameSize,'char');
            s_record.m_val3Version=char(l_char');
            
            if l_count ~= l_varNameSize
                disp('problem while reading var name');
                l_error = 1;
            end

        end

        % read the configuration version
        % read variable name size
        [l_varNameSize,l_count] = fread(l_fid,1,'uint');
        
        if l_count ~= 1
            disp('problem while reading var name size');
            l_error = 1;
        else

            % read variable name (char)
            [l_char,l_count] = fread(l_fid,l_varNameSize,'char');
            s_record.m_configurationVersion=char(l_char');
            
            if l_count ~= l_varNameSize
                disp('problem while reading var name');
                l_error = 1;
            end

        end

        % read the machine number
        % read variable name size
        [l_varNameSize,l_count] = fread(l_fid,1,'uint');
        
        if l_count ~= 1
            disp('problem while reading var name size');
            l_error = 1;
        else

            % read variable name (char)
            [l_char,l_count] = fread(l_fid,l_varNameSize,'char');
            s_record.m_machineNumber=char(l_char');
            
            if l_count ~= l_varNameSize
                disp('problem while reading var name');
                l_error = 1;
            end

        end
        
    end
    
end

% read record frequency (format # 2 et # 3)
if l_error == 0
    if ((l_format==2)|(l_format==3))
        [l_freq,l_count] = fread(l_fid,1,'double');
        if l_count ~= 1
            disp('problem while reading frequency');
            l_error = 1;
        else
            s_record.m_freq = l_freq;
        end
    end
end
   
% read the number of variables
if l_error == 0
    [l_nbVar,l_count] = fread(l_fid,1,'uint');
    if l_count ~= 1
        disp('problem while reading nb var');
        l_error = 1;
    end
end

% read number of samples
if l_error == 0
    [l_nbSample,l_count] = fread(l_fid,1,'uint');
    if l_count ~= 1
        disp('problem  while reading number of samples');
        l_error = 1;
    end;
end;
        
% read each variable name    
l_varIndex = 1;
while (l_error == 0) & (l_varIndex <= l_nbVar)
    % read variable name size
    [l_varNameSize,l_count] = fread(l_fid,1,'uint');
    if l_count ~= 1
        disp('problem while reading var name size');
        l_error = 1;
    else
        % read variable name (char)
        [l_char,l_count] = fread(l_fid,l_varNameSize,'char');
        s_record.m_var(l_varIndex).m_name = char(l_char');
        if l_count ~= l_varNameSize
            disp('problem while reading var name');
            l_error = 1;
        else
           % read variable index (int) for format #2 and #3
           if ((l_format==2)|(l_format==3))
              [l_index,l_count] = fread(l_fid,1,'int');
              s_record.m_var(l_varIndex).m_index = l_index;
              if l_count ~= 1
                 disp('problem while reading var index');
                 l_error = 1;
              end;
           end;
        end
    end
    l_varIndex = l_varIndex + 1;
end

% read variable data
if l_error == 0
    %read raw data
    [l_rawData,l_count] = fread(l_fid,l_nbVar*l_nbSample,'double');
    if l_count ~= l_nbVar*l_nbSample
        disp(['incorrect number of variables read!:' num2str(l_count) 'and should be:' num2str(l_nbVar*l_nbSample)]);
        % initialize empty value field
        for l_varIndex = 1:l_nbVar
            s_record.m_var(l_varIndex).m_value = [];
        end
        l_error = 1;
    else
        l_rawData = reshape(l_rawData,l_nbVar,l_nbSample);
        % for each variable
        for l_varIndex = 1:l_nbVar
            s_record.m_var(l_varIndex).m_value = l_rawData(l_varIndex,:)';
        end
    end
end

% if file could be opened
if l_fid > 0
    fclose(l_fid);
end;