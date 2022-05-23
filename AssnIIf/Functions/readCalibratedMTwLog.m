function data = readCalibratedMTwLog(filename)
% data = readCalibratedMTwLog(filename)
% read mtw exported txt file

% Henk Kortier Jan 2012

path = 'Measurements/';
fid = fopen([path filename]);

if fid == -1
    disp('invalid mtw file');
    
else

data.StartTime = str2double(regexp(fgetl(fid),'[0-9]+','match'));
data.fs = str2double(regexp(fgetl(fid),'[0-9]+\.[0-9]+','match'));
data.Scenario = regexp(fgetl(fid),'[0-9]+\.[0-9]','match');     
data.FW = regexp(fgetl(fid),'[0-9]+\.[0-9]+\.[0-9]+','match');

tline = fgetl(fid);
while isempty(regexp(tline,'Counter', 'once'))
    tline = fgetl(fid);
    header = regexp(tline,'\w+\s\w+\:','match');
    
%    if (strcmp(header,'Sample rate:'))
    data = regexp(tline,'(?<fs>\d+\.\d+)','names');
%    end
end

header = regexp(tline,'\t','split');
numcols = length(header)-1;

values = textscan(fid, repmat('%f',1,numcols), 'delimiter', '\t');%, 'HeaderLines',4);

fclose(fid);

data.N = length(values{1});
data.counter = values{1};

%GIVES ERROR IF Nsamples>16 bits
%if (data.counter(end)-data.counter(1)+1 ~= data.N)
%    error('Samples missing');
%end

for i = 1:length(values)
    %Remove first 0.25s at start and end samples
    ncut = round(0.5*data.fs);
    values{i} = values{i}(ncut:end-ncut);
    
    isnanValues = isnan(values{i});
    if sum(isnanValues)>0
        values{i}(isnanValues) = 0;        
        %disp('NaN values set to zero
    end    
end

data.N = length(values{1});
data.counter = values{1};
data.header{1} = 'counter';

for i = 1:numcols
    if strcmp(header{i},'Acc_X')
        data.acc = [values{i} values{i+1} values{i+2}];
        data.header{end+1} = 'acc';
    end
    
    if strcmp(header{i},'Gyr_X')
        data.gyr = [values{i} values{i+1} values{i+2}];
        data.header{end+1} = 'gyr';
    end
    
    if strcmp(header{i},'Mag_X')
        data.mag = [values{i} values{i+1} values{i+2}];
        data.header{end+1} = 'mag';
    end
    
    if strcmp(header{i},'Quat_w')
        data.qgs = [values{i} values{i+1} values{i+2} values{i+3}];
        data.header{end+1} = 'qgs';
    end
    
    if strcmp(header{i},'Mat[1][1]')
%         warning('check if first column or rows')
        data.Rgs=zeros(data.N,3,3);
        data.Rsg=zeros(data.N,3,3);
        data.Rsg(:,:,1)=[values{i} values{i+1} values{i+2} ];
        data.Rsg(:,:,2)=[values{i+3} values{i+4} values{i+5} ];
        data.Rsg(:,:,3)=[values{i+6} values{i+7} values{i+8} ];
            data.Rgs(:,1,:)=[values{i} values{i+1} values{i+2} ];
            data.Rgs(:,2,:)=[values{i+3} values{i+4} values{i+5} ];
            data.Rgs(:,3,:)=[values{i+6} values{i+7} values{i+8} ];
        data.header{end+1} = 'RgsXsens';
    end
    
    if strcmp(header{i},'OriInc_w')
        data.dqgs = [values{i} values{i+1} values{i+2} values{i+3}];
        data.header{end+1} = 'dqgs';
    end
    
    if strcmp(header{i},'VelInc_X')
        data.dv = [values{i} values{i+1} values{i+2}];
        data.header{end+1} = 'dv';
    end
    
    if strcmp(header{i},'Temperature')
        data.temp = values{i};
        data.header{end+1} = 'temp';
    end
    
    if strcmp(header{i},'Pressure')
        data.pres = values{i};
        data.header{end+1} = 'pres';
    end
    
end

end
%data.gyr = qrot(-data.gyr,data.qsg);
%data.gyr =  -data.gyr;
%data.Acc = [values{9} values{10} values{11}];
%data.Gyr = [values{12} values{13} values{14}];
%data.Mag = [values{15} values{16} values{17}];
%data.qsg = [values{19} values{20} values{21} values{22}];
%data.P = [values{18}];