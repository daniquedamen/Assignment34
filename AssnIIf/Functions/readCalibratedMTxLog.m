function data = readCalibratedMTxLog(file)

path = 'Measurements/';
fid = fopen([path file]);

tline = '';

while isempty(regexp(tline,'Counter', 'once'))
    tline = fgetl(fid);
    header = regexp(tline,'\w+\s\w+\:','match');
    
    if (strcmp(header,'Sample rate:'))
        data = regexp(tline,'(?<fs>\d+\.\d+)','names');
    end
    if (strcmp(header,'Update Rate:'))
        data = regexp(tline,'(?<fs>\d+\.\d+)','names');
    end
end

header = regexp(tline,'\t','split');
numcols = length(header)-1;

values = textscan(fid, repmat('%f',1,numcols), 'delimiter', '\t');%, 'HeaderLines',4);

data.fs = str2double(data.fs);
data.N = length(values{1});
fclose(fid);

data.counter = values{1};

if (data.counter(end)-data.counter(1)+1 ~= data.N)
    warning('Samples missing');
end

data.acc = [values{2} values{3} values{4}];
data.gyr = [values{5} values{6} values{7}];
data.mag = [values{8} values{9} values{10}];
data.Rsg(:,1,1:3) = [values{11} values{12} values{13}];
data.Rsg(:,2,1:3) = [values{14} values{15} values{16}];
data.Rsg(:,3,1:3) = [values{17} values{18} values{19}];