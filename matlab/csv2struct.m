function [data, csvData, elemLogOrder] = csv2struct(csv)
    file = fopen(csv, 'r');
    
    newelemName=[];
    elemLogOrder={};   
    data = struct;
    
    while ~feof(file)
        line = fgetl(file);      
        if (line(1) ~= '%') break; end;
        
        newElement = findstr('%element:', line);
        if (~isempty(newElement))
            newelemName = strtrim(line((10:end)));
            data = setfield(data, newelemName, struct);
            disp(strcat('New element:',newelemName));
        else
            if (~isempty(newelemName))
                newPart = strtrim(line((2:end)));
                %elemList = regexp(newPart, '\.', 'split');
                %data.(newelemName) = recurse_struct(data.(newelemName),elemList, 0);
                elemLogOrder=[elemLogOrder {newelemName; newPart}];
            end 
        end
    end
    fclose(file);
        
    csvData = load(csv)';
    for i=1:length(elemLogOrder)
        newelemName = elemLogOrder{1,i};
        elemList = regexp(elemLogOrder{2,i}, '\.', 'split');
        data.(newelemName) = recurse_struct(data.(newelemName), elemList, csvData(i,:));
    end
end

function data = recurse_struct(data, elemList, value)
    if isempty(elemList)
        data = value;
        return
    end
    
    if ~isfield(data,elemList{1})
        data.(elemList{1}) = struct;
    end
    data.(elemList{1}) = recurse_struct(data.(elemList{1}), elemList(2:end), value);
end
    