function [output] = tmp(inputStruct)

score = @(val) 1-1/(1+exp(-(val-.05)*100));
output = struct('error_text',{},'help_score',0,'effort',0,'output_error',[],'timing_performance',[]);

output(1).percentError = -1;
if ~exist('dhInvKine.m','file')
	output.error_text{end+1} = 'Function dhInvKine cannot be found.';
	return;
end
[~,name,~] = fileparts(which('dhInvKine.m'));
caseMatch = strcmp(name,'dhInvKine');
if ~caseMatch
	output.error_text{end+1} = ['Case sensitive match for dhInvKine does not exist, you used ' name]
	return;
end
T = evalc('help dhInvKine.m');

output(1).help_score = 0;
if strcmp('No help found for dhInvKine.m.',strtrim(deblank(T)));
	output.error_text{end+1} = 'No helpfile Provided.';
else
	if numel(T) < 338
		output.error_text{end+1} = 'Help File Data Insufficient.';
		output.help_score = score(max([338-numel(T),0])/5000);
	else
		output.help_score = 1;
	end
end

fileID = fopen('dhInvKine.m');
numChar = numel(fread(fileID,'char'))-numel(T);
fclose(fileID);
output.effort = numChar > 1.714000e+02;


try
     [O1] = dhInvKine(inputStruct.Inputs.I1, inputStruct.Inputs.I2, inputStruct.Inputs.I3);

	output.percentError = zeros(1,1);


	useAlt = false;
	if all(size(O1)==size(inputStruct.Outputs.O1))
		if isstruct(inputStruct.Outputs.O1)
			name_list = fieldnames(inputStruct.Outputs.O1);
			name_check = isfield(O1,name_list);
			for i=1:length(name_check)
				if ~name_check(i)
					output.error_text{end+1} = [name_list{i} ' is missing in struct'];
				end
			end
			output.percentError = 1-sum(name_check)/length(name_check);

			if all(name_check)
				for i=1:length(name_check)
				
					ref = getfield(inputStruct.Outputs.O1,name_list{i});
					val = getfield(O1,name_list{i});
					pe = 0;
					if isempty(ref)
						pe = ~isempty(val)*100;
					elseif any(isnumeric(ref) & abs(ref) > 0)
						pe = 100*norm(val-ref)/norm(ref);
					elseif islogical(ref)
						pe = (ref ~= val)*100;
					else
						pe = 100*norm(val-ref);
					end
					output.percentError(1) = max(output.percentError(1),pe);
					if pe > 1e-3
						output.error_text{end+1} = [name_list{i} ' is incorrect. Percent error: ' num2str(pe,2) '%'];
					end
				end
			end
		elseif norm(inputStruct.Outputs.O1) >0
			output.percentError(1) = 100*norm(O1-inputStruct.Outputs.O1)/norm(inputStruct.Outputs.O1);
			if isfield(inputStruct.Outputs,'O1Alt')
				e_alt = 100*norm(O1-inputStruct.Outputs.O1Alt)/norm(inputStruct.Outputs.O1Alt);
				if all(e_alt<output.percentError(1))
					output.percentError(1) = e_alt;
					useAlt = true;
				end
			end
		else
			output.percentError(1) = 100*norm(O1-inputStruct.Outputs.O1);
			if isfield(inputStruct.Outputs,'O1Alt')
				e_alt = 100*norm(O1-inputStruct.Outputs.O1Alt)/norm(inputStruct.Outputs.O1Alt);
				if all(e_alt<output.percentError(1))
					output.percentError(1) = e_alt;
					useAlt = true;
				end
			end
		end
	else
		output.percentError(1) = -1;
		output.error_text{end+1} = ['Output Number 1 has the wrong size is: [' int2str(size(O1)) '] shoudl be [' int2str(size(inputStruct.Outputs.O1)) ']'];
	end


	warning('off','all');

	I1 = inputStruct.Inputs.I1;
	I2 = inputStruct.Inputs.I2;
	I3 = inputStruct.Inputs.I3;
	N_times=5; x = zeros(N_times,1);for c=1:length(x), x(c) = timeit(@()dhInvKine(I1, I2, I3),1);end
	output.timing_performance = mean(x);
	output.timing_performance_stdev = std(x);
	warning('on','all');

catch ME
	output.error_text{end+1} = [ME.message, '  Line ' , int2str(ME.stack(1).line) , ' in ' ME.stack(1).name];
end

end

