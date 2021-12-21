function [signal,time] = loadbin(filename)
  FID = fopen(filename, 'rb');
  if FID == -1
    disp('ERROR: Unable to open file!')
    return
  end

  DATA = fread(FID, [1, Inf], 'double');

  signal = DATA(3:end)';

  xIncr = DATA(1);
  xZero = DATA(2);
  time = (0:length(signal)-1)';
  time = time.*xIncr;
  time = time .+ xZero;

  fclose(FID);
end
