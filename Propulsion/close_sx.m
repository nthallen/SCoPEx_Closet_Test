hw=instrfind;
num_ports = size(hw,2);
for ii=1:num_ports
  if(hw(ii).status(1) == 'o')
    fclose(hw(ii));
  end
end
hw=instrfind;