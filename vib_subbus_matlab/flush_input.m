function flush_input(s)
while true
  tline = fgetl(s);
  if isempty(tline)
    break;
  end
  fprintf(1,'ERROR: Unexpected input: ''%s''\n', tline);
end
